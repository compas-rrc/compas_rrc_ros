#!/usr/bin/env python
import select
import socket
import threading
import time

import rospy
from compas_rrc_driver.event_emitter import EventEmitterMixin
from compas_rrc_driver.protocol import WireProtocol
from compas_rrc_driver.topics import RobotMessageTopicProvider

try:
    import Queue as queue
except ImportError:
    import queue

CONNECTION_TIMEOUT = 5              # In seconds
QUEUE_TIMEOUT = 5                   # In seconds
RECONNECT_DELAY = 10                # In seconds
SOCKET_SELECT_TIMEOUT = 60 * 10     # In seconds
QUEUE_MESSAGE_TOKEN = 0
QUEUE_TERMINATION_TOKEN = -1
QUEUE_RECONNECTION_TOKEN = -2
SOCKET_MODE_SERVER = 1
SOCKET_MODE_CLIENT = 2

class SocketManager(EventEmitterMixin):
    def __init__(self, host, port, socket_mode):
        super(SocketManager, self).__init__()
        self.socket_mode = socket_mode

        self.host = host
        self.port = port

    # TODO: Check if these options really can be shared on both client and server modes
    def _set_socket_opts(self, sock):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 6)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)

    def _create_socket_server(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(0)
        sock.bind((self.host, self.port))
        self._set_socket_opts(sock)

        # Non-blocking listen with 5 backlog connections
        sock.listen(5)

        return sock

    def _create_socket_client(self):
        sock = socket.create_connection((self.host, self.port), CONNECTION_TIMEOUT)
        self._set_socket_opts(sock)

        return sock

    def _create_socket(self):
        if self.socket_mode == SOCKET_MODE_CLIENT:
            return self._create_socket_client()
        elif self.socket_mode == SOCKET_MODE_SERVER:
            return self._create_socket_server()
        else:
            raise ValueError('Invalid socket mode')

    def on_socket_broken(self, callback):
        """Add an event handler to be triggered when the socket is broken."""
        self.on('socket_broken', callback)

class CurrentMessage(object):
    def __init__(self):
        self.clear()

    @property
    def state(self):
        if len(self.header) < WireProtocol.FIXED_HEADER_LEN:
            return 'recv_header'

        if len(self.header) == WireProtocol.FIXED_HEADER_LEN:
            if self.remaining_payload_bytes > 0:
                return 'recv_payload'
            elif self.remaining_payload_bytes == 0:
                return 'message_complete'
            else:
                raise Exception('Payload exceeds expected length. Header={}, Payload={}'.format(self.header, self.payload))

    @property
    def protocol_version(self):
        return WireProtocol.get_protocol_version(self.header)

    @property
    def payload_length(self):
        message_length = WireProtocol.get_message_length(self.header)
        payload_length = message_length - WireProtocol.FIXED_HEADER_LEN
        return payload_length

    @property
    def remaining_payload_bytes(self):
        return self.payload_length - len(self.payload)

    @property
    def remaining_header_bytes(self):
        return WireProtocol.FIXED_HEADER_LEN - len(self.header)

    def append_header_chunk(self, chunk):
        if not chunk:
            raise socket.error('Socket broken, header chunk empty')

        self.header += chunk

    def append_payload_chunk(self, chunk):
        self.payload += chunk

    def clear(self):
        self.header = b''
        self.payload = b''

    def deserialize(self):
        return WireProtocol.deserialize(self.header, self.payload)


class RobotStateConnection(SocketManager):
    def __init__(self, host, port, socket_mode):
        super(RobotStateConnection, self).__init__(host, port, socket_mode)
        self.is_running = False

    def on_message(self, callback):
        """Add an event handler to be triggered on message arrival."""
        self.on('message', callback)

    def connect(self):
        self.is_running = True
        self._connect_socket()

        self.thread = threading.Thread(target=self.socket_worker, name='robot_state_socket')
        self.thread.daemon = True
        self.thread.start()

    def disconnect(self):
        self.is_running = False
        self.thread.join(CONNECTION_TIMEOUT)
        self._disconnect_socket()

    def _connect_socket(self):
        try:
            rospy.loginfo('Robot state: Creating socket %s:%d', self.host, self.port)
            self.socket = self._create_socket()

            rospy.loginfo('Robot state: Socket created')
        except:
            rospy.logerr('Cannot create robot state socket: %s:%d', self.host, self.port)
            raise

    def _disconnect_socket(self):
        rospy.loginfo('Robot state: Disconnecting socket')
        if self.socket:
            self.socket.close()

    def socket_worker(self):
        rospy.loginfo('Robot state: Worker started')
        current_message = CurrentMessage()
        version_already_checked = False
        inputs = []
        if self.socket:
            inputs.append(self.socket)

        while self.is_running:
            try:
                if not self.socket:
                    self._connect_socket()
                    inputs.append(self.socket)

                readable, _, failed = select.select(inputs, inputs, inputs, SOCKET_SELECT_TIMEOUT)
                rospy.logdebug('Readable Socket selected, state={}, len current header={}, len current payload={}'.format(current_message.state, len(current_message.header), len(current_message.payload)))

                for fsocket in failed:
                    if fsocket in inputs:
                        inputs.remove(fsocket)
                        rospy.loginfo('Removing socket from inputs, len={}'.format(len(inputs)))

                # if len(failed) > 0:
                #     raise socket.error('No readable socket available')

                # if len(readable) == 0:
                #     raise socket.timeout('Socket selection timed out')

                # TODO: Check this
                if len(readable) > 1:
                    raise Exception('Why do we have more than one readable socket? Something is off')

                rsocket = readable[0]

                # If it's server mode, accept connection
                if rsocket == self.socket:
                    connection, client_addr = self.socket.accept()
                    rospy.loginfo('Robot state: Incoming connection from client {}'.format(client_addr))
                    connection.setblocking(0)
                    inputs.append(connection)
                    continue

                # Start receiving bytes
                if current_message.state == 'recv_header':
                    header_chunk = rsocket.recv(current_message.remaining_header_bytes)
                    current_message.append_header_chunk(header_chunk)
                    continue

                # We have a full header, we can proceed with payload
                if current_message.state == 'recv_payload':
                    # Ensure incoming version check matches
                    if not version_already_checked:
                        server_protocol_version = current_message.protocol_version

                        if WireProtocol.VERSION != server_protocol_version:
                            raise Exception('Protocol version mismatch: Server={}, Client={}'.format(server_protocol_version, WireProtocol.VERSION))

                        version_already_checked = True

                    # TODO: Add log to trace message fragmentation scenario
                    chunk = rsocket.recv(current_message.remaining_payload_bytes)

                    try:
                        if not chunk:
                            rospy.logdebug('Nothing read in chuck recv, will continue')
                            continue

                        current_message.append_payload_chunk(chunk)

                        if current_message.state == 'message_complete':
                            message = current_message.deserialize()
                            # Emit global and individual events
                            self.emit('message', message)
                            self.emit(WireProtocol.get_response_key(message), message)
                            current_message.clear()

                    except Exception as me:
                        rospy.logerr('Exception while recv/deserialization of a message, skipping message. Exception=%s', str(me))
                        rospy.logerr(str(current_message.payload))
                        current_message.clear()

            except socket.timeout:
                # The socket has a timeout, so that it does not block on recv()
                # If it times out, it's ok, we just continue and re-start receiving
                rospy.logdebug('Robot state: Socket timeout, will retry to select socket')
            except socket.error:
                if self.is_running:
                    self.socket = None
                    self.emit('socket_broken')

                    rospy.logwarn('Robot state: Disconnection detected, waiting %d sec before reconnect...', RECONNECT_DELAY)
                    time.sleep(RECONNECT_DELAY)
            except Exception as e:
                error_message = 'Exception on robot state interface: {}'.format(str(e))
                rospy.logerr(error_message)
                rospy.signal_shutdown(error_message)
                break

        rospy.loginfo('Robot state: Worker stopped')


class StreamingInterfaceConnection(SocketManager):
    def __init__(self, host, port, socket_mode):
        super(StreamingInterfaceConnection, self).__init__(host, port, socket_mode)
        self.is_running = False

        self.queue = queue.Queue()
        self.thread = None
        self.socket = None

    def on_message_sent(self, callback):
        """Add an event handler to be triggered on message sent."""
        self.on('message_sent', callback)

    def connect(self):
        self.is_running = True
        self._connect_socket()

        self.thread = threading.Thread(target=self.socket_worker, name='streaming_interface_socket')
        self.thread.daemon = True
        self.thread.start()

    def disconnect(self):
        if self.is_running:
            self.is_running = False

        if self.queue:
            self.queue.put((QUEUE_TERMINATION_TOKEN, None))

        if self.thread:
            self.thread.join(CONNECTION_TIMEOUT)

        self._disconnect_socket()

    def reconnect(self):
        if self.queue:
            self.queue.put((QUEUE_RECONNECTION_TOKEN, time.time()))

    def _connect_socket(self):
        try:
            rospy.loginfo('Streaming interface: Creating socket %s:%d', self.host, self.port)
            self.socket = self._create_socket()

            rospy.loginfo('Streaming interface: Socket created')
        except:
            rospy.logerr('Cannot create streaming interface socket: %s:%d', self.host, self.port)
            raise

    def _disconnect_socket(self):
        rospy.loginfo('Streaming interface: Disconnecting socket')
        if self.socket:
            self.socket.close()

    def execute_instruction(self, message):
        # TODO: RAPID side does not (yet) instruct the client to disconnect
        # if message.instruction == 'exit':
        #     rospy.loginfo('Received exit instruction')
        #     ... invoke service disconnection
        #     return

        self.queue.put((QUEUE_MESSAGE_TOKEN, message))

    def socket_worker(self):
        rospy.loginfo('Streaming interface: Worker started')
        last_successful_connect = None
        streaming_sockets = []
        if self.socket:
            streaming_sockets.append(self.socket)

        while self.is_running:
            try:
                if not self.socket:
                    self._connect_socket()
                    streaming_sockets.append(self.socket)
                    last_successful_connect = time.time()

                # rospy.loginfo('Streaming interface waiting for socket, len={}'.format())
                readable, writable, failed = select.select(streaming_sockets, streaming_sockets, streaming_sockets)
                # rospy.loginfo('Streaming interface socket available: streaming_sockets={}, r={}, w={}, f={}'.format(len(streaming_sockets), len(readable), len(writable), len(failed)))

                for fsocket in failed:
                    if fsocket in streaming_sockets:
                        streaming_sockets.remove(fsocket)
                        rospy.loginfo('Removing socket from streaming_sockets, len={}'.format(len(streaming_sockets)))

                # if len(writable) == 0:
                #     raise Exception('No writable socket available')

                for wsocket in readable:
                    # If it's server mode, accept connection
                    if wsocket == self.socket:
                        connection, client_addr = self.socket.accept()
                        rospy.loginfo('Streaming interface: Incoming connection from client {}'.format(client_addr))
                        connection.setblocking(0)
                        streaming_sockets.append(connection)
                        continue

                # TODO: Check if we can lower the timeout to make sure we respond faster to failed socket
                token_type, message = self.queue.get(block=True, timeout=QUEUE_TIMEOUT)

                if token_type == QUEUE_MESSAGE_TOKEN:
                    wire_message = WireProtocol.serialize(message)
                    # TODO: Check this
                    if len(writable) > 1:
                        raise Exception('Why do we have more than one writable socket? Something is off')

                    for wsocket in writable:
                        rospy.loginfo('Sending on writable socket of {}'.format(len(writable)))
                        sent_bytes = wsocket.send(wire_message)

                    if sent_bytes == 0:
                        raise socket.error('Streaming socket connection broken')

                    self.emit('message_sent', message, wire_message)
                elif token_type == QUEUE_TERMINATION_TOKEN:
                    rospy.loginfo('Signal to terminate, closing socket')
                    # TODO: RAPID side does not yet support graceful shutdown
                    # SOCKET_CLOSE_COMMAND = 'stop\r\n'
                    # self.socket.send(SOCKET_CLOSE_COMMAND)
                    break
                elif token_type == QUEUE_RECONNECTION_TOKEN:
                    reconnection_timestamp = message
                    if reconnection_timestamp > last_successful_connect:
                        raise socket.error('Reconnection requested at {}'.format(message))
                    else:
                        rospy.loginfo('Ignoring stale reconnection request issued at {} because last successful connection was at {}'.format(
                            reconnection_timestamp, last_successful_connect))
                else:
                    raise Exception('Unknown token type')
            except queue.Empty:
                pass
            except socket.error:
                if self.is_running:
                    self.socket = None
                    self.emit('socket_broken')

                    rospy.logwarn('Streaming interface: Disconnection detected, waiting %d sec before reconnect...', RECONNECT_DELAY)
                    time.sleep(RECONNECT_DELAY)
            except Exception as e:
                error_message = 'Exception on streaming interface worker: {}'.format(str(e))
                rospy.logerr(error_message)
                rospy.signal_shutdown(error_message)
                break
        rospy.loginfo('Streaming interface: Worker stopped')


def main():
    DEBUG = True
    TOPIC_MODE = 'message'

    log_level = rospy.DEBUG if DEBUG else rospy.INFO
    rospy.init_node('compas_rrc_driver', log_level=log_level)

    robot_host = rospy.get_param('robot_ip_address')
    bind_ip_address = rospy.get_param('bind_ip_address')
    robot_streaming_port = rospy.get_param('robot_streaming_port')
    robot_state_port = rospy.get_param('robot_state_port')
    sequence_check_mode = rospy.get_param('sequence_check_mode')

    if robot_host and not bind_ip_address:
        socket_mode = SOCKET_MODE_CLIENT
        socket_mode_name = 'client mode'
        operation_text = 'connecting to robot'
        host = robot_host
    else:
        socket_mode = SOCKET_MODE_SERVER
        socket_mode_name = 'server mode'
        operation_text = 'listening on'
        host = bind_ip_address

    rospy.loginfo('Starting RRC {}, {} {}:[{},{}]...'.format(socket_mode_name, operation_text, host, robot_streaming_port, robot_state_port))
    rospy.loginfo('Sequence check mode={}'.format(sequence_check_mode))

    # Set protocol version in a parameter to enable version checks from the client side
    rospy.set_param('protocol_version', WireProtocol.VERSION)

    streaming_interface = None
    robot_state = None
    topic_provider = None

    try:
        rospy.loginfo('Connecting robot %s (ports %d & %d, sequence check mode=%s)', robot_host, robot_streaming_port, robot_state_port, sequence_check_mode)
        streaming_interface = StreamingInterfaceConnection(robot_host, robot_streaming_port, socket_mode)
        streaming_interface.connect()

        robot_state = RobotStateConnection(robot_host, robot_state_port, socket_mode)
        robot_state.connect()

        # If a disconnect is detected on the robot state socket, it will try to reconnect
        # So we notify the streaming interface to do the same
        robot_state.on_socket_broken(streaming_interface.reconnect)

        def message_received_log(message):
            rospy.logdebug('Received: "%s", content: %s', message.feedback, str(message).replace('\n', '; '))
            rospy.loginfo('Received message: feedback=%s, sequence_id=%d, feedback_id=%d', message.feedback, message.sequence_id, message.feedback_id)

        def message_sent_log(message, wire_message):
            rospy.logdebug('Sent: "%s", content: %s', message.instruction, str(message).replace('\n', '; '))
            rospy.loginfo('Sent message with length=%d, instruction=%s, sequence id=%d', len(wire_message), message.instruction, message.sequence_id)

        streaming_interface.on_message_sent(message_sent_log)
        if DEBUG:
            robot_state.on_message(message_received_log)

        if TOPIC_MODE == 'message':
            options = dict(sequence_check_mode=sequence_check_mode)
            topic_provider = RobotMessageTopicProvider('robot_command', 'robot_response', streaming_interface, robot_state, options=options)

        rospy.spin()
    finally:
        if topic_provider:
            rospy.loginfo('Disconnecting topic provider...')
            topic_provider.disconnect()

        if streaming_interface:
            rospy.loginfo('Disconnecting streaming interface...')
            streaming_interface.disconnect()

        if robot_state:
            rospy.loginfo('Disconnecting robot state...')
            robot_state.disconnect()

    rospy.loginfo('Terminated')


if __name__ == '__main__':
    main()
