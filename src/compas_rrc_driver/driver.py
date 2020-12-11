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

CONNECTION_TIMEOUT = 5          # In seconds
QUEUE_TIMEOUT = 5               # In seconds
RECONNECT_DELAY = 10            # In seconds
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
        current_header = b''
        current_payload = b''
        version_already_checked = False
        inputs = []
        if self.socket:
            inputs.append(self.socket)

        while self.is_running:
            try:
                if not self.socket:
                    self._connect_socket()
                    inputs.append(self.socket)

                # rospy.loginfo('Robot state waiting for socket, len={}'.format(len(inputs)))
                readable, _w, failed = select.select(inputs, inputs, inputs)
                # rospy.loginfo('Robot state socket available: inputs={}, r={}, w={}, f={}'.format(len(inputs), len(readable), len(_w), len(failed)))

                for fsocket in failed:
                    if fsocket in inputs:
                        inputs.remove(fsocket)
                        rospy.loginfo('Removing socket from inputs, len={}'.format(len(inputs)))

                # if len(failed) > 0:
                #     raise socket.error('No readable socket available')

                # if len(readable) == 0:
                #     raise socket.timeout('Socket selection timed out')

                for rsocket in readable:
                    # If it's server mode, accept connection
                    if rsocket == self.socket:
                        connection, client_addr = self.socket.accept()
                        rospy.loginfo('Robot state: Incoming connection from client {}'.format(client_addr))
                        connection.setblocking(0)
                        inputs.append(connection)
                        continue

                    if len(current_header) < WireProtocol.FIXED_HEADER_LEN:
                        header_chunk = rsocket.recv(WireProtocol.FIXED_HEADER_LEN)

                        if not header_chunk:
                            raise socket.error('Robot state socket broken')

                        current_header += header_chunk
                        continue

                    # We have a full header, we can proceed with payload
                    if len(current_header) == WireProtocol.FIXED_HEADER_LEN:
                        # Ensure incoming version check matches
                        if not version_already_checked:
                            server_protocol_version = WireProtocol.get_protocol_version(current_header)

                            if WireProtocol.VERSION != server_protocol_version:
                                raise Exception('Protocol version mismatch: Server={}, Client={}'.format(server_protocol_version, WireProtocol.VERSION))

                            version_already_checked = True

                        message_length = WireProtocol.get_message_length(current_header)
                        chunk = rsocket.recv(1024)

                        try:
                            if not chunk:
                                rospy.logdebug('Nothing read in chuck recv, will continue')
                                continue

                            current_payload += chunk
                            if len(current_payload) == message_length - WireProtocol.FIXED_HEADER_LEN:
                                message = WireProtocol.deserialize(current_header, current_payload)
                                # Emit global and individual events
                                self.emit('message', message)
                                self.emit(WireProtocol.get_response_key(message), message)
                                current_payload = b''

                        except Exception as me:
                            rospy.logerr('Exception while recv/deserialization of a message, skipping message. Exception=%s', str(me))
                            current_payload = b''
                        finally:
                            # Ready for next
                            current_header = b''

            except socket.timeout:
                rospy.loginfo('Socket timed out')
                # The socket has a timeout, so that it does not block on recv()
                # If it times out, it's ok, we just continue and re-start receiving
                pass
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
        outputs = []
        if self.socket:
            outputs.append(self.socket)

        while self.is_running:
            try:
                if not self.socket:
                    self._connect_socket()
                    outputs.append(self.socket)
                    last_successful_connect = time.time()

                # rospy.loginfo('Streaming interface waiting for socket, len={}'.format())
                readable, writable, failed = select.select(outputs, outputs, outputs)
                # rospy.loginfo('Streaming interface socket available: outputs={}, r={}, w={}, f={}'.format(len(outputs), len(readable), len(writable), len(failed)))

                for fsocket in failed:
                    if fsocket in outputs:
                        outputs.remove(fsocket)
                        rospy.loginfo('Removing socket from outputs, len={}'.format(len(outputs)))

                # if len(writable) == 0:
                #     raise Exception('No writable socket available')

                for wsocket in readable:
                    # If it's server mode, accept connection
                    if wsocket == self.socket:
                        connection, client_addr = self.socket.accept()
                        rospy.loginfo('Streaming interface: Incoming connection from client {}'.format(client_addr))
                        connection.setblocking(0)
                        outputs.append(connection)
                        continue

                token_type, message = self.queue.get(block=True, timeout=QUEUE_TIMEOUT)

                if token_type == QUEUE_MESSAGE_TOKEN:
                    wire_message = WireProtocol.serialize(message)
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
        streaming_interface = StreamingInterfaceConnection(host, robot_streaming_port, socket_mode)
        streaming_interface.connect()

        robot_state = RobotStateConnection(host, robot_state_port, socket_mode)
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
