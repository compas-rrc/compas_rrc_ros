#!/usr/bin/env python
import select
import socket
import threading
import time
import timeit
import queue

import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy import logging

from compas_rrc_driver.event_emitter import EventEmitterMixin
from compas_rrc_driver.protocol import WireProtocol
from compas_rrc_driver.topics import RobotMessageTopicProvider, SequenceCheckModes


CONNECTION_TIMEOUT = 5              # In seconds
QUEUE_TIMEOUT = 5                   # In seconds
RECONNECT_DELAY = 10                # In seconds
SOCKET_SELECT_TIMEOUT = 10          # In seconds
QUEUE_MESSAGE_TOKEN = 0
QUEUE_TERMINATION_TOKEN = -1
QUEUE_RECONNECTION_TOKEN = -2
START_PROCESS_TIME = timeit.default_timer()
TIMING_START = dict()

LOGGER = logging.get_logger('compas_rrc_driver')


def _set_socket_opts(sock):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    # The following options make sure keep alive is active, otherwise the socket closes after about 5 or 6 minutes
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 6)


def _get_perf_counter():
    secs = timeit.default_timer() - START_PROCESS_TIME
    return int(secs * 1000)


def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass

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
        else:
            raise Exception('Header exceeds expected length. Header={}, Payload={}'.format(self.header, self.payload))

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
        self.add_perf_marker('header_recv')

    def append_payload_chunk(self, chunk):
        self.payload += chunk
        self.add_perf_marker('payload_recv')

    def clear(self):
        self.header = b''
        self.payload = b''
        self.timing = list()

    def deserialize(self):
        return WireProtocol.deserialize(self.header, self.payload)

    def add_perf_marker(self, label):
        self.timing.append((label, _get_perf_counter()))


class RobotStateConnection(EventEmitterMixin):
    def __init__(self, host, port):
        super(RobotStateConnection, self).__init__()
        self.is_running = False
        self.host = host
        self.port = port

    def on_message(self, callback):
        """Add an event handler to be triggered on message arrival."""
        self.on('message', callback)

    def on_socket_broken(self, callback):
        """Add an event handler to be triggered when the socket is broken."""
        self.on('socket_broken', callback)

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
            LOGGER.info(f'Robot state: Connecting socket {self.host}:{self.port}')
            self.socket = socket.create_connection((self.host, self.port), CONNECTION_TIMEOUT)
            self.socket.settimeout(None)
            _set_socket_opts(self.socket)

            LOGGER.info('Robot state: Socket connected')
        except:
            LOGGER.error(f'Cannot connect robot state: {self.host}:{self.port}')
            raise

    def _disconnect_socket(self):
        LOGGER.info('Robot state: Disconnecting socket')
        if self.socket:
            self.socket.close()

    def socket_worker(self):
        LOGGER.info('Robot state: Worker started')
        current_message = CurrentMessage()
        version_already_checked = False

        while self.is_running:
            try:
                if not self.socket:
                    self._connect_socket()

                current_message.add_perf_marker('select_before')
                readable, _writable, failed = select.select([self.socket], [], [self.socket], SOCKET_SELECT_TIMEOUT)
                current_message.add_perf_marker('select_after')

                if len(failed) > 0:
                    raise socket.error('No readable socket available')

                if len(readable) == 0:
                    raise socket.timeout('Socket selection timed out')

                if current_message.state == 'recv_header':
                    current_message.add_perf_marker('recv_before')
                    header_chunk = readable[0].recv(current_message.remaining_header_bytes)
                    current_message.append_header_chunk(header_chunk)
                    # NOTE: we rely on the fact that socket will still be readable after header
                    # so, instead of continuing to the next iteration, we continue to the next line and read payload

                # We have a full header, we can proceed with payload
                if current_message.state == 'recv_payload':
                    # Ensure incoming version check matches
                    if not version_already_checked:
                        server_protocol_version = current_message.protocol_version

                        if WireProtocol.VERSION != server_protocol_version:
                            raise Exception('Protocol version mismatch: Server={}, Client={}'.format(server_protocol_version, WireProtocol.VERSION))

                        version_already_checked = True

                    chunk = readable[0].recv(current_message.remaining_payload_bytes)

                    try:
                        if not chunk:
                            LOGGER.debug('Nothing read in chuck recv, will continue')
                            continue

                        current_message.append_payload_chunk(chunk)

                        if current_message.state == 'message_complete':
                            message = current_message.deserialize()

                            # Emit global and individual events
                            self.emit('message', message)
                            self.emit(WireProtocol.get_response_key(message), message)

                            if LOGGER.get_effective_level() >= logging.LoggingSeverity.DEBUG:
                                timing_sent_to_topic = _get_perf_counter()
                                ts = TIMING_START[message.feedback_id]
                                LOGGER.debug('F-ID={}, S-ID={}, {}, sent_to_topic={}, msg_len={}'.format(
                                            message.feedback_id,
                                            message.sequence_id,
                                            ', '.join(['{}={}'.format(k, v - ts) for k, v in current_message.timing]),
                                            timing_sent_to_topic - ts,
                                            len(current_message.header) + len(current_message.payload)))

                            current_message.clear()

                    except Exception as me:
                        LOGGER.error(f"Exception while recv/deserialization of a message, skipping message. Exception={me}")
                        LOGGER.error(str(current_message.payload))
                        current_message.clear()

            except socket.timeout as ste:
                # The socket has a timeout, so that it does not block on recv()
                # If it times out, it's ok, we just continue and re-start receiving
                pass
            except socket.error as se:
                error_message = 'Socket error on robot state interface: {}'.format(str(se))
                LOGGER.error(error_message)

                if self.is_running:
                    self.socket = None
                    self.emit('socket_broken')

                    LOGGER.warn(f'Robot state: Disconnection detected, waiting {RECONNECT_DELAY} sec before reconnect...')
                    time.sleep(RECONNECT_DELAY)
            except Exception as e:
                error_message = 'Exception on robot state interface: {}'.format(str(e))
                LOGGER.error(error_message)
                raise SystemExit

        LOGGER.info('Robot state: Worker stopped')


class StreamingInterfaceConnection(EventEmitterMixin):
    def __init__(self, host, port):
        super(StreamingInterfaceConnection, self).__init__()
        self.is_running = False

        self.host = host
        self.port = port

        self.queue = queue.Queue()
        self.thread = None
        self.socket = None

    def on_message_sent(self, callback):
        """Add an event handler to be triggered on message sent."""
        self.on('message_sent', callback)

    def on_socket_broken(self, callback):
        """Add an event handler to be triggered when the socket is broken."""
        self.on('socket_broken', callback)

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
            LOGGER.info(f'Streaming interface: Connecting socket {self.host}:{self.port}')
            self.socket = socket.create_connection((self.host, self.port), CONNECTION_TIMEOUT)
            _set_socket_opts(self.socket)

            LOGGER.info('Streaming interface: Socket connected')
        except:
            LOGGER.error(f'Cannot connect streaming interface: {self.host}:{self.port}')
            raise

    def _disconnect_socket(self):
        LOGGER.info('Streaming interface: Disconnecting socket')
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
        LOGGER.info('Streaming interface: Worker started')
        last_successful_connect = None

        while self.is_running:
            try:
                if not self.socket:
                    self._connect_socket()
                    last_successful_connect = time.time()

                # TODO: Check if we can lower the timeout to make sure we respond faster to failed socket
                token_type, message = self.queue.get(block=True, timeout=QUEUE_TIMEOUT)

                if token_type == QUEUE_MESSAGE_TOKEN:
                    if LOGGER.get_effective_level() >= logging.LoggingSeverity.DEBUG:
                        timing_incoming = _get_perf_counter()
                        TIMING_START[message.sequence_id] = timing_incoming

                    wire_message = WireProtocol.serialize(message)
                    _, writable, _ = select.select([], [self.socket], [])

                    if len(writable) == 0:
                        raise Exception('No writable socket available')

                    sent_bytes = writable[0].send(wire_message)

                    if LOGGER.get_effective_level() >= logging.LoggingSeverity.DEBUG:
                        timing_sent = _get_perf_counter()
                        LOGGER.debug('S-ID={}, , sent_to_robot={}, incoming={}, msg_len={}'.format(message.sequence_id, timing_sent - timing_incoming, timing_incoming, len(wire_message)))

                    if sent_bytes == 0:
                        raise socket.error('Streaming socket connection broken')

                    self.emit('message_sent', message, wire_message)
                elif token_type == QUEUE_TERMINATION_TOKEN:
                    LOGGER.info('Signal to terminate, closing socket')
                    # TODO: RAPID side does not yet support graceful shutdown
                    # SOCKET_CLOSE_COMMAND = 'stop\r\n'
                    # self.socket.send(SOCKET_CLOSE_COMMAND)
                    break
                elif token_type == QUEUE_RECONNECTION_TOKEN:
                    reconnection_timestamp = message
                    if reconnection_timestamp > last_successful_connect:
                        raise socket.error('Reconnection requested at {}'.format(message))
                    else:
                        LOGGER.info(f'Ignoring stale reconnection request issued at {reconnection_timestamp} because last successful connection was at {last_successful_connect}')
                else:
                    raise Exception('Unknown token type')
            except queue.Empty:
                pass
            except socket.error:
                if self.is_running:
                    self.socket = None
                    self.emit('socket_broken')

                    LOGGER.warn(f'Streaming interface: Disconnection detected, waiting {RECONNECT_DELAY} sec before reconnect...')
                    time.sleep(RECONNECT_DELAY)
            except Exception as e:
                error_message = 'Exception on streaming interface worker: {}'.format(str(e))
                LOGGER.error(error_message)
                raise SystemExit(error_message)
        LOGGER.info('Streaming interface: Worker stopped')


def main():
    DEBUG = True
    ROBOT_HOST_DEFAULT = '127.0.0.1'
    ROBOT_STREAMING_PORT_DEFAULT = 30101
    ROBOT_STATE_PORT_DEFAULT = 30201
    TOPIC_MODE = 'message'

    rclpy.init()
    t = threading.Thread(target=spin_in_background)
    t.start()

    node = rclpy.create_node('compas_rrc_driver')
    rclpy.get_global_executor().add_node(node)

    logger = node.get_logger()
    logger.set_level(logging.LoggingSeverity.DEBUG if DEBUG else logging.LoggingSeverity.INFO)
    LOGGER.set_level(logging.LoggingSeverity.DEBUG if DEBUG else logging.LoggingSeverity.INFO)

    robot_host = node.declare_parameter('robot_ip_address', value=ROBOT_HOST_DEFAULT, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Robot hostname.')).value
    robot_streaming_port = node.declare_parameter('robot_streaming_port', value=ROBOT_STREAMING_PORT_DEFAULT,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description="Robot straming port")).value
    robot_state_port = node.declare_parameter('robot_state_port', value=ROBOT_STATE_PORT_DEFAULT, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description="Robot state port")).value
    sequence_check_mode = node.declare_parameter('sequence_check_mode', value=SequenceCheckModes.ALL, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Sequence check mode, from the SequenceCheckModes enum.")).value

    # Set protocol version in a parameter to enable version checks from the client side
    node.declare_parameter('protocol_version', value=WireProtocol.VERSION,descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description="Protocol version"))

    streaming_interface = None
    robot_state = None
    topic_provider = None

    logger.info(f'Connecting robot {robot_host} (ports {robot_streaming_port} & {robot_state_port}, sequence check mode={sequence_check_mode})')
    streaming_interface = StreamingInterfaceConnection(robot_host, robot_streaming_port)
    streaming_interface.connect()

    robot_state = RobotStateConnection(robot_host, robot_state_port)
    robot_state.connect()

    # If a disconnect is detected on the robot state socket, it will try to reconnect
    # So we notify the streaming interface to do the same
    robot_state.on_socket_broken(streaming_interface.reconnect)

    def message_received_log(message):
        logger.debug(f'Received: "{message.feedback}", content: {str(message).replace("\n", "; ")}')
        logger.info(f'Received message: feedback={message.feedback}, sequence_id={message.sequence_id}, feedback_id={message.feedback_id}')

    def message_sent_log(message, wire_message):
        logger.debug(f'Sent: "{message.instruction}", content: {str(message).replace("\n", "; ")}')
        logger.info(f'Sent message with length={len(wire_message)}, instruction={message.instruction}, sequence id={message.sequence_id}')

    streaming_interface.on_message_sent(message_sent_log)
    if DEBUG:
        robot_state.on_message(message_received_log)

    if TOPIC_MODE == 'message':
        options = dict(sequence_check_mode=sequence_check_mode)
        topic_provider = RobotMessageTopicProvider('robot_command', 'robot_response', streaming_interface, robot_state, node, options=options)

    try:
        rclpy.spin(node)
    except SystemExit:
        logger.info('Quitting...')
        if topic_provider:
            logger.debug('Disconnecting topic provider...')
            topic_provider.disconnect()

        if streaming_interface:
            logger.info('Disconnecting streaming interface...')
            streaming_interface.disconnect()

        if robot_state:
            logger.info('Disconnecting robot state...')
            robot_state.disconnect()

    t.join()
    logger.info('Terminated')


if __name__ == '__main__':
    main()
