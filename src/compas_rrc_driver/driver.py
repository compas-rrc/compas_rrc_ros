#!/usr/bin/env python
import logging
import os
import select
import socket
import threading
import time
import timeit

import rospy

from compas_rrc_driver.event_emitter import EventEmitterMixin
from compas_rrc_driver.protocol import WireProtocol
from compas_rrc_driver.topics import RobotMessageTopicAdapter
from compas_rrc_driver.topics import SystemMessageTopicAdapter
from compas_rrc_driver.abb import (
    build_system_message_interface as abb_system_message_interface_factory,
)

try:
    import Queue as queue
except ImportError:
    import queue

CONNECTION_TIMEOUT = 5  # In seconds
QUEUE_TIMEOUT = 5  # In seconds
RECONNECT_DELAY = 1  # In seconds
SOCKET_SELECT_TIMEOUT = 10  # In seconds
QUEUE_MESSAGE_TOKEN = 0
QUEUE_TERMINATION_TOKEN = -1
QUEUE_RECONNECTION_TOKEN = -2
START_PROCESS_TIME = timeit.default_timer()
TIMING_START = dict()
SYSTEM_MESSAGE_INTERFACE_FACTORIES = dict(ABB=abb_system_message_interface_factory)

LOGGER = logging.getLogger("compas_rrc_driver")


def _set_socket_opts(sock):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    # The following options make sure keep alive is active, otherwise the socket closes after about 5 or 6 minutes
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 6)


def _get_perf_counter():
    secs = timeit.default_timer() - START_PROCESS_TIME
    return int(secs * 1000)


def _get_logs_dir():
    sourced_catkin_ws = os.environ.get("CMAKE_PREFIX_PATH", "").split(os.pathsep)[0]

    if sourced_catkin_ws:
        logs_dir = os.path.join(sourced_catkin_ws, "..", "logs")
    else:
        logs_dir = os.path.dirname(__file__)

    return logs_dir


class CurrentMessage(object):
    def __init__(self):
        self.clear()

    @property
    def state(self):
        if len(self.header) < WireProtocol.FIXED_HEADER_LEN:
            return "recv_header"

        if len(self.header) == WireProtocol.FIXED_HEADER_LEN:
            if self.remaining_payload_bytes > 0:
                return "recv_payload"
            elif self.remaining_payload_bytes == 0:
                return "message_complete"
            else:
                raise Exception(
                    "Payload exceeds expected length. Header={}, Payload={}".format(
                        self.header, self.payload
                    )
                )
        else:
            raise Exception(
                "Header exceeds expected length. Header={}, Payload={}".format(
                    self.header, self.payload
                )
            )

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
            raise socket.error("Socket broken, header chunk empty")

        self.header += chunk
        self.add_perf_marker("header_recv")

    def append_payload_chunk(self, chunk):
        self.payload += chunk
        self.add_perf_marker("payload_recv")

    def clear(self):
        self.header = b""
        self.payload = b""
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
        self.on("message", callback)

    def on_socket_broken(self, callback):
        """Add an event handler to be triggered when the socket is broken."""
        self.on("socket_broken", callback)

    def connect(self):
        self.is_running = True

        self.thread = threading.Thread(
            target=self.socket_worker, name="robot_state_socket"
        )
        self.thread.daemon = True
        self.thread.start()

    def disconnect(self):
        self.is_running = False
        self.thread.join(CONNECTION_TIMEOUT)
        self._disconnect_socket()

    def _connect_socket(self):
        try:
            rospy.loginfo("Robot state: Connecting socket %s:%d", self.host, self.port)
            self.socket = socket.create_connection(
                (self.host, self.port), CONNECTION_TIMEOUT
            )
            self.socket.settimeout(None)
            _set_socket_opts(self.socket)

            rospy.loginfo("Robot state: Socket connected")
        except:
            rospy.logerr("Cannot connect robot state: %s:%d", self.host, self.port)
            raise

    def _disconnect_socket(self):
        rospy.loginfo("Robot state: Disconnecting socket")
        if self.socket:
            self.socket.close()

    def socket_worker(self):
        rospy.loginfo("Robot state: Worker started")
        current_message = CurrentMessage()
        version_already_checked = False

        while self.is_running:
            try:
                if not self.socket:
                    self._connect_socket()

                current_message.add_perf_marker("select_before")
                readable, _writable, failed = select.select(
                    [self.socket], [], [self.socket], SOCKET_SELECT_TIMEOUT
                )
                current_message.add_perf_marker("select_after")

                if len(failed) > 0:
                    raise socket.error("No readable socket available")

                if len(readable) == 0:
                    raise socket.timeout("Socket selection timed out")

                if current_message.state == "recv_header":
                    current_message.add_perf_marker("recv_before")
                    header_chunk = readable[0].recv(
                        current_message.remaining_header_bytes
                    )
                    current_message.append_header_chunk(header_chunk)
                    # NOTE: we rely on the fact that socket will still be readable after header
                    # so, instead of continuing to the next iteration, we continue to the next line and read payload

                # We have a full header, we can proceed with payload
                if current_message.state == "recv_payload":
                    # Ensure incoming version check matches
                    if not version_already_checked:
                        server_protocol_version = current_message.protocol_version

                        if WireProtocol.VERSION != server_protocol_version:
                            raise Exception(
                                "Protocol version mismatch: Server={}, Client={}".format(
                                    server_protocol_version, WireProtocol.VERSION
                                )
                            )

                        version_already_checked = True

                    chunk = readable[0].recv(current_message.remaining_payload_bytes)

                    try:
                        if not chunk:
                            rospy.logdebug("Nothing read in chuck recv, will continue")
                            continue

                        current_message.append_payload_chunk(chunk)

                        if current_message.state == "message_complete":
                            message = current_message.deserialize()

                            # Emit global and individual events
                            self.emit("message", message)
                            self.emit(WireProtocol.get_response_key(message), message)

                            if LOGGER.getEffectiveLevel() >= logging.DEBUG:
                                timing_sent_to_topic = _get_perf_counter()
                                ts = TIMING_START[message.feedback_id]
                                LOGGER.debug(
                                    "F-ID={}, S-ID={}, {}, sent_to_topic={}, msg_len={}".format(
                                        message.feedback_id,
                                        message.sequence_id,
                                        ", ".join(
                                            [
                                                "{}={}".format(k, v - ts)
                                                for k, v in current_message.timing
                                            ]
                                        ),
                                        timing_sent_to_topic - ts,
                                        len(current_message.header)
                                        + len(current_message.payload),
                                    )
                                )

                            current_message.clear()

                    except Exception as me:
                        rospy.logerr(
                            "Exception while recv/deserialization of a message, skipping message. Exception=%s",
                            str(me),
                        )
                        rospy.logerr(str(current_message.payload))
                        current_message.clear()

            except socket.timeout as ste:
                # The socket has a timeout, so that it does not block on recv()
                # If it times out, it's ok, we just continue and re-start receiving
                pass
            except socket.error as se:
                error_message = "Socket error on robot state interface: {}".format(
                    str(se)
                )
                rospy.logerr(error_message)

                if self.is_running:
                    self.socket = None
                    self.emit("socket_broken")

                    rospy.logwarn(
                        "Robot state: Disconnection detected, waiting %d sec before reconnect...",
                        RECONNECT_DELAY,
                    )
                    time.sleep(RECONNECT_DELAY)
            except Exception as e:
                error_message = "Exception on robot state interface: {}, waiting {} sec before reconnect...".format(
                    str(e), RECONNECT_DELAY
                )
                rospy.logerr(error_message)
                self.socket = None
                time.sleep(RECONNECT_DELAY)

        rospy.loginfo("Robot state: Worker stopped")


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
        self.on("message_sent", callback)

    def on_socket_broken(self, callback):
        """Add an event handler to be triggered when the socket is broken."""
        self.on("socket_broken", callback)

    def connect(self):
        self.is_running = True

        self.thread = threading.Thread(
            target=self.socket_worker, name="streaming_interface_socket"
        )
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
            rospy.loginfo(
                "Streaming interface: Connecting socket {}:{}".format(
                    self.host, self.port
                )
            )
            self.socket = socket.create_connection(
                (self.host, self.port), CONNECTION_TIMEOUT
            )
            _set_socket_opts(self.socket)

            rospy.loginfo("Streaming interface: Socket connected")
        except:
            rospy.logerr(
                "Cannot connect streaming interface: {}:{}".format(self.host, self.port)
            )
            raise

    def _disconnect_socket(self):
        rospy.loginfo("Streaming interface: Disconnecting socket")
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
        rospy.loginfo("Streaming interface: Worker started")
        last_successful_connect = None

        while self.is_running:
            try:
                if not self.socket:
                    self._connect_socket()
                    last_successful_connect = time.time()

                # TODO: Check if we can lower the timeout to make sure we respond faster to failed socket
                token_type, message = self.queue.get(block=True, timeout=QUEUE_TIMEOUT)

                if token_type == QUEUE_MESSAGE_TOKEN:
                    if LOGGER.getEffectiveLevel() >= logging.DEBUG:
                        timing_incoming = _get_perf_counter()
                        TIMING_START[message.sequence_id] = timing_incoming

                    wire_message = WireProtocol.serialize(message)
                    _, writable, _ = select.select([], [self.socket], [])

                    if len(writable) == 0:
                        raise Exception("No writable socket available")

                    sent_bytes = writable[0].send(wire_message)

                    if LOGGER.getEffectiveLevel() >= logging.DEBUG:
                        timing_sent = _get_perf_counter()
                        LOGGER.debug(
                            "S-ID={}, , sent_to_robot={}, incoming={}, msg_len={}".format(
                                message.sequence_id,
                                timing_sent - timing_incoming,
                                timing_incoming,
                                len(wire_message),
                            )
                        )

                    if sent_bytes == 0:
                        raise socket.error("Streaming socket connection broken")

                    self.emit("message_sent", message, wire_message)
                elif token_type == QUEUE_TERMINATION_TOKEN:
                    rospy.loginfo("Signal to terminate, closing socket")
                    # TODO: RAPID side does not yet support graceful shutdown
                    # SOCKET_CLOSE_COMMAND = 'stop\r\n'
                    # self.socket.send(SOCKET_CLOSE_COMMAND)
                    break
                elif token_type == QUEUE_RECONNECTION_TOKEN:
                    reconnection_timestamp = message
                    if reconnection_timestamp > last_successful_connect:
                        raise socket.error(
                            "Reconnection requested at {}".format(message)
                        )
                    else:
                        rospy.loginfo(
                            "Ignoring stale reconnection request issued at {} because last successful connection was at {}".format(
                                reconnection_timestamp, last_successful_connect
                            )
                        )
                else:
                    raise Exception("Unknown token type")
            except queue.Empty:
                pass
            except socket.error:
                if self.is_running:
                    self.socket = None
                    self.emit("socket_broken")

                    rospy.logwarn(
                        "Streaming interface: Disconnection detected, waiting %d sec before reconnect...",
                        RECONNECT_DELAY,
                    )
                    time.sleep(RECONNECT_DELAY)
            except Exception as e:
                error_message = "Exception on streaming interface worker: {}, waiting {} sec before reconnect...".format(
                    str(e), RECONNECT_DELAY
                )
                rospy.logerr(error_message)
                self.socket = None
                time.sleep(RECONNECT_DELAY)

        rospy.loginfo("Streaming interface: Worker stopped")


def message_received_log(message):
    rospy.logdebug(
        'Received: "%s", content: %s',
        message.feedback,
        str(message).replace("\n", "; "),
    )
    rospy.loginfo(
        "Received message: feedback=%s, sequence_id=%d, feedback_id=%d",
        message.feedback,
        message.sequence_id,
        message.feedback_id,
    )


def message_sent_log(message, wire_message):
    rospy.logdebug(
        'Sent: "%s", content: %s', message.instruction, str(message).replace("\n", "; ")
    )
    rospy.loginfo(
        "Sent message with length=%d, instruction=%s, sequence id=%d",
        len(wire_message),
        message.instruction,
        message.sequence_id,
    )


def system_message_sent_log(request_method, url):
    rospy.logdebug(
        "Sent WebService request. Method={}, URL={}".format(request_method, url)
    )


def system_message_received_log(response):
    rospy.logdebug(
        "Received WebService response. Status Code={}, URL={}".format(
            response.status_code, response.url
        )
    )


def connect_sys_interface(robot_host, debug):
    disable_sys_interface = rospy.get_param("disable_sys_interface", False)
    if disable_sys_interface:
        return []

    ROBOT_TYPE_DEFAULT = "ABB"
    robot_type = rospy.get_param("robot_type", ROBOT_TYPE_DEFAULT)

    rospy.loginfo("Connecting system message interface %s", robot_host)
    # TODO: change this to plugins
    if robot_type not in SYSTEM_MESSAGE_INTERFACE_FACTORIES:
        raise Exception(
            "Robot type {} has no supported system message interface".format(robot_type)
        )

    prefix = rospy.get_namespace().replace("/", "").upper()
    robot_user_key = prefix + "_ROBOT_USER"
    robot_pass_key = prefix + "_ROBOT_PASS"

    if robot_user_key not in os.environ or robot_pass_key not in os.environ:
        message = (
            "User and/or password for the system message interface are not defined in the environment variables. "
            + "Please configure '{}' and '{}' with the appropriate credentials. Defaults will be used.".format(
                robot_user_key, robot_pass_key
            )
        )
        rospy.logwarn(message)

    mechunit = rospy.get_param("mechunit")
    build_system_message_interface = SYSTEM_MESSAGE_INTERFACE_FACTORIES[robot_type]

    robot_user = os.environ.get(robot_user_key)
    robot_pass = os.environ.get(robot_pass_key)

    system_interface = build_system_message_interface(
        robot_host, robot_user, robot_pass, options=dict(mechunit=mechunit)
    )
    system_topic_adapter = SystemMessageTopicAdapter(
        "robot_command_system", "robot_response_system", system_interface
    )

    if debug:
        rospy.loginfo("Attaching debug log handlers")
        system_interface.on_request(system_message_sent_log)
        system_interface.on_response(system_message_received_log)

    return [system_topic_adapter]


def connect_app_interface(robot_host, debug):
    disable_app_interface = rospy.get_param("disable_app_interface", False)
    if disable_app_interface:
        return []

    robot_streaming_port = rospy.get_param("robot_streaming_port")
    robot_state_port = rospy.get_param("robot_state_port")
    sequence_check_mode = rospy.get_param("sequence_check_mode")

    rospy.loginfo(
        "Connecting robot %s (ports %d & %d, sequence check mode=%s)",
        robot_host,
        robot_streaming_port,
        robot_state_port,
        sequence_check_mode,
    )
    streaming_interface = StreamingInterfaceConnection(robot_host, robot_streaming_port)
    streaming_interface.connect()

    robot_state = RobotStateConnection(robot_host, robot_state_port)
    robot_state.connect()

    # If a disconnect is detected on the robot state socket, it will try to reconnect
    # So we notify the streaming interface to do the same
    robot_state.on_socket_broken(streaming_interface.reconnect)

    streaming_interface.on_message_sent(message_sent_log)
    if debug:
        robot_state.on_message(message_received_log)

    options = dict(sequence_check_mode=sequence_check_mode)
    topic_adapter = RobotMessageTopicAdapter(
        "robot_command",
        "robot_response",
        streaming_interface,
        robot_state,
        options=options,
    )

    return [robot_state, streaming_interface, topic_adapter]


def main():
    DEBUG = True
    ROBOT_HOST_DEFAULT = "127.0.0.1"

    LOGGER.setLevel(logging.DEBUG if DEBUG else logging.INFO)

    # if DEBUG:
    #     fh = logging.FileHandler(os.path.join(_get_logs_dir(), 'message-trace.log'))
    #     ff = logging.Formatter('%(asctime)s %(levelname)s %(message)s', datefmt='%H:%M:%S')
    #     fh.setFormatter(ff)
    #     LOGGER.addHandler(fh)

    log_level = rospy.DEBUG if DEBUG else rospy.INFO
    rospy.init_node("compas_rrc_driver", log_level=log_level)

    # Set protocol version in a parameter to enable version checks from the client side
    rospy.set_param("protocol_version", WireProtocol.VERSION)

    # Get general parameters
    robot_host = rospy.get_param("robot_ip_address", ROBOT_HOST_DEFAULT)

    connected_interfaces = []

    try:
        connected_interfaces.extend(connect_sys_interface(robot_host, DEBUG))
        connected_interfaces.extend(connect_app_interface(robot_host, DEBUG))

        rospy.spin()
    finally:
        rospy.loginfo("Disconnecting interfaces and adapters...")
        for interface in connected_interfaces:
            interface.disconnect()

    rospy.loginfo("Terminated")


if __name__ == "__main__":
    main()
