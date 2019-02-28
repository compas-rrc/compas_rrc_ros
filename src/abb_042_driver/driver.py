#!/usr/bin/env python
import json
import socket
import threading
import time

import rospy
from abb_042_driver import msg
from abb_042_driver import srv
from abb_042_driver.event_emitter import EventEmitterMixin
from abb_042_driver.message import Message
from abb_042_driver.protocol import WireProtocol

try:
    import Queue as queue
except ImportError:
    import queue

CONNECTION_TIMEOUT = 5          # In seconds
RESPONSE_TIMEOUT = 5            # In seconds
QUEUE_TIMEOUT = 5               # In seconds
RECONNECT_DELAY = 3             # In seconds
QUEUE_TERMINATION_TOKEN = None


class RobotStateConnection(EventEmitterMixin):
    def __init__(self, host, port):
        super(RobotStateConnection, self).__init__()
        self.is_running = False
        self.host = host
        self.port = port

    def on_message(self, callback):
        """Add an event handler to be triggered on message arrival."""
        self.on('message', callback)

    def connect(self):
        self._connect_socket()
        self.is_running = True

        self.thread = threading.Thread(target=self.socket_worker, name='robot_state_socket')
        self.thread.daemon = True
        self.thread.start()

    def disconnect(self):
        self.is_running = False
        self.thread.join(CONNECTION_TIMEOUT)
        self._disconnect_socket()

    def _connect_socket(self):
        rospy.loginfo('Connecting robot state socket: %s:%d', self.host, self.port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(CONNECTION_TIMEOUT)
        self.socket.connect((self.host, self.port))

    def _disconnect_socket(self):
        rospy.loginfo('Disconnecting robot state socket')
        if self.socket:
            self.socket.close()

    def socket_worker(self):
        rospy.loginfo('Starting robot state socket worker')
        current_header = b''
        current_payload = b''

        while self.is_running:
            try:
                if len(current_header) < WireProtocol.FIXED_HEADER_LEN:
                    current_header += self.socket.recv(WireProtocol.FIXED_HEADER_LEN)
                    continue

                # We have a full header, we can proceed with payload
                if len(current_header) == WireProtocol.FIXED_HEADER_LEN:
                    try:
                        message_length = WireProtocol.get_message_length(current_header)
                        # message_length, protocol_version, sec, nsec = struct.unpack(WireProtocol.BYTE_ORDER + WireProtocol.HEADER_FORMAT, current_header)
                        # rospy.logdebug('HEADER: message_length=%d, protocol_version=%d, sec=%d, nsec=%d', message_length, protocol_version, sec, nsec)

                        chunk = self.socket.recv(1024)
                        if not chunk:
                            rospy.logdebug('Nothing read in chuck recv, will continue')
                            continue

                        current_payload += chunk
                        if len(current_payload) == message_length - WireProtocol.FIXED_HEADER_LEN:
                            message = WireProtocol.deserialize(current_header, current_payload)
                            # Emit global and individual events
                            self.emit('message', message)
                            self.emit(message.key, message)
                            current_payload = b''

                    finally:
                        # Ready for next
                        current_header = b''

            except socket.timeout:
                pass
            except socket.error:  # Python 3 would probably be ConnectionResetError
                rospy.loginfo('Disconnection detected, waiting %d sec before reconnect...', RECONNECT_DELAY)
                time.sleep(RECONNECT_DELAY)
                self._connect_socket()
            except Exception:
                rospy.logerr('Robot state socket failed')
                break

        rospy.loginfo('Robot state socket worker stopping...')


class StreamingInterfaceConnection(object):
    def __init__(self, host, port):
        self.is_running = False

        self.host = host
        self.port = port

        self.queue = queue.Queue()
        self.thread = None
        self.socket = None

    def connect(self):
        self._connect_socket()
        self.is_running = True

        self.thread = threading.Thread(target=self.socket_worker, name='streaming_interface_socket')
        self.thread.daemon = True
        self.thread.start()

    def disconnect(self):
        if self.is_running:
            self.is_running = False

        if self.queue:
            self.queue.put(QUEUE_TERMINATION_TOKEN)

        if self.thread:
            self.thread.join(CONNECTION_TIMEOUT)

        self._disconnect_socket()

    def _connect_socket(self):
        rospy.logdebug('Connecting interface streaming socket: %s:%d', self.host, self.port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(CONNECTION_TIMEOUT)
        self.socket.connect((self.host, self.port))

    def _disconnect_socket(self):
        rospy.logdebug('Disconnecting interface streaming socket')
        if self.socket:
            self.socket.close()

    def execute_instruction(self, message):
        # TODO: RAPID side does not (yet) instruct the client to disconnect
        # if message.instruction == 'exit':
        #     rospy.loginfo('Received exit instruction')
        #     ... invoke service disconnection
        #     return

        self.queue.put(message)

    def socket_worker(self):
        rospy.loginfo('Starting interface streaming socket worker')
        while self.is_running:
            try:
                message = self.queue.get(True, QUEUE_TIMEOUT)

                if message == QUEUE_TERMINATION_TOKEN:
                    rospy.loginfo('Signal to terminate, closing socket')
                    # TODO: RAPID side does not yet support graceful shutdown
                    # SOCKET_CLOSE_COMMAND = 'stop\r\n'
                    # self.socket.send(SOCKET_CLOSE_COMMAND)
                    break
                else:
                    rospy.logdebug('Executing: "%s"\n        with values: %s', message.instruction, message.float_values)
                    self.socket.send(WireProtocol.serialize(message))
            except queue.Empty:
                pass
        rospy.loginfo('Interface streaming socket worker stopping...')


class AbbStringServiceProvider(object):
    def __init__(self, service_name, streaming_interface, robot_state):
        rospy.logdebug('Starting string command service...')
        self.service = rospy.Service(service_name, srv.AbbStringCommand, self.handle_service_call)
        self.streaming_interface = streaming_interface
        self.robot_state = robot_state

    def handle_service_call(self, request):
        # String command handler assumes the string is JSON encoded
        command = json.loads(request.command)

        wait_event = threading.Event()
        call_results = {}

        def abb_response_received(response_message):
            call_results['response'] = json.dumps(response_message.to_data())
            wait_event.set()

        # Command might be a single instruction or a list of them
        if 'instruction' in command:
            message = Message.from_data(command)
            self.robot_state.on(message.key, abb_response_received)
            self.streaming_interface.execute_instruction(message)
            wait_event.wait(RESPONSE_TIMEOUT)
            return srv.AbbStringCommandResponse(call_results['response'])

        # TODO: Disabled for now
        # elif 'instructions' in command:
        #     for single_command in command['instructions']:
        #         message = Message.from_data(single_command)
        #         self.streaming_interface.execute_instruction(message)
        else:
            raise ValueError('Unexpected command')


def main():
    import sys
    DEBUG = True

    # ROS_HOST = '192.168.0.221'
    # ABB_HOST = '192.168.125.21'
    ABB_HOST = '127.0.0.1'

    ROBOT_STREAMING_INTERFACE_PORT = 30003
    ROBOT_STATE_PORT = 30004
    SERVICE_FORMAT = 'string'

    log_level = rospy.DEBUG if DEBUG else rospy.INFO
    rospy.init_node('abb_042_driver', log_level=log_level)

    if len(sys.argv) > 1:
        ABB_HOST = sys.argv[1]

    try:
        # topic = roslibpy.Topic(ros, '/robot_state', 'std_msgs/String') # sensor_msgs/JointState

        rospy.loginfo('Connecting robot %s...', ABB_HOST)
        streaming_interface = StreamingInterfaceConnection(ABB_HOST, ROBOT_STREAMING_INTERFACE_PORT)
        streaming_interface.connect()

        robot_state = RobotStateConnection(ABB_HOST, ROBOT_STATE_PORT)
        robot_state.connect()

        if DEBUG:
            def message_tracing_output(message):
                rospy.logdebug('MESSAGE: sec=%d, nsec=%d, Sequence id: %s, Exec level: %s, Feedback level: %s, Instruction: %s',
                               message.sec, message.nsec, str(message.sequence_id), str(message.exec_level), str(message.feedback_level), message.instruction)
                rospy.logdebug('MESSAGE (contd) Strings: %s, Floats: %s', str(message.string_values), str(message.float_values))

            robot_state.on_message(message_tracing_output)

        if SERVICE_FORMAT == 'string':
            AbbStringServiceProvider('abb_command', streaming_interface, robot_state)
        # TODO: Add AbbMessageCommand support

        rospy.spin()
    finally:
        if streaming_interface:
            rospy.loginfo('Disconnecting streaming interface...')
            streaming_interface.disconnect()

        if robot_state:
            rospy.loginfo('Disconnecting robot state...')
            robot_state.disconnect()

    rospy.loginfo('Terminated')


if __name__ == '__main__':
    main()
