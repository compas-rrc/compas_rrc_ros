#!/usr/bin/env python
import socket
import threading
import time

import rospy
from abb_042_driver.event_emitter import EventEmitterMixin
from abb_042_driver.protocol import WireProtocol
from abb_042_driver.services import AbbStringServiceProvider
from abb_042_driver.topics import AbbMessageTopicProvider

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
        rospy.loginfo('Robot state: Connecting socket %s:%d', self.host, self.port)
        self.socket = socket.create_connection((self.host, self.port), CONNECTION_TIMEOUT)
        rospy.loginfo('Robot state: Socket connected')

    def _disconnect_socket(self):
        rospy.loginfo('Robot state: Disconnecting socket')
        if self.socket:
            self.socket.close()

    def socket_worker(self):
        rospy.loginfo('Robot state: Worker started')
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

                        chunk = self.socket.recv(1024)
                        if not chunk:
                            rospy.logdebug('Nothing read in chuck recv, will continue')
                            continue

                        current_payload += chunk
                        if len(current_payload) == message_length - WireProtocol.FIXED_HEADER_LEN:
                            message = WireProtocol.deserialize(current_header, current_payload)
                            # Emit global and individual events
                            self.emit('message', message)
                            self.emit(message.response_key, message)
                            current_payload = b''

                    except Exception as me:
                        rospy.logerr('Exception while recv/deserialization of a message, skipping message. Exception=%s', str(me))
                        current_payload = b''
                    finally:
                        # Ready for next
                        current_header = b''

            except socket.timeout:
                # The socket has a timeout, so that it does not block on recv()
                # If it times out, it's ok, we just continue and re-start receiving
                pass
            except socket.error:  # Python 3 would probably be ConnectionResetError
                rospy.loginfo('Disconnection detected, waiting %d sec before reconnect...', RECONNECT_DELAY)
                time.sleep(RECONNECT_DELAY)
                self._connect_socket()
            except Exception as e:
                error_message = 'Exception on robot state interface: {}'.format(str(e))
                rospy.logerr(error_message)
                rospy.signal_shutdown(error_message)
                break

        rospy.loginfo('Robot state: Worker stopped')


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
        rospy.loginfo('Streaming interface: Connecting socket %s:%d', self.host, self.port)
        self.socket = socket.create_connection((self.host, self.port), CONNECTION_TIMEOUT)
        rospy.loginfo('Streaming interface: Socket connected')

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

        self.queue.put(message)

    def socket_worker(self):
        rospy.loginfo('Streaming interface: Worker started')
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
                    rospy.logdebug('Executing: "%s"\n        with content: %s', message.instruction, str(message.to_data()))
                    wire_message = WireProtocol.serialize(message)
                    sent_bytes = self.socket.send(wire_message)
                    if sent_bytes == 0:
                        raise Exception('Socket connection broken')

                    # rospy.loginfo('Sent message with length=%d', len(wire_message))
            except queue.Empty:
                pass
            except Exception as e:
                error_message = 'Exception on streaming interface worker: {}'.format(str(e))
                rospy.logerr(error_message)
                rospy.signal_shutdown(error_message)
                break
        rospy.loginfo('Streaming interface: Worker stopped')


def main():
    DEBUG = False

    ABB_HOST_DEFAULT = '127.0.0.1'

    SERVICE_FORMAT = 'string'
    TOPIC_FORMAT = 'message'

    log_level = rospy.DEBUG if DEBUG else rospy.INFO
    rospy.init_node('abb_042_driver', log_level=log_level)

    abb_host = rospy.get_param('robot_ip_address', ABB_HOST_DEFAULT)
    abb_streaming_port = rospy.get_param('robot_streaming_port')
    abb_state_port = rospy.get_param('robot_state_port')

    streaming_interface = None
    robot_state = None

    try:
        rospy.loginfo('Connecting robot %s (ports %d & %d)', abb_host, abb_streaming_port, abb_state_port)
        streaming_interface = StreamingInterfaceConnection(abb_host, abb_streaming_port)
        streaming_interface.connect()

        robot_state = RobotStateConnection(abb_host, abb_state_port)
        robot_state.connect()

        if DEBUG:
            def message_tracing_output(message):
                rospy.logdebug('Message received: %s', str(message.to_data()))

            robot_state.on_message(message_tracing_output)

        # TODO: Add support for AbbMessageCommand service
        if SERVICE_FORMAT == 'string':
            AbbStringServiceProvider('abb_command', streaming_interface, robot_state)
        if TOPIC_FORMAT == 'message':
            AbbMessageTopicProvider('abb_command', streaming_interface)

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
