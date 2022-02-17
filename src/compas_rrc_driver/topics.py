from threading import Lock

import rospy

from compas_rrc_driver import msg


class SequenceCheckModes(object):
    NONE = 'none'
    ALL = 'all'
    INCOMING = 'incoming'
    OUTGOING = 'outgoing'


class SequenceCounter(object):
    """An atomic, thread-safe sequence increament counter."""
    ROLLOVER_THRESHOLD = 1000000

    def __init__(self, start=0):
        """Initialize a new counter to given initial value."""
        self._lock = Lock()
        self._value = start

    def increment(self, num=1):
        """Atomically increment the counter by ``num`` and
        return the new value.
        """
        with self._lock:
            self._value += num
            if self._value > SequenceCounter.ROLLOVER_THRESHOLD:
                self._value = 1
            return self._value

    @property
    def value(self):
        """Current sequence counter."""
        with self._lock:
            return self._value


class RobotMessageTopicAdapter(object):
    def __init__(self, topic_name_sub, topic_name_pub, streaming_interface, robot_state, options=None):
        super(RobotMessageTopicAdapter, self).__init__()

        self._publish_lock = Lock()
        self._receive_lock = Lock()
        self._last_published_id = 0
        self._last_received_id = 0
        self.sequence_check_mode = SequenceCheckModes.NONE

        if options:
            self.sequence_check_mode = options.get('sequence_check_mode', SequenceCheckModes.NONE)

            if self.sequence_check_mode not in (SequenceCheckModes.NONE, SequenceCheckModes.ALL, SequenceCheckModes.INCOMING, SequenceCheckModes.OUTGOING):
                raise Exception('Unsupported sequence check mode. Value={}'.format(self.sequence_check_mode))

        self.streaming_interface = streaming_interface
        self.robot_state = robot_state

        # TODO: Verify topic queue sizes
        self.subscriber = rospy.Subscriber(topic_name_sub, msg.RobotMessage, self.ros_to_robot_handler)
        self.publisher = rospy.Publisher(topic_name_pub, msg.RobotMessage, queue_size=1)

        self.robot_state.on_message(self.robot_to_ros_handler)
        self.robot_state.on_socket_broken(self._reset_sequence_id)

        rospy.loginfo('Topic adapter started. Subscribed to %s, publishing to %s', topic_name_sub, topic_name_pub)

    def _reset_sequence_id(self):
        with self._publish_lock:
            self._last_published_id = 0

        with self._receive_lock:
            self._last_received_id = 0

    def ros_to_robot_handler(self, ros_message):
        """Handle messages from ROS topics to the robot controller"""
        try:
            with self._publish_lock:
                if self.sequence_check_mode in (SequenceCheckModes.OUTGOING, SequenceCheckModes.ALL):
                    if ros_message.sequence_id != self._last_published_id + 1:
                        raise Exception('Received out of order (ROS -> Controller). Received={}, Last sequence id={}'.format(ros_message.sequence_id, self._last_published_id))
                self.streaming_interface.execute_instruction(ros_message)
        except Exception as e:
            rospy.logerr(e)
            raise e
        finally:
            self._last_published_id = ros_message.sequence_id

    def robot_to_ros_handler(self, ros_message):
        """Handle messages from the robot controller to ROS topic."""
        with self._receive_lock:
            try:
                if self.sequence_check_mode in (SequenceCheckModes.INCOMING, SequenceCheckModes.ALL):
                    if ros_message.sequence_id != self._last_received_id + 1:
                        raise Exception('Received out of order (Controller -> ROS). Received={}, Last sequence id={}'.format(ros_message.sequence_id, self._last_received_id))
                self.publisher.publish(ros_message)
            except Exception as e:
                rospy.logerr(e)
                raise e
            finally:
                self._last_received_id = ros_message.sequence_id

    def disconnect(self):
        try:
            self.subscriber.unregister()
            self.publisher.unregister()
            rospy.loginfo('Topic adapter disconnected')
        except Exception as e:
            rospy.logerr(e)


class SystemMessageTopicAdapter(object):
    def __init__(self, topic_name_sub, topic_name_pub, system_interface):
        super(SystemMessageTopicAdapter, self).__init__()
        self.counter = SequenceCounter()
        self.system_interface = system_interface
        # TODO: Verify topic queue sizes
        self.subscriber = rospy.Subscriber(topic_name_sub, msg.RobotMessage, self.ros_to_robot_handler)
        self.publisher = rospy.Publisher(topic_name_pub, msg.RobotMessage, queue_size=1)

        rospy.loginfo('System Topic adapter started. Subscribed to %s, publishing to %s', topic_name_sub, topic_name_pub)

    def ros_to_robot_handler(self, ros_message):
        """Handle messages from ROS topics to the robot controller"""
        try:
            response = self.system_interface.execute_instruction(ros_message)
            if response is not None:
                response.sequence_id = self.counter.increment()
                self.handle_response(response)
        except Exception as e:
            rospy.logerr(e)
            raise e

    def handle_response(self, response):
        """Handle messages from the robot controller to ROS topic."""
        try:
            message = response.to_ros_message(msg.RobotMessage)
            self.publisher.publish(message)
        except Exception as e:
            rospy.logerr(e)
            raise e

    def disconnect(self):
        try:
            self.subscriber.unregister()
            self.publisher.unregister()
            rospy.loginfo('Topic adapter disconnected')
        except Exception as e:
            rospy.logerr(e)
