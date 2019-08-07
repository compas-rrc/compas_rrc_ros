import rospy
from threading import Lock
from abb_042_driver import msg


class RobotMessageTopicProvider(object):
    def __init__(self, topic_name_sub, topic_name_pub, streaming_interface, robot_state):
        super(RobotMessageTopicProvider, self).__init__()

        self._publish_lock = Lock()
        self._receive_lock = Lock()
        self._last_published_id = 0
        self._last_received_id = 0

        self.streaming_interface = streaming_interface
        self.robot_state = robot_state

        self.subscriber = rospy.Subscriber(topic_name_sub, msg.RobotMessage, self.ros_to_robot_handler)
        self.publisher = rospy.Publisher(topic_name_pub, msg.RobotMessage)

        self.robot_state.on_message(self.robot_to_ros_handler)

        rospy.loginfo('Topic provider started. Subscribed to %s, publishing to %s', topic_name_sub, topic_name_pub)

    def ros_to_robot_handler(self, ros_message):
        """Handle messages from ROS topics to the robot controller"""
        try:
            with self._publish_lock:
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
            rospy.loginfo('Topic provider disconnected')
        except Exception as e:
            rospy.logerr(e)
