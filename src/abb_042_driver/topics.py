import rospy
from abb_042_driver import msg


class RobotMessageTopicProvider(object):
    def __init__(self, topic_name_sub, topic_name_pub, streaming_interface, robot_state):
        super(RobotMessageTopicProvider, self).__init__()

        self.streaming_interface = streaming_interface
        self.robot_state = robot_state

        self.subscriber = rospy.Subscriber(topic_name_sub, msg.RobotMessage, self.callback)
        self.publisher = rospy.Publisher(topic_name_pub, msg.RobotMessage, queue_size=10)

        self.robot_state.on_message(self.message_received)

        rospy.loginfo('Topic provider started. Subscribed to %s, publishing to %s', topic_name_sub, topic_name_pub)

    def callback(self, ros_message):
        try:
            self.streaming_interface.execute_instruction(ros_message)
        except Exception as e:
            rospy.logerr(e)

    def message_received(self, ros_message):
        try:
            self.publisher.publish(ros_message)
        except Exception as e:
            rospy.logerr(e)

    def disconnect(self):
        try:
            self.subscriber.unregister()
            self.publisher.unregister()
            rospy.loginfo('Topic provider disconnected')
        except Exception as e:
            rospy.logerr(e)
