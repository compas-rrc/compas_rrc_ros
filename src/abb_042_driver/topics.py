import rospy
from abb_042_driver import msg
from abb_042_driver.message import Message


class AbbMessageTopicProvider(object):
    def __init__(self, topic_name_sub, topic_name_pub, streaming_interface, robot_state):
        super(AbbMessageTopicProvider, self).__init__()

        self.streaming_interface = streaming_interface
        self.robot_state = robot_state

        self.subscriber = rospy.Subscriber(topic_name_sub, msg.AbbMessage, self.callback)
        self.publisher = rospy.Publisher(topic_name_pub, msg.AbbMessage, queue_size=10)

        self.robot_state.on_message(self.message_received)

        rospy.logdebug('Topic provider started. Subscribed to %s, publishing to %s', topic_name_sub, topic_name_pub)

    def callback(self, ros_message):
        message = Message.from_ros_message(ros_message)
        self.streaming_interface.execute_instruction(message)

    def message_received(self, message):
        try:
            ros_message = message.to_ros_message(msg.AbbMessage)
            rospy.logdebug(str(ros_message))
            self.publisher.publish(ros_message)
        except Exception as e:
            rospy.logerr(str(e))

