import requests
import rospy
import sys

def main():
    DEBUG = True

    log_level = rospy.DEBUG if DEBUG else rospy.INFO
    rospy.init_node('compas_rrc_webservice_client', log_level=log_level)

    webservice_username, webservice_password = sys.argv[1], sys.argv[2]
    robot_host = rospy.get_param('robot_ip_address')

    if not robot_host:
        rospy.logerror('No robot IP address defined')
        rospy.signal_shutdown('Cannot connect to webservices, no robot IP defined')
        return

    rospy.loginfo('Starting RRC webservice client: [{}] as {}:{}'.format(robot_host, webservice_username, '*' * len(webservice_password)))

    rospy.spin()

    rospy.loginfo('Terminated')

if __name__ == '__main__':
    main()
