#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry



class TurnRobotAction(object):

    def __init__():
        rospy.logdebug("Not handled")



def main():
    rospy.init_node('rotate_robot_server')

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()