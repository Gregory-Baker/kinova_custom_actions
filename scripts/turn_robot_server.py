#!/usr/bin/env python
from scipy import angle
import rospy
import actionlib
from kinova_custom_actions.msg import TurnAngleFeedback, TurnAngleResult, TurnAngleAction
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Vector3
import math



class TurnRobotAction(object):
    _feedback = TurnAngleFeedback()
    _result = TurnAngleResult()

    _turn_speed = 0.3           # rad/sec

    _yaw = 0.0


    def __init__(self, name, cmd_vel_topic="/cmd_vel", odom_topic="/odom"):
        self._action_name = name
        self._cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber (odom_topic, Odometry, self.get_rotation)
        self._as = actionlib.SimpleActionServer(self._action_name, TurnAngleAction, execute_cb=self.execute_action_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Turn robot action server started")

    def execute_action_cb(self, goal):
        success = True
        rate = rospy.Rate(50)

        angle_turned = 0 
        angular_z_cmd = Vector3(z=math.copysign(self._turn_speed, goal.turn_angle))
        twist_command = Twist(angular = angular_z_cmd)
        yaw_previous=self._yaw

        rospy.loginfo("Requested turn angle (rad): %f" % goal.turn_angle)

        while(abs(angle_turned) < abs(goal.turn_angle)):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            angle_change = self._yaw - yaw_previous
            if (angle_change > math.pi): 
                angle_change -= 2* math.pi
            if (angle_change < -math.pi):
                angle_change += 2* math.pi

            angle_turned += angle_change
            yaw_previous = self._yaw


            self._cmd_pub.publish(twist_command)
            self._feedback.angle_turned = angle_turned
            self._as.publish_feedback(self._feedback)

        rospy.loginfo("Angle turned: %f" % angle_turned)

        self._cmd_pub.publish(Twist())  # Stop robot

        if (success):
            rospy.loginfo('%s: Succeeded' % self._action_name)

            self._result.angle_turned = self._feedback.angle_turned
            self._as.set_succeeded(result=self._result)



    def get_rotation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self._yaw) = euler_from_quaternion (orientation_list)
        


    


def main():
    rospy.init_node('rotate_robot_server')

    server = TurnRobotAction(rospy.get_name(), odom_topic="/odometry/filtered")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()