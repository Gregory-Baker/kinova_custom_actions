#!/usr/bin/env python

from cmath import pi
import rospy
import actionlib
from kinova_custom_actions.msg import OdomNavigationFeedback, OdomNavigationResult, OdomNavigationAction, OdomNavigationGoal
from kinova_custom_actions.msg import MoveDistanceAction, TurnAngleAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import math
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

STOP_DISTANCE = 0.25
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class OdomNavigationServer(object):

    _feedback = OdomNavigationFeedback()
    _result = OdomNavigationResult()

    def __init__(self, name, cmd_vel_topic, odom_topic, linear_speed, angular_speed, laser_stop = False, laser_topic = ""):
        self._action_name = name
        self._linear_speed = linear_speed
        self._angular_speed = angular_speed

        self._x = 0
        self._y = 0

        self._x_init = 0
        self._y_init = 0

        self._yaw_init = 0
        self._yaw = 0

        self._reverse_threshold = (2.0 / 3.0) * pi

        self._cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber (odom_topic, Odometry, self.odom_callback)

        self._as = actionlib.SimpleActionServer(self._action_name, OdomNavigationAction, execute_cb=self.odom_navigation_action_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Odometry navigation server started")

        self._min_scan_distance = 1.0
        self._laser_stop = laser_stop
        if (self._laser_stop):
            self._scan_sub = rospy.Subscriber(laser_topic, LaserScan, self.scan_callback)

    def odom_navigation_action_cb(self, goal_pose):
        
        success = True
        reverse = 1

        rospy.loginfo("Requested pose x: %f, y: %f, theta: %f" % (goal_pose.relative_goal_pose_2d.x, goal_pose.relative_goal_pose_2d.y, goal_pose.relative_goal_pose_2d.theta))

        turn_angle_1 = math.atan2(goal_pose.relative_goal_pose_2d.y, goal_pose.relative_goal_pose_2d.x)

        if (abs(turn_angle_1) > self._reverse_threshold):
            rospy.loginfo("Reversing: Turn anle 1, %f > threshold, %f" % (turn_angle_1, self._reverse_threshold))
            reverse = -1
            turn_angle_1 -= math.copysign(pi, turn_angle_1)

        angle_turned = 0
        angular_z_cmd = Vector3(z=math.copysign(self._angular_speed, turn_angle_1))
        twist_command = Twist(angular = angular_z_cmd)
        self._yaw_init = self._yaw

        while (abs(angle_turned) < abs(turn_angle_1)):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            angle_turned = self.get_angle_turned(self._yaw, self._yaw_init)

            self._cmd_pub.publish(twist_command)
            self._feedback.start_angle_turned = angle_turned
            self._as.publish_feedback(self._feedback)

        rospy.loginfo("Start turn (rad): %f" % angle_turned)

        self._cmd_pub.publish(Twist())  # Stop robot


        if (success):

            self._x_init = self._x
            self._y_init = self._y

            distance_moved = 0
            move_distance = reverse * math.sqrt(goal_pose.relative_goal_pose_2d.x**2 + goal_pose.relative_goal_pose_2d.y**2)
        
            linear_x_cmd = Vector3(x=math.copysign(self._linear_speed, move_distance))
            twist_command = Twist(linear=linear_x_cmd)

            while(distance_moved < abs(move_distance)):

                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                
                distance_moved = self.get_distance_moved(self._x, self._y, self._x_init, self._y_init)

                # rospy.loginfo(str(self.check_safety_stop()))
                if (self._laser_stop and self.check_safety_stop()):
                    rospy.loginfo('Safety Stop!')
                    self._cmd_pub.publish(Twist())  # Stop robot
                    self._as.set_aborted(result=self._result)
                    return
                else:
                    self._cmd_pub.publish(twist_command)
                    self._feedback.distance_moved = distance_moved
                    self._as.publish_feedback(self._feedback)
        
        rospy.loginfo("Distance moved (m): %f" % distance_moved)

        self._cmd_pub.publish(Twist())  # Stop robot

        if (success):
            turn_angle_2 = goal_pose.relative_goal_pose_2d.theta - turn_angle_1

            if (turn_angle_2 > math.pi): 
                turn_angle_2 -= 2* math.pi
            if (turn_angle_2 < -math.pi):
                turn_angle_2 += 2* math.pi

            angle_turned = 0
            angular_z_cmd = Vector3(z=math.copysign(self._angular_speed, turn_angle_2))
            twist_command = Twist(angular = angular_z_cmd)
            self._yaw_init = self._yaw

            while (abs(angle_turned) < abs(turn_angle_2)):
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break

                angle_turned = self.get_angle_turned(self._yaw, self._yaw_init)

                self._cmd_pub.publish(twist_command)
                self._feedback.final_angle_turned = angle_turned
                self._as.publish_feedback(self._feedback)

            rospy.loginfo("Final turn (rad): %f" % angle_turned)

            self._cmd_pub.publish(Twist())  # Stop robot

        if (success):
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(result=self._result)
        else:
            rospy.loginfo('%s: Failed or preempted' % self._action_name)


    def get_distance_moved(self, current_x, current_y, start_x, start_y):
        return math.sqrt((current_x - start_x) * (current_x - start_x) + (current_y - start_y) * (current_y - start_y))
    

    def get_angle_turned(self, current_angle, start_angle):
        angle_turned = current_angle - start_angle

        # 0.2 included as buffer otherwise robot can rotate indefinitely when set a goal of pi rad turn
        # ... hacky fix, needs to be replaced
        if (angle_turned > math.pi + 0.2): 
            angle_turned -= 2* math.pi
        if (angle_turned < -math.pi - 0.2):
            angle_turned += 2* math.pi

        return angle_turned

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self._yaw) = euler_from_quaternion (orientation_list)

        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

    def scan_callback(self, data):
        if (self._as.is_active):
            self._min_scan_distance = min(data.ranges)

    def check_safety_stop(self):     

        if self._min_scan_distance < SAFE_STOP_DISTANCE:
            return True
        else:
            return False


def main():
    rospy.init_node('odom_navigation_action_server')

    linear_speed = rospy.get_param('~linear_speed', 0.3)
    angular_speed = rospy.get_param('~angular_speed', 0.5)
    server = OdomNavigationServer(rospy.get_name(), 
                                    cmd_vel_topic="/cmd_vel", 
                                    odom_topic="/odometry/filtered",
                                    linear_speed=linear_speed,
                                    angular_speed=angular_speed,
                                    laser_stop=True,
                                    laser_topic='/scan_filtered')

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()