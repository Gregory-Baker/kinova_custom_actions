#!/usr/bin/env python
from scipy import angle
import rospy
import actionlib
from kinova_custom_actions.msg import MoveDistanceFeedback, MoveDistanceResult, MoveDistanceAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import math
from sensor_msgs.msg import LaserScan

STOP_DISTANCE = 0.25
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class MoveDistanceServer(object):
    _feedback = MoveDistanceFeedback()
    _result = MoveDistanceResult()
    _goal_distance = 0

    _move_speed = 0.3   # m/s

    def __init__(self, name, cmd_vel_topic="/cmd_vel", odom_topic="/odom", scan_topic="/scan_filtered"):
        self.previous_x = 0
        self.previous_y = 0
        self.distance_moved = 0.0
        self.first_run = True

        self._action_name = name
        self._cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber (odom_topic, Odometry, self.get_distance_moved)
        self._scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)

        self._as = actionlib.SimpleActionServer(self._action_name, MoveDistanceAction, execute_cb=self.execute_action_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Move robot forward/back action server started")

    def scan_callback(self, data):
        if (self._as.is_active):
            min_distance = min(data.ranges)

            if min_distance < SAFE_STOP_DISTANCE and self._goal_distance > 0:
                self._as.preempt_request = True
                self._safety_stop = True
            else:
                self._safety_stop = False

    def execute_action_cb(self, goal):
        self._goal_distance = goal.move_distance
        success = True

        self.first_run = True
        self.distance_moved = 0.0

        linear_x_cmd = Vector3(x=math.copysign(self._move_speed, goal.move_distance))
        twist_command = Twist(linear=linear_x_cmd)


        rospy.loginfo("Requested move distance: %f" % goal.move_distance)

        while(self.distance_moved < abs(goal.move_distance)):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            self._cmd_pub.publish(twist_command)
            self._feedback.distance_moved = self.distance_moved
            self._as.publish_feedback(self._feedback)


        rospy.loginfo("Distance moved: %f" % self.distance_moved)

        self._cmd_pub.publish(Twist())  # Stop robot
        self._goal_distance = 0

        if (success):
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.distance_moved = self._feedback.distance_moved
            self._as.set_succeeded(result=self._result)


    def get_distance_moved(self, data):
        if(self.first_run):
            self.previous_x = data.pose.pose.position.x
            self.previous_y = data.pose.pose.position.y
            self.first_run = False
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.distance_moved = math.sqrt((x - self.previous_x) * (x - self.previous_x) + (y - self.previous_y) * (y - self.previous_y))


def main():
    rospy.init_node('move_robot_action_server')

    server = MoveDistanceServer(rospy.get_name(), odom_topic="/husky_velocity_controller/odom")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()