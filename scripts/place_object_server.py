#! /usr/bin/env python

import rospy

import actionlib

import kinova_custom_actions.msg

from kortex_driver.msg import BaseCyclic_Feedback, BaseFeedback, TwistCommand, Twist

class PlaceObjectAction(object):
    _feedback = kinova_custom_actions.msg.PlaceObjectFeedback()
    _result = kinova_custom_actions.msg.PlaceObjectResult()
    _wrench_force_z = 0.0
    _move_speed = 0.03
    _move_down_twist_msg = Twist(linear_z=-_move_speed)
    _stop_twist_msg = Twist()

    def __init__(self, name, base_feedback_topic, cartesian_vel_pub_topic):
        self._action_name = name
        self._cmd_pub = rospy.Publisher(cartesian_vel_pub_topic, TwistCommand, queue_size=10)
        self._base_feedback_topic = base_feedback_topic
        self._base_feedback_sub = rospy.Subscriber(base_feedback_topic, BaseCyclic_Feedback, callback=self.base_feedback_sub)
        self._as = actionlib.SimpleActionServer(self._action_name, kinova_custom_actions.msg.PlaceObjectAction, execute_cb=self.execute_action_cb, auto_start = False)
        rospy.loginfo("Place action server started")
        self._as.start()

    def execute_action_cb(self, goal):
        # helper variables
        rate = rospy.Rate(50)
        success = True

        rospy.loginfo('Placing object vertically down')

        while(self._wrench_force_z < 10.0):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            
            twist_command = TwistCommand(twist=self._move_down_twist_msg)
            self._cmd_pub.publish(twist_command)
            self._feedback.wrench_force_z = self._wrench_force_z
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        rospy.loginfo("measured force: %f" % self._wrench_force_z)
        
        twist_command = TwistCommand(twist=self._stop_twist_msg)
        self._cmd_pub.publish(twist_command)

        if (success):
            rospy.loginfo('%s: Succeeded' % self._action_name)

        self._result = success
        self._as.set_succeeded(self._result)

    
    def base_feedback_sub(self, data):
        self._wrench_force_z = data.base.tool_external_wrench_force_z
        # rospy.loginfo("Updating measured force: %f" % self._wrench_force_z)

def main():
    rospy.init_node('kinova_place_object_server')
    server = PlaceObjectAction(rospy.get_name(), '/kinova_arm/base_feedback', '/kinova_arm/in/cartesian_velocity')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")




if __name__ == '__main__':
    main()