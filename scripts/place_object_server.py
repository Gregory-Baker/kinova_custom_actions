#! /usr/bin/env python

from time import time
import rospy
import actionlib
from std_msgs.msg import Empty
import kinova_custom_actions.msg
from kortex_driver.msg import BaseCyclic_Feedback, TwistCommand, Twist
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class PlaceObjectAction(object):
    _feedback = kinova_custom_actions.msg.PlaceObjectFeedback()
    _result = kinova_custom_actions.msg.PlaceObjectResult()
    _wrench_force_z = 0.0
    _move_speed = 0.05
    _move_down_twist_msg = Twist(linear_z=-_move_speed)
    _move_up_twist_msg = Twist(linear_z=_move_speed)
    _stop_twist_msg = Twist()
    _max_wrench_force_z = 20.0

    def __init__(self, place_action_name, pick_action_name, base_feedback_topic, cartesian_vel_pub_topic, stop_arm_topic, gripper_action_ns):
        self._place_action_name = place_action_name
        self._pick_action_name = pick_action_name
        self._cmd_pub = rospy.Publisher(cartesian_vel_pub_topic, TwistCommand, queue_size=10)
        self._base_feedback_topic = base_feedback_topic
        self._base_feedback_sub = rospy.Subscriber(base_feedback_topic, BaseCyclic_Feedback, callback=self.base_feedback_sub)
        self._stop_arm_sub = rospy.Subscriber(stop_arm_topic, Empty, callback=self.cancel_goal)
        self.gripper_action_client = actionlib.SimpleActionClient(gripper_action_ns, GripperCommandAction)
        self.gripper_action_client.wait_for_server()

        self._as_place = actionlib.SimpleActionServer(self._place_action_name, kinova_custom_actions.msg.PlaceObjectAction, execute_cb=self.execute_place_action_cb, auto_start = False)
        self._as_place.start()
        rospy.loginfo("Place action server started")

        
        self._as_pick = actionlib.SimpleActionServer(self._pick_action_name, kinova_custom_actions.msg.PlaceObjectAction, execute_cb=self.execute_pick_action_cb, auto_start = False)
        self._as_pick.start()
        rospy.loginfo("Pick action server started")


    def execute_place_action_cb(self, goal):

        # helper variables
        rate = rospy.Rate(50)
        success = True

        rospy.loginfo('Placing object vertically down')

        twist_command = TwistCommand(twist=self._move_down_twist_msg)

        while(self._wrench_force_z < self._max_wrench_force_z):
            if self._as_place.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._place_action_name)
                self._as_place.set_preempted()
                success = False
                break
            
            self._cmd_pub.publish(twist_command)
            self._feedback.wrench_force_z = self._wrench_force_z
            self._as_place.publish_feedback(self._feedback)
            rate.sleep()

        rospy.loginfo("Measured force: %f" % self._wrench_force_z)
        
        twist_command = TwistCommand(twist=self._stop_twist_msg)
        self._cmd_pub.publish(twist_command)

        if (success):

            self.open_gripper()
            
            time = 0
            time_start = rospy.Time.now().to_sec()
            twist_command = TwistCommand(twist=self._move_up_twist_msg)

            while(time < 2.5):
                if self._as_place.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._place_action_name)
                    self._as_place.set_preempted()
                    success = False
                    break
                
                self._cmd_pub.publish(twist_command)
                time = rospy.Time.now().to_sec() - time_start

            twist_command = TwistCommand(twist=self._stop_twist_msg)
            self._cmd_pub.publish(twist_command)


        if (success):
            rospy.loginfo('%s: Succeeded' % self._place_action_name)
            self._result.success = success
            self._as_place.set_succeeded(result=self._result)

    def execute_pick_action_cb(self, goal):

        # helper variables
        rate = rospy.Rate(50)
        success = True

        rospy.loginfo('Picking Object')

        self.open_gripper()

        twist_command = TwistCommand(twist=self._move_down_twist_msg)

        while(self._wrench_force_z < self._max_wrench_force_z):
            if self._as_pick.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._pick_action_name)
                self._as_pick.set_preempted()
                success = False
                break
            
            self._cmd_pub.publish(twist_command)
            self._feedback.wrench_force_z = self._wrench_force_z
            self._as_pick.publish_feedback(self._feedback)
            rate.sleep()

        rospy.loginfo("Measured force: %f" % self._wrench_force_z)
        
        twist_command = TwistCommand(twist=self._stop_twist_msg)
        self._cmd_pub.publish(twist_command)

        if (success):

            time = 0
            time_start = rospy.Time.now().to_sec()
            twist_command = TwistCommand(twist=self._move_up_twist_msg)

            while(time < 0.5):
                if self._as_pick.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._pick_action_name)
                    self._as_pick.set_preempted()
                    success = False
                    break
                
                self._cmd_pub.publish(twist_command)
                time = rospy.Time.now().to_sec() - time_start

            twist_command = TwistCommand(twist=self._stop_twist_msg)
            self._cmd_pub.publish(twist_command)

            if (success):
                self.close_gripper()

            time = 0
            time_start = rospy.Time.now().to_sec()
            twist_command = TwistCommand(twist=self._move_up_twist_msg)

            if (success):
                while(time < 2.5):
                    if self._as_pick.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._pick_action_name)
                        self._as_pick.set_preempted()
                        success = False
                        break
                    
                    self._cmd_pub.publish(twist_command)
                    time = rospy.Time.now().to_sec() - time_start

            twist_command = TwistCommand(twist=self._stop_twist_msg)
            self._cmd_pub.publish(twist_command)


        if (success):
            rospy.loginfo('%s: Succeeded' % self._pick_action_name)
            self._result.success = success
            self._as_pick.set_succeeded(result=self._result)


    def open_gripper(self):
        grip_cmd = GripperCommandGoal()
        grip_cmd.command.position = 0.0
        self.gripper_action_client.send_goal(grip_cmd)
        self.gripper_action_client.wait_for_result()

    def close_gripper(self):
        grip_cmd = GripperCommandGoal()
        grip_cmd.command.position = 0.8
        self.gripper_action_client.send_goal(grip_cmd)
        self.gripper_action_client.wait_for_result()

    
    def base_feedback_sub(self, data):
        self._wrench_force_z = data.base.tool_external_wrench_force_z


    
    def cancel_goal(self, data):
        self._as_place.preempt_request = True


def main():
    rospy.init_node('kinova_place_object_server')
    server = PlaceObjectAction(rospy.get_name(),
                                'kinova_pick_object_server',
                                '/kinova_arm/base_feedback', 
                                '/kinova_arm/in/cartesian_velocity', 
                                '/kinova_arm/in/stop',
                                '/kinova_arm/kinova_arm_robotiq_2f_85_gripper_controller/gripper_cmd')
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()