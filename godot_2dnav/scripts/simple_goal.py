#! /usr/bin/env python

import roslib; roslib.load_manifest('move_base')
import rospy
import actionlib

# Brings in the SimpleActionClient
import move_base_msgs.msg

from move_base_msgs.msg import MoveBaseGoal

def simple_goal_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
 
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0

    #ROS_INFO("Sending goal")
    client.send_goal(goal)
    
    client.wait_for_result()
    
    #if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    #    ROS_INFO("Hooray, the base moved 1 meter forward")
    #else
    #    ROS_INFO("The base failed to move forward 1 meter for some reason")

    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('goal_client_py')
        result = simple_goal_client()
        print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

