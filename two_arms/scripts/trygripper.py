import argparse
import geometry_msgs.msg
import rospy, sys, numpy as np
import actionlib
import control_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty

def gripper_client1(value):

    # Create an action client
    client = actionlib.SimpleActionClient(
        '/robot1/gripper_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    # Wait until the action server has been started and is listening for goals
    client.wait_for_server()

    # Create a goal to send (to the action server)
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value   # From 0.0 to 0.8
    goal.command.max_effort = -1 # Do not limit the effort
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()

rospy.init_node('gripper_command_robot1')
while not rospy.is_shutdown():
    gripper_client1(-0.5)
    rospy.sleep(1)
    gripper_client1(0.0)
    rospy.sleep(1)
