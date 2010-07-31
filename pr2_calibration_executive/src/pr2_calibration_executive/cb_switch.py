#!/usr/bin/env python

import roslib; roslib.load_manifest("pr2_calibration_launch")
import rospy
import actionlib

from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *

r_joint_names = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
l_joint_names = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']

l_arm_traj_1 = [[0.06902974179422372, 0.66061040571038554, 0.11791529759328223, -1.6643821056659893, -23.633336292371826, -0.098284679326103186, 0.0064751806095266762]]

r_arm_traj_1 = [[-0.21065663550227953, 0.021057696894857078, -0.33177998073304127, -0.89190556944782906, 5.048362590336815, -1.3989014948790031, -3.2121962517688809],[0.041711107732385067, 0.29074631175185855, -0.163086703391921, -1.3273758692894804, 4.4621980582667859, -1.2637943543446366, -2.9487062316457382]] 


l_arm_traj_2 =[[0.22166082434843459, 0.12453237799745623, 0.20755364553404876, -0.99394894124887234, -23.845635101480383, -1.6531194480052087, 0.28832596501469898], [0.12109509749441094, 0.080796802752453115, 0.29975767069673331, -1.0205867521434415, -23.845635101480383, -1.4693087635577893, 0.28832596501469898]] 

r_arm_traj_2 = [[-0.36088342615313773, 0.21435709618853602, -0.296982983410776, -1.2257468298982439, 5.0520648039016507, -1.3478220226820787, -3.2732827755887115]]

r_arm_traj_3 = [[-0.44228583725469994, 0.2258519682931889, -0.24795370891385282, -1.2019200363695242, -89.147789488363188, -1.4729128739354223, -3.3501060428921523], [0.16948209222249799, 1.2620551813996543, 0.40292653132151535, -2.0511450293452977, -89.453164260375203, -0.10499242600417369, 0.0], [0.47781180217312824, 0.56431625563411192, 0.45712646263453682, -1.6557762165568837, -89.857284009811806, -0.10877770205283921, 0.0]] 

l_arm_traj_3 = [[0.8167984721739604, 0.21639885046221027, 0.44461739023511071, -0.80894843287953355, 0.97444581574990652, -0.097490072086086066, -11.919077167787565]]


right_client = None
left_client = None

left_grip_client = None
right_grip_client = None

def move_right_arm(traj):
    r_joint_names = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']

    global right_client
    goal = JointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names = r_joint_names
    for i in range(len(traj)):
      goal.trajectory.points.append(JointTrajectoryPoint())
      goal.trajectory.points[i].positions = traj[i]
      goal.trajectory.points[i].time_from_start = rospy.Duration(5.0 * i + 5.0)
    right_client.send_goal(goal)
    right_client.wait_for_result()

def move_left_arm(traj):
    l_joint_names = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
    
    global left_client
    goal = JointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names = l_joint_names
    for i in range(len(traj)):
      goal.trajectory.points.append(JointTrajectoryPoint())
      goal.trajectory.points[i].positions = traj[i]
      goal.trajectory.points[i].time_from_start = rospy.Duration(5.0 * i + 5.0)
    left_client.send_goal(goal)
    left_client.wait_for_result()

def move_left_grip(val):
    global left_grip_client
    
    goal = Pr2GripperCommandGoal()
    goal.command.position = val
    goal.command.max_effort = -1.0
    left_grip_client.send_goal(goal)
    left_grip_client.wait_for_result()

def move_right_grip(val):
    global right_grip_client

    goal = Pr2GripperCommandGoal()
    goal.command.position = val
    goal.command.max_effort = -1.0
    right_grip_client.send_goal(goal)
    right_grip_client.wait_for_result()

def run():
    global right_client
    global left_client
    global right_grip_client
    global left_grip_client
    right_client = actionlib.SimpleActionClient("/r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
    left_client = actionlib.SimpleActionClient("/l_arm_controller/joint_trajectory_action", JointTrajectoryAction)

    right_grip_client = actionlib.SimpleActionClient("r_gripper_controller/gripper_action", Pr2GripperCommandAction)
    left_grip_client = actionlib.SimpleActionClient("l_gripper_controller/gripper_action", Pr2GripperCommandAction)

    right_client.wait_for_server()
    left_client.wait_for_server() 

    right_grip_client.wait_for_server()
    left_grip_client.wait_for_server()

    move_right_grip(1.0)
 
    move_left_arm(l_arm_traj_1)
    move_right_arm(r_arm_traj_1)

    move_right_grip(0.0)
    rospy.sleep(1)
    move_left_grip(1.0)
    

    move_right_arm(r_arm_traj_2)
    move_left_arm(l_arm_traj_2)    

    move_left_grip(0.0)
    rospy.sleep(1)
    move_right_grip(1.0)
    move_right_arm(r_arm_traj_3)
        
    move_right_grip(0.0)
    rospy.sleep(1)
    move_left_grip(1.0)

    move_left_arm(l_arm_traj_3)
    move_left_grip(0.0)

if __name__ == "__main__":
    rospy.init_node("cb_switch")
    run()
