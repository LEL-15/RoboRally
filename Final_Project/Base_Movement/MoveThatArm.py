#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import rospy
import geometry_msgs.msg
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from std_msgs.msg import UInt8, UInt16
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import time


state = UInt16(0)

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

def callback_update_state(data):
  global state
  #DONE: Load data into your program's local state variables
  state = data.data
  print(state)

def main():

    global state

    rospy.init_node('MoveThatArm', anonymous=True)
    subscriber_state = rospy.Subscriber('/state', UInt16, callback_update_state)
    publisher_state = rospy.Publisher('/state', UInt16, queue_size = 10)

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    while not rospy.is_shutdown():

        if (state == 3):

            group = moveit_commander.MoveGroupCommander("arm_with_torso")

            raise_torso = geometry_msgs.msg.Pose()
            raise_torso.position.x = 0.0553662361377
            raise_torso.position.y = 0.0
            raise_torso.position.z = 0.91
            raise_torso.orientation.x = 0.459828548039
            raise_torso.orientation.y = -0.503145155084
            raise_torso.orientation.z = 0.511628918876
            raise_torso.orientation.w = 0.523104682347 


            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = 0.9
            pose_goal.position.y = 0.15
            pose_goal.position.z = 0.9
            pose_goal.orientation.x = 0.1
            pose_goal.orientation.y = -0.503145155084
            pose_goal.orientation.z = 0.511628918876
            pose_goal.orientation.w = 0.523104682347

#  Original Pose: 
#   position: 
#     x: 0.0553662361377
#     y: -0.139643269349
#     z: 0.571328129145
#   orientation: 
#     x: 0.459828548039
#     y: -0.503145155084
#     z: 0.511628918876
#     w: 0.523104682347                

        # print(pose_goal)


            # group.set_pose_target(raise_torso)
            # ## Now, we call the planner to compute the plan and execute it.
            # plan = group.go(wait=True)
            # # Calling `stop()` ensures that there is no residual movement
            # group.stop()
            # # It is always good to clear your targets after planning with poses.
            # # Note: there is no equivalent function for clear_joint_value_targets()
            # group.clear_pose_targets()

            # # Give robot time to move arm
            # time.sleep(10)

            # print("raised?")

            group.set_pose_target(pose_goal)
            ## Now, we call the planner to compute the plan and execute it.
            plan = group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            group.clear_pose_targets()

            # Give robot time to move arm
            time.sleep(10)            

            publisher_state.publish(UInt16(4))




        elif (state == 5):
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = 0.0553662361377
            pose_goal.position.y = -0.139643269349
            pose_goal.position.z = 0.571328129145
            pose_goal.orientation.x = 0.459828548039
            pose_goal.orientation.y = -0.503145155084
            pose_goal.orientation.z = 0.511628918876
            pose_goal.orientation.w = 0.523104682347     

            group.set_pose_target(raise_torso)
            ## Now, we call the planner to compute the plan and execute it.
            plan = group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            group.clear_pose_targets()

            # Give robot time to move arm
            time.sleep(10)
            publisher_state.publish(UInt16(7))



    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()
  
# Helpful Websites:
# https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py    

if __name__ == '__main__':
    main()