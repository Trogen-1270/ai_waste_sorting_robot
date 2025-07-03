#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def move_robot_to_pose():
    # --- Initialization ---
    # Initialize the moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_pose_node', anonymous=True)

    # Instantiate a RobotCommander object. This object is the outer-level interface to the robot.
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints.
    # In this case, the group is the joints in the 'arm_group'. This is the object we use to plan and execute motions.
    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    
    rospy.loginfo("MoveIt! and ROS have been initialized.")
    rospy.loginfo("Planning frame: %s" % arm_group.get_planning_frame())
    rospy.loginfo("End effector link: %s" % arm_group.get_end_effector_link())
    rospy.loginfo("Available planning groups: %s" % robot.get_group_names())

    # --- Moving the Robot ---
    # Set the named target for the arm. These are the poses you created in the MoveIt! Setup Assistant.
    # Make sure the name matches one of your saved poses.
    named_target = "straight_up"
    rospy.loginfo(f"Setting pose target to: {named_target}")
    arm_group.set_named_target(named_target)

    # Plan and execute the motion.
    rospy.loginfo("Planning and executing motion...")
    arm_group.go(wait=True)
    
    # Calling stop() ensures that there is no residual movement
    arm_group.stop()
    
    rospy.loginfo("Motion complete. Shutting down.")
    
    # --- Shutdown ---
    # Shut down moveit_commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_robot_to_pose()
    except rospy.ROSInterruptException:
        pass