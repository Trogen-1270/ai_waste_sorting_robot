#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
from robot_arm_urdf.srv import GetTargetPose

class PickAndPlaceClient:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_and_place_client')

        self.arm_group = moveit_commander.MoveGroupCommander("arm_group")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.arm_group.set_goal_position_tolerance(0.01)
        self.arm_group.set_planning_time(10)

        rospy.loginfo("✅ Robot Controller Client is ready.")

    def call_detector_service(self, label):
        rospy.loginfo("Waiting for perception service...")
        rospy.wait_for_service('get_target_pose')
        try:
            get_target_pose = rospy.ServiceProxy('get_target_pose', GetTargetPose)
            response = get_target_pose(label)
            if response.success:
                rospy.loginfo("Pose received successfully!")
                return response.pose
            else:
                rospy.logerr(f"Perception service failed to find '{label}'.")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def run_pick_place(self):
        # 1. Go to rest, open gripper
        self.arm_group.set_named_target("rest_pose")
        self.arm_group.go(wait=True)
        self.gripper_group.set_named_target("gripper_open")
        self.gripper_group.go(wait=True)
        rospy.sleep(1)

        # 2. Get can's location
        can_pose = self.call_detector_service("red_can")
        if can_pose is None: return

        # 3. Go to "pick_ready" pose
        self.arm_group.set_named_target("pick_ready")
        self.arm_group.go(wait=True)

        # 4. Create an approach pose (aligned in X and Y)
        approach_pose = self.arm_group.get_current_pose().pose
        approach_pose.position.x = can_pose.position.x
        approach_pose.position.y = can_pose.position.y

        rospy.loginfo("Moving to align with can (X,Y).")
        self.arm_group.set_pose_target(approach_pose)
        if not self.arm_group.go(wait=True):
            rospy.logerr("Motion plan to align with can failed.")
            return

        # 5. ✅ FINAL STEP: Go down to grasp the can
        rospy.loginfo("Moving down to grasp.")
        grasp_pose = approach_pose
        grasp_pose.position.z -= 0.08 # Move down 8cm from approach pose
        self.arm_group.set_pose_target(grasp_pose)
        if not self.arm_group.go(wait=True):
            rospy.logerr("Motion plan to grasp pose failed.")
            return
        rospy.sleep(1)

        # 6. Close gripper, lift, and return home
        self.gripper_group.set_named_target("gripper_closed")
        self.gripper_group.go(wait=True)
        rospy.sleep(1)

        rospy.loginfo("Lifting can.")
        # Use shift_pose_target for a simple straight-up motion
        self.arm_group.shift_pose_target(2, 0.15) # Index 2 is the Z-axis, move up 15cm
        self.arm_group.go(wait=True)

        rospy.loginfo("Moving to rest_pose.")
        self.arm_group.set_named_target("rest_pose")
        self.arm_group.go(wait=True)

        rospy.loginfo("🎉🎉🎉 MISSION COMPLETE! 🎉🎉🎉")
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        controller = PickAndPlaceClient()
        controller.run_pick_place()
    except rospy.ROSInterruptException:
        pass