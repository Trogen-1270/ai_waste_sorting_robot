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
        rospy.loginfo("✅ Robot Controller Client is ready.")

    def call_detector_service(self, label):
        rospy.loginfo("Waiting for perception service...")
        rospy.wait_for_service('get_target_pose')
        try:
            get_target_pose = rospy.ServiceProxy('get_target_pose', GetTargetPose)
            rospy.loginfo(f"Requesting pose for '{label}'...")
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
        # 1. Go to a neutral starting position
        self.arm_group.set_named_target("rest_pose")
        self.arm_group.go(wait=True)
        self.gripper_group.set_named_target("gripper_open")
        self.gripper_group.go(wait=True)
        rospy.sleep(1)

        # 2. Ask the perception service for the object's location
        target_pose = self.call_detector_service("red_can")

        if target_pose is None:
            rospy.logerr("Halting execution. Cannot proceed without target pose.")
            return

        # 3. Go to the object and pick it up
        rospy.loginfo("Moving to pick object.")
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True)

        self.gripper_group.set_named_target("gripper_closed")
        self.gripper_group.go(wait=True)
        rospy.sleep(1)

        # 4. Lift the object
        lift_pose = target_pose
        lift_pose.position.z += 0.15 # Lift 15cm
        self.arm_group.set_pose_target(lift_pose)
        self.arm_group.go(wait=True)

        # 5. Go to drop-off location and release
        rospy.loginfo("Moving to drop-off location.")
        self.arm_group.set_named_target("home") # Or another predefined pose
        self.arm_group.go(wait=True)

        self.gripper_group.set_named_target("gripper_open")
        self.gripper_group.go(wait=True)

        rospy.loginfo("🎉 Pick and place routine complete!")
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        controller = PickAndPlaceClient()
        controller.run_pick_place()
    except rospy.ROSInterruptException:
        pass