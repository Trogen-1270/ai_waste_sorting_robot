#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from robot_arm_urdf.srv import GetTargetPose # Import our new service

class PickAndPlaceClient:
    def __init__(self):
        rospy.init_node('pick_and_place_client_node', anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_group = moveit_commander.MoveGroupCommander("arm_group")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.run_routine()

    def run_routine(self):
        rospy.loginfo("Waiting for detection service...")
        rospy.wait_for_service('get_target_pose_service')

        try:
            # Create a handle to the service
            get_target_pose = rospy.ServiceProxy('get_target_pose_service', GetTargetPose)

            # Call the service to get the object's pose
            rospy.loginfo("Requesting target pose from detector...")
            response = get_target_pose()

            if response.target_pose:
                target_pose = response.target_pose
                rospy.loginfo(f"Got target pose: X={target_pose.position.x:.2f}, Y={target_pose.position.y:.2f}")

                # --- Now we execute the same pick and place routine ---
                self.arm_group.set_named_target("gripper_open")
                self.gripper_group.go(wait=True)

                pre_pick_pose = Pose()
                pre_pick_pose.orientation.w = 1.0
                pre_pick_pose.position.x = target_pose.position.x
                pre_pick_pose.position.y = target_pose.position.y
                pre_pick_pose.position.z = target_pose.position.z + 0.15

                self.arm_group.set_pose_target(pre_pick_pose)
                self.arm_group.go(wait=True)

                # ... (The rest of the pick and place steps would go here) ...

                rospy.loginfo("Routine complete.")
            else:
                rospy.logerr("Service call failed or no object was detected.")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        rospy.signal_shutdown("Task finished.")

if __name__ == '__main__':
    try:
        PickAndPlaceClient()
    except rospy.ROSInterruptException:
        pass