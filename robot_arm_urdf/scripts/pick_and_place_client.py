#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory

class PickAndPlaceClient:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_and_place_client_debugger')

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm_group")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
        self.arm_group.set_planning_time(10.0)
        rospy.loginfo("✅ Robot Controller Debug Client is ready.")

    def call_detector_service(self, label):
        rospy.loginfo("Using hardcoded target for debugging.")
        hardcoded_pose = Pose()
        # Using a proper "pointing down" orientation
        hardcoded_pose.orientation.y = 1.0
        hardcoded_pose.orientation.w = 0.0
        # These are the coordinates where the can is physically located in Gazebo
        hardcoded_pose.position.x = 0.0
        hardcoded_pose.position.y = -0.60 # Corrected to match your spawn location
        hardcoded_pose.position.z = 0.06
        return hardcoded_pose

    def plan_and_execute(self, goal_pose, description):
        rospy.loginfo(f"--- Planning: {description} ---")
        self.arm_group.set_pose_target(goal_pose)
        
        plan_success, plan, _, _ = self.arm_group.plan()

        if not plan_success:
            rospy.logerr(f"PLANNING FAILED for: {description}")
            return False

        rospy.loginfo(f"Plan found for: {description}. Press Enter to execute.")
        
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        
        input("Press Enter to execute the plan...")
        
        self.arm_group.execute(plan, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        rospy.sleep(1)
        return True

    def run_pick_and_place(self):
        # 1. Go to rest pose and open the gripper
        self.arm_group.set_named_target("rest_pose")
        self.arm_group.go(wait=True)
        self.gripper_group.set_named_target("gripper_open")
        self.gripper_group.go(wait=True)
        rospy.sleep(1)

        # 2. Get the can's location (using our hardcoded debug value)
        can_pose = self.call_detector_service("red_can")
        if can_pose is None: return

        # 3. Plan and execute the move to the pre-grasp pose
        pre_grasp_pose = Pose()
        pre_grasp_pose.orientation = can_pose.orientation
        pre_grasp_pose.position.x = can_pose.position.x
        pre_grasp_pose.position.y = can_pose.position.y
        pre_grasp_pose.position.z = can_pose.position.z + 0.12 # 12cm above can center

        if not self.plan_and_execute(pre_grasp_pose, "Move to Pre-Grasp"):
            return

        # 4. Plan and execute the move down to the grasp pose
        grasp_pose = pre_grasp_pose
        grasp_pose.position.z -= 0.12 # Move down to the can center

        if not self.plan_and_execute(grasp_pose, "Move to Grasp"):
            return

        rospy.loginfo("Pick successful! Returning home.")
        self.arm_group.set_named_target("rest_pose")
        self.arm_group.go(wait=True)
        rospy.loginfo("🎉🎉🎉 STAGE 1 COMPLETE! 🎉🎉🎉")

if __name__ == '__main__':
    try:
        controller = PickAndPlaceClient()
        controller.run_pick_and_place()
    except rospy.ROSInterruptException:
        pass