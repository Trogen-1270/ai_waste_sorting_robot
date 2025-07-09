#!/usr/bin/env python3
import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def publish_joint_command():
    # Initialize the ROS node
    rospy.init_node('pid_tuner_joint5', anonymous=True)
    rospy.loginfo("--> PID Tuner Node Initialized.")
    
    # Create a publisher for the arm controller's command topic
    pub = rospy.Publisher('/arm_group_controller/command', JointTrajectory, queue_size=10)
    rospy.loginfo("--> Publisher Created. Waiting for connection...")
    
    # Wait until the publisher has connected to a subscriber
    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)
    rospy.loginfo("--> Publisher Connected!")
    
    # Set the loop rate (e.g., 100 Hz)
    rate = rospy.Rate(100)
    
    # Create a reusable JointTrajectory message
    traj_msg = JointTrajectory()
    traj_msg.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        # Calculate the elapsed time
        elapsed_time = rospy.get_time() - start_time
        
        # Calculate the desired position using a sine wave
        amplitude = 1.0
        frequency = 0.2
        desired_pos = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
        
        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, desired_pos]
        point.time_from_start = rospy.Duration(0.1)
        
        # Add the point to the trajectory message
        traj_msg.points = [point]
        
        # Publish the message
        rospy.loginfo(f"Publishing command for joint_5: {desired_pos:.2f}")
        pub.publish(traj_msg)
        
        # Wait for the next cycle
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_command()
    except rospy.ROSInterruptException:
        pass