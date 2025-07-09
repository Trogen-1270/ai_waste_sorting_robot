#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64

def publish_joint_commands():
    # ==================================================
    # CHANGE THIS TOPIC TO TUNE A DIFFERENT JOINT
    # e.g., '/joint_4_controller/command'
    # ==================================================
    topic_name = '/joint_5_controller/command'

    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    rospy.init_node('joint_tuner_simple', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    
    start_time = rospy.get_time()
    rospy.loginfo("Starting to publish sine wave commands to %s", topic_name)

    while not rospy.is_shutdown():
        # Generate a sine wave command with an amplitude of 1.0 radian (~57 degrees)
        elapsed_time = rospy.get_time() - start_time
        angle = 1.0 * math.sin(elapsed_time * 0.5) # The 0.5 slows the wave down a bit
        
        # Publish the command
        pub.publish(angle)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_commands()
    except rospy.ROSInterruptException:
        pass