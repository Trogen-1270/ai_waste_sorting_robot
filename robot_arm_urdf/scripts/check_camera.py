#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def image_callback(msg):
    """
    Callback function to process the incoming image from the camera.
    """
    rospy.loginfo("Image received!")
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to an OpenCV image (in BGR format)
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Display the image in a window
        cv2.imshow("Camera Check", cv_image)
        
        # This line is crucial for the window to update and be interactive
        cv2.waitKey(1)
        
    except Exception as e:
        rospy.logerr(e)

def main():
    """
    Main function to initialize the ROS node and subscriber.
    """
    rospy.init_node('camera_check_node')
    
    # ✅ Corrected topic name
    image_topic = "/waste_camera/image_raw"
    
    rospy.Subscriber(image_topic, Image, image_callback)
    
    rospy.loginfo(f"Subscribing to {image_topic}. Waiting for images...")
    
    # Keep the node running
    rospy.spin()
    
    # Clean up OpenCV windows when the node is shut down
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()