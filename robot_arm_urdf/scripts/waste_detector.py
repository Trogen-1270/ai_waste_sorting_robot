#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector_node', anonymous=True)
        self.bridge = CvBridge()

        # Subscribe to the camera's image topic
        self.image_sub = rospy.Subscriber("/waste_camera/image_raw", Image, self.image_callback)

        # Create publishers for the detected object coordinates
        self.can_pub = rospy.Publisher("/detected_can_pose", PointStamped, queue_size=10)
        self.bottle_pub = rospy.Publisher("/detected_bottle_pose", PointStamped, queue_size=10)

        rospy.loginfo("Color Detector node started.")

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convert the image from BGR to HSV (Hue, Saturation, Value) color space
        # HSV is much better for color detection than standard RGB
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # --- Detect Red Can ---
        # Define the range for red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)

        # --- Detect Blue Bottle ---
        # Define the range for blue color in HSV
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Find contours (shapes) in the masks
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.process_contours(contours_red, "can", self.can_pub)
        self.process_contours(contours_blue, "bottle", self.bottle_pub)

    def process_contours(self, contours, object_type, publisher):
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the center of the contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                pixel_x = int(M["m10"] / M["m00"])
                pixel_y = int(M["m01"] / M["m00"])

                # This is a simple conversion from pixel coordinates to world coordinates
                # based on our known camera setup. A real system would use a more complex calibration.
                world_x = 0.6 - (pixel_y - 400) * 0.0005
                world_y = 0.0 - (pixel_x - 400) * 0.0005
                world_z = 0.0 # On the ground

                # Create a PointStamped message
                pose_msg = PointStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "world" # The coordinates are in the world frame
                pose_msg.point.x = world_x
                pose_msg.point.y = world_y
                pose_msg.point.z = world_z

                # Publish the message
                publisher.publish(pose_msg)

if __name__ == '__main__':
    try:
        detector = ColorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass