#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker # Import the Marker message

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector_node', anonymous=True)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/waste_camera/image_raw", Image, self.image_callback)

        self.can_pub = rospy.Publisher("/detected_can_pose", PointStamped, queue_size=10)
        self.bottle_pub = rospy.Publisher("/detected_bottle_pose", PointStamped, queue_size=10)

        # --- NEW: Publisher for the visual marker ---
        self.marker_pub = rospy.Publisher("/detection_marker", Marker, queue_size=10)

        rospy.loginfo("Color Detector node started.")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define HSV color range for Red
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)

        # Define HSV color range for Blue
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Process the largest red contour if found
        if contours_red:
            largest_red_contour = max(contours_red, key=cv2.contourArea)
            self.process_contour(largest_red_contour, "can", self.can_pub)

        # Process the largest blue contour if found
        if contours_blue:
            largest_blue_contour = max(contours_blue, key=cv2.contourArea)
            self.process_contour(largest_blue_contour, "bottle", self.bottle_pub)


    def process_contour(self, contour, object_type, publisher):
        M = cv2.moments(contour)
        if M["m00"] != 0:
            # Calculate pixel coordinates
            pixel_x = int(M["m10"] / M["m00"])
            pixel_y = int(M["m01"] / M["m00"])

            # This is the simple pixel-to-world conversion that needs tuning
            world_x = 0.7 - (pixel_y - 400) * 0.001
            world_y = 0.0 - (pixel_x - 400) * 0.001
            world_z = 0.0

            # Publish the PointStamped message
            pose_msg = PointStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            pose_msg.point.x = world_x
            pose_msg.point.y = world_y
            pose_msg.point.z = world_z
            publisher.publish(pose_msg)

            # --- NEW: Publish a visual marker for debugging ---
            self.publish_marker(world_x, world_y, world_z, object_type)

    def publish_marker(self, x, y, z, object_type):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "detections"

        if object_type == "can":
            marker.id = 0 # Unique ID for the can marker
            marker.color.r = 0.0
            marker.color.g = 1.0 # Green
            marker.color.b = 0.0
        else: # bottle
            marker.id = 1 # Unique ID for the bottle marker
            marker.color.r = 1.0 # Yellow
            marker.color.g = 1.0
            marker.color.b = 0.0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 0.8 # Alpha (transparency)
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        detector = ColorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass