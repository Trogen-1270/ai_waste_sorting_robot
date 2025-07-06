#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from robot_arm_urdf.srv import GetTargetPose, GetTargetPoseResponse

class WasteDetectorServer:
    def __init__(self):
        rospy.init_node('waste_detector_server')

        self.bridge = CvBridge()
        self.latest_image = None

        # --- Calibration Constants (from tutorial) ---
        self.x0_world, self.y0_world, self.z0_world = 1.0, 0.0, 0.5
        self.x_pxl_ref, self.y_pxl_ref = 319, 240
        self.pxl_per_meter = 1350.0

        # --- HSV Mask for TOP of red object ---
        self.hsv_ranges = {
            'red_can': (np.array([0, 20, 160]), np.array([5, 120, 226]))
        }

        # --- ROS Subscribers and Services (Updated Logic) ---
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        # --- Wait for the first image before starting the service ---
        rospy.loginfo("Perception Server: Waiting for first image...")
        while self.latest_image is None:
            rospy.sleep(0.1)  # Wait for a short moment

        # Now that we have an image, we can start the service
        self.service = rospy.Service('get_target_pose', GetTargetPose, self.handle_get_target_pose)

        rospy.loginfo("✅ Perception Service Server is ready.")
        rospy.spin()

    def image_callback(self, msg):
        self.latest_image = msg

    def handle_get_target_pose(self, req):
        rospy.loginfo(f"Request received for: {req.label}")

        if self.latest_image is None:
            # This check is now redundant but kept for safety
            rospy.logwarn("Server has not received an image yet.")
            return GetTargetPoseResponse(pose=Pose(), success=False)
        
        if req.label not in self.hsv_ranges:
            rospy.logerr(f"Label '{req.label}' is not a recognized object.")
            return GetTargetPoseResponse(pose=Pose(), success=False)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_hsv, upper_hsv = self.hsv_ranges[req.label]
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                rospy.logwarn(f"No contours found for label '{req.label}'.")
                return GetTargetPoseResponse(pose=Pose(), success=False)

            cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) < 50:
                rospy.logwarn("Detected contour is too small, likely noise.")
                return GetTargetPoseResponse(pose=Pose(), success=False)

            rect = cv2.minAreaRect(cnt)
            (x_pxl, y_pxl), _, _ = rect

            dx_pxl = x_pxl - self.x_pxl_ref
            dy_pxl = y_pxl - self.y_pxl_ref
            dx_world = dy_pxl / self.pxl_per_meter
            dy_world = -dx_pxl / self.pxl_per_meter

            final_x = self.x0_world + dx_world
            final_y = self.y0_world + dy_world
            final_z = self.z0_world

            response = GetTargetPoseResponse()
            response.pose.position.x = final_x
            response.pose.position.y = final_y
            response.pose.position.z = final_z
            response.pose.orientation.w = 1.0
            response.success = True
            rospy.loginfo("Found object. Sending coordinates.")
            return response

        except Exception as e:
            rospy.logerr(f"Error during detection: {e}")
            return GetTargetPoseResponse(pose=Pose(), success=False)

if __name__ == '__main__':
    WasteDetectorServer()