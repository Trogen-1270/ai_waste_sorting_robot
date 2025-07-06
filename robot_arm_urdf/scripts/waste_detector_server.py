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

        # --- Calibration Constants ---
        self.x0_world, self.y0_world, self.z0_world = 0.0, -0.693, 0.05
        self.x_pxl_ref, self.y_pxl_ref = 320, 240
        self.pxl_per_meter = 1250.0

        # --- HSV Mask for TOP of red object ---
        self.hsv_ranges = {
            'red_can': (np.array([0, 150, 100]), np.array([10, 255, 255]))
        }

        self.image_sub = rospy.Subscriber("/waste_camera/image_raw", Image, self.image_callback)
        
        rospy.loginfo("Perception Server: Waiting for first image...")
        while self.latest_image is None:
            rospy.sleep(0.1)

        self.service = rospy.Service('get_target_pose', GetTargetPose, self.handle_get_target_pose)

        rospy.loginfo("✅ Perception Service Server is ready.")
        rospy.spin()
        
        cv2.destroyAllWindows()

    def image_callback(self, msg):
        self.latest_image = msg

    def handle_get_target_pose(self, req):
        rospy.loginfo(f"Request received for: {req.label}")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            debug_image = cv_image.copy()

            lower_hsv, upper_hsv = self.hsv_ranges[req.label]
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                return GetTargetPoseResponse(pose=Pose(), success=False)

            cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) < 50:
                return GetTargetPoseResponse(pose=Pose(), success=False)

            rect = cv2.minAreaRect(cnt)
            (x_pxl, y_pxl), _, _ = rect

            # --- Coordinate Conversion ---
            dx_pxl = x_pxl - self.x_pxl_ref
            dy_pxl = y_pxl - self.y_pxl_ref
            dx_world = -dy_pxl / self.pxl_per_meter
            dy_world = -dx_pxl / self.pxl_per_meter

            final_x = self.x0_world + dx_world
            final_y = self.y0_world + dy_world
            final_z = self.z0_world
            
            # ===================================================================
            # ADDED: Draw debugging information on the image
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(debug_image, [box], 0, (0, 255, 0), 2) # Green box
            cv2.circle(debug_image, (int(x_pxl), int(y_pxl)), 5, (0, 0, 255), -1) # Red circle
            text = f"X: {final_x:.3f} Y: {final_y:.3f}"
            cv2.putText(debug_image, text, (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            # ===================================================================
            
            cv2.imshow("Debug Feed", debug_image)
            cv2.imshow("Detection Mask", mask)
            cv2.waitKey(1)
            
            # --- Prepare and return the response ---
            response = GetTargetPoseResponse()
            response.pose.position.x = final_x
            response.pose.position.y = final_y
            response.pose.position.z = final_z
            response.pose.orientation.w = 1.0
            response.success = True
            rospy.loginfo(f"Found object. Sending coordinates: X={final_x:.3f}, Y={final_y:.3f}, Z={final_z:.3f}")
            return response

        except Exception as e:
            rospy.logerr(f"Error during detection: {e}")
            return GetTargetPoseResponse(pose=Pose(), success=False)

if __name__ == '__main__':
    WasteDetectorServer()