#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from robot_arm_urdf.srv import GetTargetPose, GetTargetPoseResponse

class WasteDetectorServer:
    def __init__(self):
        rospy.init_node('waste_detector_server')

        self.bridge_object = CvBridge()
        self.latest_image = None

        # --- Calibration Constants ---
        # Updated with better guesses based on your feedback.
        self.x0_world = 0.0      # The can is at X=0.0
        self.y0_world = -0.6     # The can is at Y=-0.6
        self.z0_world = 0.06     # The height of the can's center
        self.pxl_per_meter = 1150.0 # This is our main tuning parameter now

        # --- HSV Mask for RED ---
        # This will specifically look for the red color of the can.
        self.lower_red = np.array([0, 120, 70])
        self.upper_red = np.array([10, 255, 255])
        
        # --- Publishers and Subscribers ---
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.image_sub = rospy.Subscriber("/waste_camera/image_raw", Image, self.camera_callback)
        
        self.service = rospy.Service('get_target_pose', GetTargetPose, self.handle_get_target_pose)
        
        rospy.loginfo("✅ Perception Service Server is ready.")
        rospy.spin()
        cv2.destroyAllWindows()

    def camera_callback(self, data):
        self.latest_image = data
        self.process_and_visualize()

    def process_and_visualize(self):
        if self.latest_image is None:
            return
            
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            
            # --- COLOR-BASED DETECTION LOGIC ---
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
            
            # Optional: Clean up the mask
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 100:
                    rect = cv2.minAreaRect(largest_contour)
                    (x_pxl, y_pxl), _, _ = rect

                    # --- Coordinate Conversion ---
                    img_center_x, img_center_y = cv_image.shape[1] / 2.0, cv_image.shape[0] / 2.0
                    dx_pxl = x_pxl - img_center_x
                    dy_pxl = y_pxl - img_center_y

                    dx_world = dy_pxl / self.pxl_per_meter
                    dy_world = -dx_pxl / self.pxl_per_meter

                    final_x = self.x0_world + dx_world
                    final_y = self.y0_world + dy_world
                    final_z = self.z0_world

                    # --- Publish RViz Marker ---
                    self.publish_cube_marker(final_x, final_y, final_z)

                    # --- Draw Debug Info ---
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
                    cv2.circle(cv_image, (int(x_pxl), int(y_pxl)), 7, (0, 0, 255), -1)

            cv2.imshow("Detection View", cv_image)
            cv2.imshow("Color Mask", mask)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def publish_cube_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "object_marker"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = 0.05, 0.05, 0.05
        marker.color.g, marker.color.a = 1.0, 0.7  # Green, semi-transparent
        self.marker_pub.publish(marker)

    def handle_get_target_pose(self, req):
        return GetTargetPoseResponse(success=True) # Dummy response for now
    
if __name__ == '__main__':
    WasteDetectorServer()