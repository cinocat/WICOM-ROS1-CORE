#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RL Target Visualizer Node
Subscribes to RL target (in robot base frame) and visualizes it on camera feed.
Uses TF to transform between robot base frame and camera frame.
Draws a 1cm radius circle at the target position.
"""
import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, PointStamped
import math

class RLTargetVisualizer:
    def __init__(self):
        rospy.init_node('rl_target_visualizer', anonymous=False)
        
        # Parameters
        self.camera_frame = rospy.get_param('~camera_frame', 'front_cam/usb_cam_link')
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'base_link')
        self.image_topic = rospy.get_param('~image_topic', '/front_cam/usb_cam/robust_debug')
        self.target_topic = rospy.get_param('~target_topic', '/front_cam/rl_target')
        self.fallback_fov_deg = rospy.get_param('~fallback_fov_deg', 60.0)
        self.target_radius_m = rospy.get_param('~target_radius_m', 0.01)  # 1cm radius
        
        # State
        self.bridge = CvBridge()
        self.latest_image = None
        self.rl_target = None  # In robot base frame
        self.camera_position = None  # Camera position in board frame
        self.K = None
        self.D = None
        
        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribers
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.target_sub = rospy.Subscriber(self.target_topic, Vector3, self.target_cb, queue_size=1)
        self.camera_pos_sub = rospy.Subscriber('/front_cam/robust_aruco_node/camera_position', 
                                                Vector3, self.camera_pos_cb, queue_size=1)
        self.info_sub = rospy.Subscriber('/usb_camera/camera_info', CameraInfo, self.info_cb, queue_size=1)
        
        # Publisher
        self.viz_pub = rospy.Publisher('/front_cam/rl_visualization', Image, queue_size=1)
        
        rospy.loginfo("[RL Target Visualizer] Started")
        rospy.loginfo(f"  Robot frame: {self.robot_base_frame}")
        rospy.loginfo(f"  Camera frame: {self.camera_frame}")
        rospy.loginfo(f"  Target radius: {self.target_radius_m*100:.1f}cm")
    
    def info_cb(self, msg):
        """Store camera intrinsics"""
        K_temp = np.array(msg.K).reshape(3, 3)
        if K_temp[0, 0] > 0:
            self.K = K_temp
            self.D = np.array(msg.D) if msg.D else np.zeros(5)
    
    def get_fallback_intrinsics(self, width, height):
        """Generate fallback camera matrix if calibration unavailable"""
        fov_rad = self.fallback_fov_deg * math.pi / 180.0
        fx = fy = (width / 2.0) / math.tan(fov_rad / 2.0)
        cx = width / 2.0
        cy = height / 2.0
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0,  0,  1]], dtype=np.float32)
        D = np.zeros(5, dtype=np.float32)
        return K, D
    
    def target_cb(self, msg):
        """Receive RL target in robot base frame"""
        self.rl_target = msg
        rospy.loginfo_throttle(2.0, f"[RL Target] Received: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")
    
    def camera_pos_cb(self, msg):
        """Receive camera position from robust_aruco_node"""
        self.camera_position = msg
    
    def image_cb(self, msg):
        """Process and visualize"""
        self.latest_image = msg
        self.process()
    
    def transform_target_to_camera_frame(self):
        """
        Transform RL target from robot base frame to camera frame.
        Returns: (x, y, z) in camera frame, or None if transform fails
        """
        if self.rl_target is None:
            return None
        
        try:
            # Create PointStamped in robot base frame
            target_point = PointStamped()
            target_point.header.frame_id = self.robot_base_frame
            target_point.header.stamp = rospy.Time(0)  # Use latest available transform
            target_point.point.x = self.rl_target.x
            target_point.point.y = self.rl_target.y
            target_point.point.z = self.rl_target.z
            
            # Transform to camera frame
            transformed = self.tf_buffer.transform(target_point, self.camera_frame, timeout=rospy.Duration(0.1))
            
            return np.array([transformed.point.x, transformed.point.y, transformed.point.z])
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"[RL Target Visualizer] TF transform failed: {e}")
            return None
    
    def draw_circle_3d(self, cv_img, center_3d, radius_m, K, D, color, thickness=2, num_points=32):
        """
        Draw a circle in 3D space (parallel to camera image plane) at the target position.
        
        Args:
            cv_img: OpenCV image to draw on
            center_3d: 3D center position in camera frame (x, y, z)
            radius_m: Radius in meters (real-world scale)
            K: Camera intrinsic matrix
            D: Distortion coefficients
            color: BGR color tuple
            thickness: Line thickness
            num_points: Number of points to approximate circle
        """
        # Generate circle points in 3D (in camera frame, parallel to image plane)
        # The circle is in the XY plane at distance Z from camera
        angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)
        
        # Create circle points around the center
        circle_points_3d = []
        for angle in angles:
            # Offset in camera X and Y directions (parallel to image plane)
            dx = radius_m * np.cos(angle)
            dy = radius_m * np.sin(angle)
            
            point_3d = np.array([
                center_3d[0] + dx,
                center_3d[1] + dy,
                center_3d[2]
            ])
            circle_points_3d.append(point_3d)
        
        circle_points_3d = np.array(circle_points_3d, dtype=np.float32)
        
        # Project 3D points to 2D
        rvec = np.zeros(3)
        tvec = np.zeros(3)
        pixels, _ = cv2.projectPoints(circle_points_3d, rvec, tvec, K, D)
        pixels = pixels.reshape(-1, 2).astype(np.int32)
        
        # Draw the circle as a polygon
        cv2.polylines(cv_img, [pixels], isClosed=True, color=color, thickness=thickness)
        
        # Draw center point
        center_pixel = pixels[0]  # Approximate center (could compute exact)
        center_3d_reshaped = center_3d.reshape(1, 3)
        center_px, _ = cv2.projectPoints(center_3d_reshaped, rvec, tvec, K, D)
        center_px = center_px.reshape(2).astype(np.int32)
        cv2.circle(cv_img, tuple(center_px), 3, color, -1)  # Filled center dot
        
        return center_px
    
    def process(self):
        """Main processing loop"""
        if self.latest_image is None:
            return
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, f"CV Bridge error: {e}")
            return
        
        h, w = cv_img.shape[:2]
        
        # Get camera intrinsics
        if self.K is None:
            K_use, D_use = self.get_fallback_intrinsics(w, h)
        else:
            K_use, D_use = self.K, self.D
        
        # Transform target to camera frame
        target_cam = self.transform_target_to_camera_frame()
        
        if target_cam is not None:
            try:
                # Draw 1cm radius circle at target position
                color = (0, 255, 255)  # Yellow
                center_px = self.draw_circle_3d(cv_img, target_cam, self.target_radius_m, 
                                                 K_use, D_use, color, thickness=2)
                
                # Check if center is within image bounds
                if 0 <= center_px[0] < w and 0 <= center_px[1] < h:
                    # Draw distance text
                    dist = np.linalg.norm(target_cam)
                    cv2.putText(cv_img, f"Target: {dist:.3f}m ({self.target_radius_m*100:.0f}cm radius)", 
                                (center_px[0] + 15, center_px[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5, color, 2, cv2.LINE_AA)
                else:
                    # Target is out of frame
                    cv2.putText(cv_img, "Target OUT OF FRAME", 
                                (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.7, (0, 0, 255), 2, cv2.LINE_AA)
            
            except cv2.error as e:
                rospy.logwarn_throttle(5.0, f"Projection error: {e}")
        else:
            # No target or transform failed
            cv2.putText(cv_img, "No RL Target", 
                        (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, (128, 128, 128), 2, cv2.LINE_AA)
        
        # Publish visualization
        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            out_msg.header = self.latest_image.header
            self.viz_pub.publish(out_msg)
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, f"CV Bridge publish error: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RLTargetVisualizer()
        node.run()
    except rospy.ROSInterruptException:
        pass
