#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import csv
import os
import math
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Vector3

# ==========================================
# CẤU HÌNH NGƯỜI DÙNG (USER CONFIG)
# ==========================================
# [QUAN TRỌNG] Đặt True để BẮT BUỘC dùng thông số K_user bên dưới
FORCE_USER_CALIB = True 

# Thông số Camera bạn đã Calib (Thay thế các con số này bằng của bạn)
K_user = np.array([
    [839.3449,    0.    , 288.5372],
    [   0.    , 839.4361, 236.4550],
    [   0.    ,    0.    ,    1.    ]
], dtype=np.float32)

D_user = np.array([[-0.22608, -0.02472, -0.00085, 0.00054, -0.31300]], dtype=np.float32)

# Cấu hình Marker
BOARD_SIZE_M = 0.120       
OFFSET = 0.048             
SAFE_ZONE = 0.070          
MARKER_SIZE = 0.020        # [LƯU Ý] Hãy chắc chắn marker thật là 2cm!
HALF_MARKER = MARKER_SIZE / 2
NUM_POINTS = 20

def get_marker_corners_3d(center_x, center_y, half_size):
    return np.array([
        [center_x - half_size, center_y + half_size, 0], 
        [center_x + half_size, center_y + half_size, 0], 
        [center_x + half_size, center_y - half_size, 0], 
        [center_x - half_size, center_y - half_size, 0]  
    ], dtype=np.float32)

BOARD_CONFIG_3D = {
    0: get_marker_corners_3d(-OFFSET,  OFFSET, HALF_MARKER), 
    1: get_marker_corners_3d( OFFSET,  OFFSET, HALF_MARKER), 
    2: get_marker_corners_3d( OFFSET, -OFFSET, HALF_MARKER), 
    3: get_marker_corners_3d(-OFFSET, -OFFSET, HALF_MARKER)  
}

class RobustPBVSNode:
    def __init__(self):
        rospy.init_node('robust_pbvs_node', anonymous=False)

        self.image_topic = rospy.get_param('~image_topic', '/usb_camera/image_raw')
        self.info_topic  = rospy.get_param('~camera_info_topic', '/usb_camera/camera_info')
        self.log_folder  = rospy.get_param('~log_folder', os.path.expanduser('~')) 
        self.show_gui    = rospy.get_param('~show_gui', False) 
        self.log_interval= rospy.get_param('~log_interval', 3.0) 

        self.pub_euler = rospy.Publisher('~board_euler', Vector3, queue_size=1) 
        self.pub_position = rospy.Publisher('~camera_position', Vector3, queue_size=1)
        self.pub_distances = rospy.Publisher('~spiral_distances', Float32MultiArray, queue_size=1)
        self.debug_pub = rospy.Publisher('/front_cam/usb_cam/robust_debug', Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.info_sub  = rospy.Subscriber(self.info_topic, CameraInfo, self.info_cb, queue_size=1)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.params = aruco.DetectorParameters_create()
        
        # Thiết lập ma trận
        self.K = None
        self.D = None
        self.has_valid_info = False

        if FORCE_USER_CALIB:
            rospy.logwarn("⚠️  CHẾ ĐỘ CƯỠNG CHẾ: Đang dùng Ma trận User Calib (Bỏ qua ROS Topic).")
            self.K = K_user
            self.D = D_user
            self.has_valid_info = True

        self.spiral_points_3d = self.generate_spiral_3d()
        self.last_log_time = 0
        self.frame_count = 0
        self.fps_start_time = rospy.Time.now().to_sec()
        self.fps = 0.0

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = os.path.join(self.log_folder, f"log_robust_ros_{timestamp_str}.csv")
        self.init_log_file()
        
        rospy.loginfo(f"[RobustPBVS] Started. User Matrix: {FORCE_USER_CALIB}")

    def init_log_file(self):
        try:
            with open(self.log_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                base_header = ["Timestamp", "FPS", "Markers", "X", "Y", "Z", "Roll", "Pitch", "Yaw"]
                points_header = [f"D{i}" for i in range(NUM_POINTS)]
                writer.writerow(base_header + points_header)
        except Exception as e:
            rospy.logerr(f"Cannot create log file: {e}")

    def generate_spiral_3d(self):
        t = np.linspace(0, 4 * np.pi, NUM_POINTS)
        r = np.linspace(0.05, 0.45, NUM_POINTS)
        u = 0.5 + r * np.cos(t)
        v = 0.5 + r * np.sin(t)
        points = []
        for i in range(NUM_POINTS):
            x_b = (u[i] - 0.5) * SAFE_ZONE
            y_b = (0.5 - v[i]) * SAFE_ZONE
            points.append([x_b, y_b, 0.0])
        return np.array(points, dtype=np.float32)

    def rotation_matrix_to_euler(self, R):
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([math.degrees(x), math.degrees(y), math.degrees(z)])

    def info_cb(self, msg):
        # [FIX] Nếu đang bật cưỡng chế, ta CHẶN không cho cập nhật từ Topic
        if FORCE_USER_CALIB:
            return 

        if not self.has_valid_info:
            K_temp = np.array(msg.K).reshape(3, 3)
            if K_temp[0, 0] > 0:
                self.K = K_temp
                self.D = np.array(msg.D)
                self.has_valid_info = True
                rospy.loginfo("[Robust] Đã nhận Camera Info từ ROS Topic.")

    def image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Luôn dùng self.K hiện tại (đã được force hoặc nhận từ topic)
        current_K = self.K if self.has_valid_info else K_user
        current_D = self.D if self.has_valid_info else D_user

        self.frame_count += 1
        now = rospy.Time.now().to_sec()
        if now - self.fps_start_time >= 1.0:
            self.fps = self.frame_count / (now - self.fps_start_time)
            self.frame_count = 0
            self.fps_start_time = now

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)

        found_board = False
        rvec, tvec = None, None
        markers_found_count = 0
        image_points_collected = []
        object_points_collected = []

        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):
                curr_id = ids[i][0]
                if curr_id in BOARD_CONFIG_3D:
                    markers_found_count += 1
                    curr_corners_2d = corners[i][0]
                    curr_corners_3d = BOARD_CONFIG_3D[curr_id]
                    for pt in curr_corners_2d: image_points_collected.append(pt)
                    for pt in curr_corners_3d: object_points_collected.append(pt)

            if len(image_points_collected) >= 4:
                img_pts = np.array(image_points_collected, dtype=np.float32)
                obj_pts = np.array(object_points_collected, dtype=np.float32)
                
                success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, current_K, current_D)
                if success:
                    found_board = True
                    rmat, _ = cv2.Rodrigues(rvec)
                    euler = self.rotation_matrix_to_euler(rmat) 
                    
                    T_mat = np.eye(4); T_mat[:3,:3]=rmat; T_mat[:3,3]=tvec.flatten()
                    distances = []
                    for pt_board in self.spiral_points_3d:
                        pt_homo = np.append(pt_board, 1.0)
                        pt_cam = T_mat @ pt_homo
                        distances.append(np.linalg.norm(pt_cam[:3]))
                    
                    self.pub_euler.publish(Vector3(x=euler[0], y=euler[1], z=euler[2]))
                    self.pub_position.publish(Vector3(x=tvec[0][0], y=tvec[1][0], z=tvec[2][0]))
                    self.pub_distances.publish(Float32MultiArray(data=distances))

                    cv2.drawFrameAxes(cv_img, current_K, current_D, rvec, tvec, 0.05)
                    pixels, _ = cv2.projectPoints(self.spiral_points_3d, rvec, tvec, current_K, current_D)
                    pixels = pixels.reshape(-1, 2).astype(int)
                    for i in range(len(pixels) - 1):
                        cv2.line(cv_img, tuple(pixels[i]), tuple(pixels[i+1]), (0, 255, 0), 2)
                    
                    cv2.putText(cv_img, f"Y:{euler[2]:.1f} P:{euler[1]:.1f} R:{euler[0]:.1f}", 
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(cv_img, f"DistZ: {tvec[2][0]:.3f}m", 
                                (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

        if (now - self.last_log_time) >= self.log_interval:
            self.last_log_time = now

        if found_board:
            status_text = f"Robust: LOCKED ({markers_found_count}) FPS:{self.fps:.1f}"
            color = (0, 255, 0)
        else:
            status_text = f"Robust: SEARCHING... FPS:{self.fps:.1f}"
            color = (0, 0, 255)
            
        cv2.putText(cv_img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            out_msg.header = msg.header
            self.debug_pub.publish(out_msg)
        except Exception:
            pass

        if self.show_gui:
            cv2.imshow("Robust PBVS ROS", cv_img)
            cv2.waitKey(1)

    def run(self):
        rospy.spin()
        if self.show_gui:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    node = RobustPBVSNode()
    node.run()