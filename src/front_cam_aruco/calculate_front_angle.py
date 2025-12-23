#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class FrontCamArucoAngleNode:
    def __init__(self):
        rospy.init_node('calculate_front_angle', anonymous=False)

        self.marker_id     = rospy.get_param('~marker_id', 0)
        self.camera_frame  = rospy.get_param('~camera_frame', 'front_cam/usb_cam_link')
        self.publish_rate  = rospy.get_param('~publish_rate', 10)

        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.listener = tf2_ros.TransformListener(self.buffer)

        # Frames ArUco thường: aruco_marker_<id>
        self.marker_frame = f"aruco_marker_{self.marker_id}"

        self.pub_incidence = rospy.Publisher('~angle_of_incidence', Float32, queue_size=10)
        self.pub_yaw       = rospy.Publisher('~board_yaw', Float32, queue_size=10)
        self.pub_pitch     = rospy.Publisher('~board_pitch', Float32, queue_size=10)
        self.pub_vector    = rospy.Publisher('~plane_normal_cam', Float32MultiArray, queue_size=10)

        rospy.loginfo(f"[calculate_front_angle] Tracking marker frame: {self.marker_frame}")

    @staticmethod
    def quat_to_rot_matrix(qx, qy, qz, qw):
        # Chuẩn ROS quaternions (x,y,z,w)
        # R = rotation từ marker -> camera frame nếu lấy transform(camera_frame, marker_frame)
        R = np.zeros((3,3))
        R[0,0] = 1 - 2*(qy**2 + qz**2)
        R[0,1] = 2*(qx*qy - qz*qw)
        R[0,2] = 2*(qx*qz + qy*qw)
        R[1,0] = 2*(qx*qy + qz*qw)
        R[1,1] = 1 - 2*(qx**2 + qz**2)
        R[1,2] = 2*(qy*qz - qx*qw)
        R[2,0] = 2*(qx*qz - qy*qw)
        R[2,1] = 2*(qy*qz + qx*qw)
        R[2,2] = 1 - 2*(qx**2 + qy**2)
        return R

    def compute_angles(self, normal_vec):
        # normal_vec: vector 3D (n_x, n_y, n_z) trong camera frame
        n = normal_vec / (np.linalg.norm(normal_vec) + 1e-9)

        # Optical axis camera: +Z (chuẩn frame camera)
        # Angle of incidence: 0 nếu n song song Z (mặt phẳng đối diện camera)
        incidence = math.degrees(math.acos(max(min(n[2],1.0), -1.0)))

        # Yaw bảng: xoay quanh trục dọc -> thay đổi thành phần n_x
        # Định nghĩa: yaw = atan2(n_x, n_z)
        yaw_board = math.degrees(math.atan2(n[0], n[2]))

        # Pitch bảng: ngửa/cúi -> ảnh hưởng n_y (Y xuống)
        # pitch = atan2(-n_y, n_z) để pitch dương khi mặt phẳng ngửa lên phía trên
        pitch_board = math.degrees(math.atan2(-n[1], n[2]))

        return incidence, yaw_board, pitch_board

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            try:
                # Lấy transform marker -> camera frame
                # lookup_transform(target, source, time)
                # target = camera_frame, source = marker_frame
                trans = self.buffer.lookup_transform(self.camera_frame,
                                                     self.marker_frame,
                                                     rospy.Time(0),
                                                     timeout=rospy.Duration(0.5))

                q = trans.transform.rotation
                R = self.quat_to_rot_matrix(q.x, q.y, q.z, q.w)

                # Normal mặt phẳng = trục +Z của marker trong camera frame
                normal_cam = R.dot(np.array([0,0,1]))

                incidence, yaw_board, pitch_board = self.compute_angles(normal_cam)

                # Publish
                self.pub_incidence.publish(incidence)
                self.pub_yaw.publish(yaw_board)
                self.pub_pitch.publish(pitch_board)

                arr = Float32MultiArray()
                arr.data = list(normal_cam)
                self.pub_vector.publish(arr)

                rospy.logdebug(f"Marker {self.marker_id} incidence={incidence:.2f} yaw={yaw_board:.2f} pitch={pitch_board:.2f}")

            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException) as e:
                rospy.logwarn_throttle(5.0, f"TF unavailable for {self.marker_frame}: {e}")

            rate.sleep()

if __name__ == "__main__":
    try:
        node = FrontCamArucoAngleNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass