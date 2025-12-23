#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, math, tf2_ros, numpy as np
from std_msgs import msg as std_msgs

class FrontCamArucoAngleNode:
    def __init__(self):
        rospy.init_node('calculate_front_angle', anonymous=False)
        self.marker_id     = rospy.get_param('~marker_id', 0)
        self.camera_frame  = rospy.get_param('~camera_frame', 'front_cam/usb_cam_link')
        self.publish_rate  = rospy.get_param('~publish_rate', 20)
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.marker_frame = f"aruco_marker_{self.marker_id}"
        self.pub_incidence = rospy.Publisher('~angle_of_incidence', std_msgs.Float32, queue_size=10)
        self.pub_yaw       = rospy.Publisher('~board_yaw', std_msgs.Float32, queue_size=10)
        self.pub_pitch     = rospy.Publisher('~board_pitch', std_msgs.Float32, queue_size=10)
        self.pub_vector    = rospy.Publisher('~plane_normal_cam', std_msgs.Float32MultiArray, queue_size=10)
        self.last_ok_pub = rospy.Time(0)
        rospy.loginfo(f"[calculate_front_angle] Tracking marker frame: {self.marker_frame}")

    @staticmethod
    def quat_to_rot_matrix(qx,qy,qz,qw):
        R = np.zeros((3,3))
        R[0,0]=1-2*(qy**2+qz**2); R[0,1]=2*(qx*qy-qz*qw); R[0,2]=2*(qx*qz+qy*qw)
        R[1,0]=2*(qx*qy+qz*qw); R[1,1]=1-2*(qx**2+qz**2); R[1,2]=2*(qy*qz-qx*qw)
        R[2,0]=2*(qx*qz-qy*qw); R[2,1]=2*(qy*qz+qx*qw); R[2,2]=1-2*(qx**2+qy**2)
        return R

    def compute_angles(self, nvec):
        n = nvec/(np.linalg.norm(nvec)+1e-9)
        incidence = math.degrees(math.acos(max(min(n[2],1.0),-1.0)))
        yaw_board = math.degrees(math.atan2(n[0], n[2]))
        pitch_board = math.degrees(math.atan2(-n[1], n[2]))
        return incidence, yaw_board, pitch_board

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            try:
                trans = self.buffer.lookup_transform(self.camera_frame,
                                                     self.marker_frame,
                                                     rospy.Time(0),
                                                     timeout=rospy.Duration(0.3))
                q=trans.transform.rotation
                R=self.quat_to_rot_matrix(q.x,q.y,q.z,q.w)
                normal_cam = R.dot(np.array([0,0,1]))
                inc,yaw,pitch = self.compute_angles(normal_cam)
                self.pub_incidence.publish(std_msgs.Float32(data=inc))
                self.pub_yaw.publish(std_msgs.Float32(data=yaw))
                self.pub_pitch.publish(std_msgs.Float32(data=pitch))
                self.pub_vector.publish(std_msgs.Float32MultiArray(data=list(normal_cam)))
                self.last_ok_pub = rospy.Time.now()
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                rospy.logwarn_throttle(5.0, f"[calculate_front_angle] TF unavailable for {self.marker_frame} (waiting for detect)")
            rate.sleep()

if __name__ == "__main__":
    FrontCamArucoAngleNode().spin()