#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, cv2, numpy as np, tf2_ros, math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TransformStamped

class FrontArucoDetect:
    def __init__(self):
        rospy.init_node('front_aruco_detect', anonymous=False)
        self.camera_frame       = rospy.get_param('~camera_frame', 'front_cam/usb_cam_link')
        self.image_topic        = rospy.get_param('~image_topic', '/usb_camera/image_raw')
        self.camera_info_topic  = rospy.get_param('~camera_info_topic', '/usb_camera/camera_info')
        self.marker_length      = rospy.get_param('~marker_length', 0.03)
        self.dict_name          = rospy.get_param('~aruco_dictionary', 'DICT_4X4_50')
        self.publish_rate       = rospy.get_param('~publish_rate', 20)
        self.debug_draw         = rospy.get_param('~debug_draw', True)
        self.min_detect_interval= rospy.get_param('~min_detect_interval', 0.0)
        self.fallback_fov_deg   = rospy.get_param('~fallback_fov_deg', 62.0)  # ~62° FOV cho webcam phổ biến

        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.K = None
        self.D = None
        self.last_detect_time = 0.0
        self.frame_count = 0
        self.detect_count = 0
        self.im_size = None  # (w,h)

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.info_sub  = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.info_cb, queue_size=1)

        self.img_pub     = rospy.Publisher('/front_cam/usb_cam/image_aruco', Image, queue_size=1)
        self.markers_pub = rospy.Publisher('~markers', Int32MultiArray, queue_size=10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.aruco_dict = self.get_dictionary(self.dict_name)
        self.parameters = cv2.aruco.DetectorParameters_create()

        rospy.loginfo(f"[front_aruco_detect] dict={self.dict_name} marker_length={self.marker_length}m image_topic={self.image_topic}")

    @staticmethod
    def get_dictionary(name):
        mapping = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
            'DICT_APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36h11
        }
        return cv2.aruco.getPredefinedDictionary(mapping.get(name, cv2.aruco.DICT_4X4_50))

    def image_cb(self, msg: Image):
        self.latest_image_msg = msg
        self.frame_count += 1
        # Lưu kích thước ảnh để tạo K fallback khi cần
        self.im_size = (msg.width, msg.height) if hasattr(msg, 'width') and msg.width > 0 else None

    def info_cb(self, msg: CameraInfo):
        # Nhận camera_info nếu có; có thể rỗng/0 khi chưa calibrate
        K = np.array(msg.K).reshape(3,3)
        self.D = np.array(msg.D) if msg.D else np.zeros(5)
        # Kiểm tra hợp lệ: fx, fy > 0
        if K[0,0] > 0 and K[1,1] > 0:
            self.K = K
            rospy.loginfo("[front_aruco_detect] Camera info received (valid).")
        else:
            self.K = None  # sẽ tạo fallback khi xử lý
            rospy.logwarn_throttle(10.0, "[front_aruco_detect] Camera info invalid/empty, will use fallback intrinsics.")

    def get_intrinsics(self, w, h):
        # Nếu đã có K hợp lệ thì dùng, otherwise tạo K fallback từ FOV
        if self.K is not None:
            return self.K, self.D if self.D is not None else np.zeros(5)
        fov = float(self.fallback_fov_deg) * math.pi / 180.0
        fx = fy = (w / 2.0) / math.tan(fov / 2.0)
        cx = w / 2.0
        cy = h / 2.0
        K_fb = np.array([[fx, 0, cx],
                         [0, fy, cy],
                         [0,  0,  1]], dtype=np.float64)
        D_fb = np.zeros(5, dtype=np.float64)
        rospy.logwarn_throttle(5.0, f"[front_aruco_detect] Using fallback K fx=fy={fx:.1f}, cx={cx:.1f}, cy={cy:.1f}")
        return K_fb, D_fb

    def publish_tf(self, marker_id, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        trace = np.trace(R)
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2
            qw = 0.25 * s
            qx = (R[2,1] - R[1,2]) / s
            qy = (R[0,2] - R[2,0]) / s
            qz = (R[1,0] - R[0,1]) / s
        else:
            if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
                s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
                qx = 0.25 * s
                qy = (R[0,1] + R[1,0]) / s
                qz = (R[0,2] + R[2,0]) / s
                qw = (R[2,1] - R[1,2]) / s
            elif (R[1,1] > R[2,2]):
                s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
                qx = (R[0,1] + R[1,0]) / s
                qy = 0.25 * s
                qz = (R[1,2] + R[2,1]) / s
                qw = (R[0,2] - R[2,0]) / s
            else:
                s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
                qx = (R[0,2] + R[2,0]) / s
                qy = (R[1,2] + R[2,1]) / s
                qz = 0.25 * s
                qw = (R[1,0] - R[0,1]) / s

        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = f"aruco_marker_{marker_id}"
        transform.transform.translation.x = float(tvec[0])
        transform.transform.translation.y = float(tvec[1])
        transform.transform.translation.z = float(tvec[2])
        transform.transform.rotation.x = float(qx)
        transform.transform.rotation.y = float(qy)
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(transform)

    def process(self):
        if self.latest_image_msg is None:
            return
        # Lấy intrinsics hợp lệ (K,D)
        w = self.latest_image_msg.width if hasattr(self.latest_image_msg, 'width') else None
        h = self.latest_image_msg.height if hasattr(self.latest_image_msg, 'height') else None
        if not w or not h:
            if self.im_size:
                w, h = self.im_size
            else:
                return
        K_use, D_use = self.get_intrinsics(w, h)

        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"CVBridge error: {e}")
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        detected_ids = []
        now = rospy.get_time()
        if ids is not None and len(ids) > 0 and (now - self.last_detect_time) >= self.min_detect_interval:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, K_use, D_use)
            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                self.publish_tf(marker_id, rvec, tvec)
                detected_ids.append(int(marker_id))
                if self.debug_draw:
                    cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)
                    cv2.drawFrameAxes(cv_img, K_use, D_use, rvec, tvec, self.marker_length*0.5)
            self.detect_count += len(detected_ids)
            self.last_detect_time = now

        self.markers_pub.publish(Int32MultiArray(data=detected_ids))

        if self.debug_draw:
            cv2.putText(cv_img, f"Frames:{self.frame_count} Dets:{self.detect_count}",
                        (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1, cv2.LINE_AA)
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            out_msg.header.stamp = rospy.Time.now()
            out_msg.header.frame_id = self.camera_frame
            self.img_pub.publish(out_msg)

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()

if __name__ == "__main__":
    FrontArucoDetect().spin()