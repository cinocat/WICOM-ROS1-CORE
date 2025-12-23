#!/usr/bin/env python3
import os
import csv
import time

import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge
import cv2

class DatasetLogger:
    def __init__(self):
        # Topics and params
        self.image_topic = rospy.get_param("~image_topic", "/main_camera/image_raw")

        # Backward compatibility: if only cmd_topics is provided, treat it as Twist (unstamped)
        cmd_topics_legacy = rospy.get_param("~cmd_topics", "")
        cmd_twist_default = cmd_topics_legacy if cmd_topics_legacy else "/mavros/setpoint_velocity/cmd_vel_unstamped,/cmd_vel"

        cmd_twist_topics_str = rospy.get_param("~cmd_twist_topics", cmd_twist_default)
        cmd_twist_stamped_topics_str = rospy.get_param("~cmd_twist_stamped_topics", "/mavros/setpoint_velocity/cmd_vel")

        self.cmd_twist_topics = [t.strip() for t in cmd_twist_topics_str.split(",") if t.strip()]
        self.cmd_twist_stamped_topics = [t.strip() for t in cmd_twist_stamped_topics_str.split(",") if t.strip()]

        self.out_dir = rospy.get_param("~out_dir", os.path.join(os.path.expanduser("~"), "road_dataset"))
        self.save_stride = int(rospy.get_param("~save_stride", 1))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.auto_start = bool(rospy.get_param("~auto_start", True))

        # State
        self.bridge = CvBridge()
        self.enabled = self.auto_start
        self.frame_count = 0
        self.last_cmd_time = None
        self.last_yaw = None

        # Prepare output dirs/files
        images_dir = os.path.join(self.out_dir, "images")
        os.makedirs(images_dir, exist_ok=True)
        os.makedirs(self.out_dir, exist_ok=True)

        self.csv_path = os.path.join(self.out_dir, "data.csv")
        self.csv_file = open(self.csv_path, "a", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        if os.stat(self.csv_path).st_size == 0:
            self.csv_writer.writerow(["file_path", "yaw"])

        # Subscribers
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=10)

        # Subscribe ONLY to the correct type per topic list to avoid ROS type mismatch errors
        for t in self.cmd_twist_topics:
            rospy.Subscriber(t, Twist, self.handle_twist, queue_size=50)
        for t in self.cmd_twist_stamped_topics:
            rospy.Subscriber(t, TwistStamped, self.handle_twist_stamped, queue_size=50)

        # Service
        self.toggle_srv = rospy.Service("~toggle", SetBool, self.toggle_cb)

        rospy.loginfo(f"[dataset_logger] Logging to: {self.out_dir}")
        rospy.loginfo(f"[dataset_logger] Image: {self.image_topic}")
        rospy.loginfo(f"[dataset_logger] Cmd (Twist): {self.cmd_twist_topics}")
        rospy.loginfo(f"[dataset_logger] Cmd (TwistStamped): {self.cmd_twist_stamped_topics}")
        rospy.loginfo(f"[dataset_logger] Enabled: {self.enabled}, save_stride: {self.save_stride}, cmd_timeout: {self.cmd_timeout}s")

    def handle_twist(self, tw: Twist):
        # Unstamped command: use ROS receive time as the command time
        self.last_yaw = float(tw.angular.z)
        self.last_cmd_time = rospy.Time.now()

    def handle_twist_stamped(self, ts: TwistStamped):
        self.last_yaw = float(ts.twist.angular.z)
        # Prefer the header stamp when valid
        if ts.header.stamp and ts.header.stamp.to_sec() > 0:
            self.last_cmd_time = ts.header.stamp
        else:
            self.last_cmd_time = rospy.Time.now()

    def toggle_cb(self, req: SetBoolRequest):
        self.enabled = bool(req.data)
        msg = "enabled" if self.enabled else "disabled"
        rospy.loginfo(f"[dataset_logger] Toggled: {msg}")
        return SetBoolResponse(success=True, message=msg)

    def image_cb(self, img_msg: Image):
        if not self.enabled:
            return

        self.frame_count += 1
        if self.frame_count % self.save_stride != 0:
            return

        # Require a recent command to pair with the image
        now = rospy.Time.now()
        if self.last_cmd_time is None or self.last_yaw is None:
            return
        if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            return

        # Convert image
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn(f"[dataset_logger] cv_bridge error: {e}")
            return

        # Use image timestamp for filename (stable and monotonic per camera)
        ts = img_msg.header.stamp.to_sec() if img_msg.header.stamp and img_msg.header.stamp.to_sec() > 0 else time.time()
        fname = f"{int(ts * 1e6)}.jpg"
        rel_path = os.path.join("images", fname)
        abs_path = os.path.join(self.out_dir, rel_path)

        # Write
        try:
            cv2.imwrite(abs_path, cv_img)
            self.csv_writer.writerow([rel_path, f"{self.last_yaw:.6f}"])
            self.csv_file.flush()
        except Exception as e:
            rospy.logwarn(f"[dataset_logger] write error: {e}")

    def close(self):
        try:
            self.csv_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("dataset_logger")
    node = DatasetLogger()
    rospy.on_shutdown(node.close)
    rospy.spin()