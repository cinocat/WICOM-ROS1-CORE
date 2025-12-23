#!/usr/bin/env python3
import os
import threading
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

try:
    import onnxruntime as ort
except Exception as e:
    rospy.logerr("Failed to import onnxruntime: %s", e)
    raise

class RoadFollowingNode:
    def __init__(self):
        self.model_path = rospy.get_param("~model_path", os.path.expanduser("~/models/model_best.onnx"))
        self.image_topic = rospy.get_param("~image_topic", "/main_camera/image_raw")
        self.cmd_pub_topic = rospy.get_param("~cmd_pub_topic", "/mavros/setpoint_velocity/cmd_vel_unstamped")
        self.fwd_speed = float(rospy.get_param("~fwd_speed", 0.2))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 1.8))
        self.alpha = float(rospy.get_param("~smooth_alpha", 0.8))
        self.camera_rotate_deg = float(rospy.get_param("~camera_rotate_deg", 0.0))
        self.min_edge_density = float(rospy.get_param("~min_edge_density", 0.02))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.enable_confidence_stop = bool(rospy.get_param("~enable_confidence_stop", False))
        self.cmd_msg_type = rospy.get_param("~cmd_msg_type", "auto")  # auto|twist|stamped

        # - TwistStamped: đặt frame_id theo ~cmd_frame (mặc định "base_link" => đi theo mũi drone)
        # - Twist (unstamped): nếu ~force_body_fwd_for_unstamped=True, tự transform body->ENU theo yaw hiện tại
        self.cmd_frame = rospy.get_param("~cmd_frame", "base_link")
        self.force_body_fwd_for_unstamped = bool(rospy.get_param("~force_body_fwd_for_unstamped", True))

        # Altitude hold
        self.hold_altitude = bool(rospy.get_param("~hold_altitude", True))
        self.alt_mode = rospy.get_param("~alt_mode", "capture")  # capture|fixed
        self.alt_target_z = float(rospy.get_param("~alt_target_z", 1.0))
        self.kp_z = float(rospy.get_param("~kp_z", 1.0))
        self.kd_z = float(rospy.get_param("~kd_z", 0.6))
        self.max_z_vel = float(rospy.get_param("~max_z_vel", 0.5))
        self.invert_z_sign = bool(rospy.get_param("~invert_z_sign", False))

        self.bridge = CvBridge()
        self._img_lock = threading.Lock()
        self._last_bgr = None
        self._last_stamp = rospy.Time(0)

        self._z_lock = threading.Lock()
        self._z = None
        self._z_prev = None
        self._z_t = None
        self._z_t_prev = None

        # NEW: lưu yaw hiện tại để transform body->map khi cần
        self._yaw_lock = threading.Lock()
        self._yaw = 0.0
        self._yaw_valid = False

        providers = ["CPUExecutionProvider"]
        try:
            self.session = ort.InferenceSession(self.model_path, providers=providers)
        except Exception as e:
            rospy.logerr("Failed to load ONNX model at %s: %s", self.model_path, e)
            raise

        self.input_name = self.session.get_inputs()[0].name
        in_shape = self.session.get_inputs()[0].shape
        self.in_h, self.in_w = 90, 160
        if len(in_shape) == 4:
            h, w = in_shape[2], in_shape[3]
            if isinstance(h, int) and h > 0: self.in_h = h
            if isinstance(w, int) and w > 0: self.in_w = w

        if self.cmd_msg_type == "twist":
            self.use_twist = True
        elif self.cmd_msg_type == "stamped":
            self.use_twist = False
        else:
            self.use_twist = self.cmd_pub_topic.endswith("_unstamped")

        if self.use_twist:
            self.cmd_pub = rospy.Publisher(self.cmd_pub_topic, Twist, queue_size=10)
            pub_type = "Twist"
        else:
            self.cmd_pub = rospy.Publisher(self.cmd_pub_topic, TwistStamped, queue_size=10)
            pub_type = "TwistStamped"

        self.enabled = True
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2**24)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=10)
        self.toggle_srv = rospy.Service("~toggle", SetBool, self.toggle_cb)

        self._yaw_smooth = 0.0
        self._init_smooth = True

        try:
            provs = ort.get_available_providers()
        except Exception:
            provs = []
        rospy.loginfo("[path_following] Loaded model: %s", self.model_path)
        rospy.loginfo("[path_following] Providers: %s", provs)
        rospy.loginfo("[path_following] Input size: %dx%d", self.in_w, self.in_h)
        rospy.loginfo("[path_following] Publishing: %s (%s) at %.1f Hz", self.cmd_pub_topic, pub_type, self.publish_rate_hz)
        rospy.loginfo("[path_following] cmd_frame=%s, force_body_fwd_for_unstamped=%s",
                      self.cmd_frame, self.force_body_fwd_for_unstamped)

    def toggle_cb(self, req: SetBoolRequest):
        self.enabled = bool(req.data)
        if self.enabled and self.hold_altitude and self.alt_mode.lower() == "capture":
            with self._z_lock:
                if self._z is not None:
                    self.alt_target_z = float(self._z)
                    rospy.loginfo("[path_following] Captured alt_target_z=%.3f m", self.alt_target_z)
                else:
                    rospy.logwarn("[path_following] Cannot capture altitude target: z is None")
        rospy.loginfo("[path_following] Toggled: %s", "enabled" if self.enabled else "disabled")
        return SetBoolResponse(success=True, message="enabled" if self.enabled else "disabled")

    def image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[path_following] cv_bridge error: %s", e)
            return
        with self._img_lock:
            self._last_bgr = bgr
            self._last_stamp = msg.header.stamp if msg.header.stamp and msg.header.stamp.to_sec() > 0 else rospy.Time.now()

    @staticmethod
    def _quat_to_yaw(x, y, z, w) -> float:
        # yaw around Z (ENU)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny_cosp, cosy_cosp))

    def pose_cb(self, msg: PoseStamped):
        z = float(msg.pose.position.z)
        t = msg.header.stamp if msg.header.stamp and msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        with self._z_lock:
            self._z_prev = self._z
            self._z_t_prev = self._z_t
            self._z = z
            self._z_t = t
        # NEW: cập nhật yaw để transform body->map khi cần
        q = msg.pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        with self._yaw_lock:
            self._yaw = yaw
            self._yaw_valid = True

    @staticmethod
    def _rotate_bgr(bgr, deg):
        if bgr is None: return None
        d = int(deg) % 360
        if d == 0: return bgr
        if d == 90: return cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE)
        if d == 180: return cv2.rotate(bgr, cv2.ROTATE_180)
        if d == 270: return cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        h, w = bgr.shape[:2]
        M = cv2.getRotationMatrix2D((w/2, h/2), d, 1.0)
        return cv2.warpAffine(bgr, M, (w, h), flags=cv2.INTER_LINEAR)

    def _preprocess(self, bgr):
        bgr = self._rotate_bgr(bgr, self.camera_rotate_deg)
        resized = cv2.resize(bgr, (self.in_w, self.in_h), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        chw = np.transpose(rgb, (2, 0, 1))
        blob = np.expand_dims(chw, axis=0).astype(np.float32).copy()
        return blob, resized

    def _infer_yaw(self, blob: np.ndarray) -> float:
        y = self.session.run(None, {self.input_name: blob})[0]
        return float(y.reshape(-1)[0])

    def _edge_confidence_ok(self, resized_bgr) -> bool:
        if not self.enable_confidence_stop:
            return True
        gray = cv2.cvtColor(resized_bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        density = float(np.count_nonzero(edges)) / float(edges.size)
        return density >= self.min_edge_density

    def _smooth_yaw(self, y: float) -> float:
        if self._init_smooth:
            self._yaw_smooth = y
            self._init_smooth = False
        else:
            self._yaw_smooth = self.alpha * y + (1.0 - self.alpha) * self._yaw_smooth
        return self._yaw_smooth

    @staticmethod
    def _clamp(v, lo, hi): return max(lo, min(hi, v))

    def _compute_vz_hold(self) -> float:
        if not self.hold_altitude:
            return 0.0
        with self._z_lock:
            if self._z is None or self._z_t is None or self._z_prev is None or self._z_t_prev is None:
                return 0.0
            z, t = self._z, self._z_t.to_sec()
            z_prev, t_prev = self._z_prev, self._z_t_prev.to_sec()
        dt = t - t_prev if t_prev is not None else 0.0
        dz = (z - z_prev) / dt if dt and dt > 1e-3 else 0.0
        err = self.alt_target_z - z
        vz = self.kp_z * err - self.kd_z * dz
        if self.invert_z_sign: vz = -vz
        return self._clamp(vz, -self.max_z_vel, self.max_z_vel)

    def _publish_cmd(self, vx: float, yaw_rate: float, vz: float, stamp: rospy.Time):
        # vx là "forward theo thân drone"
        if self.use_twist:
            # Unstamped: mặc định MAVROS hiểu theo ENU/map; để đi theo thân drone, ta transform body->map
            msg = Twist()
            vx_e = vx
            vy_e = 0.0
            if self.force_body_fwd_for_unstamped:
                with self._yaw_lock:
                    yaw = self._yaw
                    valid = self._yaw_valid
                if valid:
                    c = np.cos(yaw); s = np.sin(yaw)
                    vx_e = c * vx - s * 0.0
                    vy_e = s * vx + c * 0.0
                else:
                    rospy.logwarn_throttle(1.0, "[path_following] Yaw not ready; publishing vx in ENU X")
            msg.linear.x = float(vx_e)
            msg.linear.y = float(vy_e)
            msg.linear.z = float(vz)
            msg.angular.x = 0.0; msg.angular.y = 0.0; msg.angular.z = float(yaw_rate)
            self.cmd_pub.publish(msg)
        else:
            # Stamped: đặt frame_id để MAVROS hiểu đúng khung (base_link => theo thân)
            ts = TwistStamped()
            ts.header.stamp = stamp if stamp and stamp.to_sec() > 0 else rospy.Time.now()
            ts.header.frame_id = self.cmd_frame  # "base_link" khuyến nghị
            ts.twist.linear.x = float(vx)
            ts.twist.linear.y = 0.0
            ts.twist.linear.z = float(vz)
            ts.twist.angular.x = 0.0; ts.twist.angular.y = 0.0; ts.twist.angular.z = float(yaw_rate)
            self.cmd_pub.publish(ts)

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            with self._img_lock:
                bgr = None if self._last_bgr is None else self._last_bgr.copy()
                stamp = self._last_stamp

            vz_cmd = self._compute_vz_hold() if self.enabled else 0.0

            if bgr is not None:
                blob, resized = self._preprocess(bgr)
                if self.enabled:
                    try:
                        yaw_pred = self._infer_yaw(blob)
                    except Exception as e:
                        rospy.logwarn_throttle(2.0, "[path_following] Inference error: %s", e)
                        yaw_pred = 0.0
                    yaw_sm = self._smooth_yaw(yaw_pred)
                    yaw_out = self._clamp(yaw_sm, -self.max_yaw_rate, self.max_yaw_rate)
                    vx = self.fwd_speed
                    if self.enable_confidence_stop and not self._edge_confidence_ok(resized):
                        # Giữ nguyên yaw_out nhưng KHÔNG set vx=0 nếu bạn muốn "no-stop"; ở đây vẫn theo tham số gốc
                        vx = 0.0
                    self._publish_cmd(vx, yaw_out, vz_cmd, stamp)
                else:
                    self._publish_cmd(0.0, 0.0, 0.0, rospy.Time.now())
            else:
                self._publish_cmd(0.0, 0.0, 0.0, rospy.Time.now())
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("path_following")
    node = RoadFollowingNode()
    node.spin()
