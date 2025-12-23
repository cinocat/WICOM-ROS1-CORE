#!/usr/bin/env python3
import math
import time
import threading

import rospy
from sensor_msgs.msg import Range, JointState
from std_srvs.srv import Trigger, TriggerResponse
from wicom_roboarm.srv import SetAngle, SetAngleResponse

from smbus2 import SMBus

import board
import busio
import adafruit_tca9548a
import adafruit_vl53l0x
import adafruit_vl53l1x

# ================== Helpers parse ==================
def _parse_i2c_addr(val, default):
    if val is None:
        return default
    if isinstance(val, int):
        return val
    if isinstance(val, str):
        s = val.strip()
        try:
            return int(s, 0)
        except ValueError:
            rospy.logwarn("Not parse address I2C from '%s', use default 0x%02X", s, default)
            return default
    rospy.logwarn("Address type I2C not support (%s), use default 0x%02X", type(val), default)
    return default

def _parse_int(val, default):
    if val is None:
        return default
    if isinstance(val, int):
        return val
    if isinstance(val, str):
        s = val.strip()
        try:
            return int(s, 0)
        except ValueError:
            rospy.logwarn("Not parse int from '%s', use default %d", s, default)
            return default
    try:
        return int(val)
    except Exception:
        rospy.logwarn("Not force %s to int, use default %d", str(val), default)
        return default

# ================== PCA9685 LOW-LEVEL ==================
MODE1      = 0x00
MODE2      = 0x01
PRESCALE   = 0xFE
LED0_ON_L  = 0x06
LED0_ON_H  = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
ALL_LED_ON_L  = 0xFA
ALL_LED_ON_H  = 0xFB
ALL_LED_OFF_L = 0xFC
ALL_LED_OFF_H = 0xFD

RESTART = 0x80
SLEEP   = 0x10
ALLCALL = 0x01
OUTDRV  = 0x04

class PCA9685:
    def __init__(self, bus: SMBus, address: int, oscillator_hz: int = 25_000_000,
                 select_mux_fn=None, mux_channel=None):
        self._bus = bus
        self._addr = address
        self._osc = oscillator_hz
        self._select_mux = select_mux_fn
        self._mux_channel = mux_channel
        self._init_device()

    def _sel(self):
        if self._select_mux and self._mux_channel is not None:
            self._select_mux(self._mux_channel)

    def _write8(self, reg, val):
        self._sel()
        self._bus.write_byte_data(self._addr, reg, val & 0xFF)

    def _read8(self, reg):
        self._sel()
        return self._bus.read_byte_data(self._addr, reg)

    def _init_device(self):
        self._write8(MODE2, OUTDRV)
        self._write8(MODE1, ALLCALL)
        time.sleep(0.005)
        mode1 = self._read8(MODE1) & ~SLEEP
        self._write8(MODE1, mode1)
        time.sleep(0.005)

    def set_pwm_freq(self, freq_hz: float):
        prescaleval = float(self._osc) / (4096.0 * float(freq_hz)) - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self._read8(MODE1)
        newmode = (oldmode & 0x7F) | SLEEP
        self._write8(MODE1, newmode)
        self._write8(PRESCALE, prescale)
        self._write8(MODE1, oldmode)
        time.sleep(0.005)
        self._write8(MODE1, oldmode | RESTART)

    def set_pwm(self, channel: int, on: int, off: int):
        base = LED0_ON_L + 4 * channel
        for attempt in range(3):
            try:
                self._write8(base + 0, on & 0xFF)
                self._write8(base + 1, (on >> 8) & 0x0F)
                self._write8(base + 2, off & 0xFF)
                self._write8(base + 3, (off >> 8) & 0x0F)
                return
            except OSError as e:
                if attempt == 2:
                    raise
                time.sleep(0.001)
                self._sel()

    def set_off(self, channel: int):
        base = LED0_ON_L + 4 * channel
        self._write8(base + 0, 0x00)
        self._write8(base + 1, 0x00)
        self._write8(base + 2, 0x00)
        self._write8(base + 3, 0x10)

    def set_all_off(self):
        self._write8(ALL_LED_ON_L, 0)
        self._write8(ALL_LED_ON_H, 0)
        self._write8(ALL_LED_OFF_L, 0)
        self._write8(ALL_LED_OFF_H, 0x10)

# ================== UNIFIED NODE ==================
class UnifiedRoboArmNode:
    def __init__(self):
        rospy.init_node("wicom_roboarm_unified", anonymous=False)

        # -------- Parameters (servo) ----------
        self.busnum          = _parse_int(rospy.get_param("~i2c_bus", 1), 1)
        self.mux_address     = _parse_i2c_addr(rospy.get_param("~mux_address", 0x70), 0x70)
        self.servo_mux_chan  = _parse_int(rospy.get_param("~mux_channel", 2), 2)
        self.pca_address     = _parse_i2c_addr(rospy.get_param("~i2c_address", 0x40), 0x40)
        self.oscillator_hz   = _parse_int(rospy.get_param("~oscillator_hz", 25_000_000), 25_000_000)
        self.pwm_freq        = float(rospy.get_param("~pwm_frequency_hz", 50.0))
        self.enable_on_start = bool(rospy.get_param("~enable_on_start", False))
        self.use_mux         = bool(rospy.get_param("~use_mux", True))  # always True

        self.pulse_us_min    = float(rospy.get_param("~pulse_us_min", 600.0))
        self.pulse_us_max    = float(rospy.get_param("~pulse_us_max", 2400.0))
        self.neutral_deg     = float(rospy.get_param("~neutral_deg", 30.0))
        # New: per-joint neutral degrees (optional). Falls back to neutral_deg if not provided.
        self.neutral_deg_map = rospy.get_param("~neutral_deg_per_joint", {})

        self.joint_names     = rospy.get_param("~joint_names")
        self.channels        = rospy.get_param("~channels")
        self.limits_deg      = rospy.get_param("~limits_deg", {})
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 1.0))
        self.timeout_behavior    = rospy.get_param("~timeout_behavior", "hold")
        self.shutdown_behavior   = rospy.get_param("~shutdown_behavior", "neutral")

        if (not self.joint_names) or (not self.channels) or len(self.joint_names) != len(self.channels):
            rospy.logfatal("joint_names and channels must same length.")
            raise RuntimeError("Invalid joint configuration")

        self.num_joints = len(self.joint_names)
        self.name_to_idx = {n: i for i, n in enumerate(self.joint_names)}
        self.channel_by_idx = {i: ch for i, ch in enumerate(self.channels)}
        # per-joint neutral list
        self.neutral_deg_by_idx = [
            float(self.neutral_deg_map.get(name, self.neutral_deg))
            for name in self.joint_names
        ]
        self.current_deg = list(self.neutral_deg_by_idx)
        self.last_cmd_time = [rospy.get_time()] * self.num_joints
        self.enabled = self.enable_on_start

        # -------- Parameters (VL53 sensors) ----------
        self.vl53_rate_hz   = float(rospy.get_param("~vl53_publish_rate_hz", 15.0))
        self.ch_short       = _parse_int(rospy.get_param("~channel_short", 0), 0)
        self.ch_long        = _parse_int(rospy.get_param("~channel_long", 1), 1)
        self.frame_short    = rospy.get_param("~frame_id_short", "vl53_short")
        self.frame_long     = rospy.get_param("~frame_id_long", "vl53_long")
        self.vl53_max_short = float(rospy.get_param("~max_range_short", 2.0))
        self.vl53_max_long  = float(rospy.get_param("~max_range_long", 4.0))

        # -------- I2C & Lock ----------
        self.lock = threading.Lock()
        self.bus_smbus = SMBus(self.busnum)
        # busio for adafruit multiplexer
        try:
            self.bus_busio = busio.I2C(board.SCL, board.SDA)
        except Exception as e:
            rospy.logfatal("Not init busio I2C: %s", e)
            raise

        try:
            self.tca = adafruit_tca9548a.TCA9548A(self.bus_busio, address=self.mux_address)
        except Exception as e:
            rospy.logfatal("Not init TCA9548A @0x%02X: %s", self.mux_address, e)
            raise

        # -------- Multiplexer select function ----------
        def select_mux(channel: int, force=False):
            self.bus_smbus.write_byte(self.mux_address, 1 << channel)
            time.sleep(0.001)

        self._select_mux = select_mux

        # -------- Init PCA9685 ----------
        try:
            if not self.use_mux:
                rospy.logwarn("Unified node design use mux.")
            self.pca = PCA9685(self.bus_smbus,
                               self.pca_address,
                               oscillator_hz=self.oscillator_hz,
                               select_mux_fn=self._select_mux if self.use_mux else None,
                               mux_channel=self.servo_mux_chan if self.use_mux else None)
            self.pca.set_pwm_freq(self.pwm_freq)
        except Exception as e:
            rospy.logfatal("Error init PCA9685: %s", e)
            raise

        if self.enabled:
            self._apply_all(self.current_deg)
        else:
            self._apply_behavior_all("off")

        # -------- Init VL53 sensors ----------
        self.sensor_long = None
        self.sensor_short = None
        with self.lock:
            try:
                self._select_mux(self.ch_long)
                self.sensor_long = adafruit_vl53l1x.VL53L1X(self.tca[self.ch_long])
                self.sensor_long.start_ranging()
                rospy.loginfo("VL53L1X (long) init channel %d OK", self.ch_long)
            except Exception as e:
                rospy.logwarn("VL53L1X init error: %s", e)

            try:
                self._select_mux(self.ch_short)
                self.sensor_short = adafruit_vl53l0x.VL53L0X(self.tca[self.ch_short])
                rospy.loginfo("VL53L0X (short) init channel %d OK", self.ch_short)
            except Exception as e:
                rospy.logwarn("VL53L0X init error: %s", e)

        active = []
        if self.sensor_short: active.append(f"VL53L0X@ch{self.ch_short}")
        if self.sensor_long:  active.append(f"VL53L1X@ch{self.ch_long}")
        rospy.loginfo("Sensors active: %s", ", ".join(active) if active else "NONE")

        # -------- Publishers / Services --------
        self.pub_joint = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.pub_short = rospy.Publisher("/vl53/short_range", Range, queue_size=10)
        self.pub_long  = rospy.Publisher("/vl53/long_range",  Range, queue_size=10)

        self.srv_set_angle = rospy.Service("set_angle", SetAngle, self.handle_set_angle)
        self.srv_enable    = rospy.Service("enable",    Trigger, self.handle_enable)
        self.srv_disable   = rospy.Service("disable",   Trigger, self.handle_disable)
        self.srv_home      = rospy.Service("home",      Trigger, self.handle_home)

        # Timers
        self.pub_timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1.0)), self._publish_joint_state)
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.05), self._watchdog_tick)

        # Thread VL53
        self.vl53_thread = threading.Thread(target=self._vl53_loop, daemon=True)
        self.vl53_thread.start()

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("Unified RoboArm Node started: PCA9685 addr=0x%02X mux=0x%02X servo_mux_ch=%d pwm=%.1fHz enabled=%s",
                      self.pca_address, self.mux_address, self.servo_mux_chan,
                      self.pwm_freq, self.enabled)

    # ================== Servo helpers ==================
    def angle_to_count(self, angle_deg: float) -> int:
        span_us = self.pulse_us_max - self.pulse_us_min
        frac = max(0.0, min(1.0, angle_deg / 180.0))
        pulse_us = self.pulse_us_min + frac * span_us
        counts = int(round((pulse_us / 1e6) * self.pwm_freq * 4096.0))
        return max(0, min(4095, counts))

    def apply_joint(self, idx: int, angle_deg: float):
        ch = self.channel_by_idx[idx]
        counts = self.angle_to_count(angle_deg)
        with self.lock:
            if self.use_mux:
                self._select_mux(self.servo_mux_chan)
            self.pca.set_pwm(ch, 0, counts)

    def _move_to_neutral(self, idx: int):
        ch = self.channel_by_idx[idx]
        neutral = self.neutral_deg_by_idx[idx]
        counts = self.angle_to_count(neutral)
        with self.lock:
            if self.use_mux:
                self._select_mux(self.servo_mux_chan)
            self.pca.set_pwm(ch, 0, counts)
        self.current_deg[idx] = neutral

    def _turn_off(self, idx: int):
        ch = self.channel_by_idx[idx]
        with self.lock:
            if self.use_mux:
                self._select_mux(self.servo_mux_chan)
            self.pca.set_off(ch)

    def _apply_behavior_all(self, behavior: str):
        for idx in range(self.num_joints):
            if behavior == "hold":
                self.apply_joint(idx, self.current_deg[idx])
            elif behavior == "neutral":
                self._move_to_neutral(idx)
            elif behavior == "off":
                self._turn_off(idx)

    def _apply_all(self, deg_list):
        for idx, deg in enumerate(deg_list):
            self.apply_joint(idx, deg)

    # ================== Services ==================
    def handle_set_angle(self, req):
        try:
            idx = self.channels.index(req.channel)
        except ValueError:
            return SetAngleResponse(False, f"Channel {req.channel} not manager.")

        name = self.joint_names[idx]
        lim = self.limits_deg.get(name, {"min": 0.0, "max": 180.0})
        target = max(lim.get("min", 0.0), min(lim.get("max", 180.0), float(req.angle_deg)))

        if not self.enabled:
            rospy.logwarn("Auto enable outputs, received set_angle")
            self.enabled = True

        try:
            self.apply_joint(idx, target)
            self.current_deg[idx] = target
            self.last_cmd_time[idx] = rospy.get_time()
            return SetAngleResponse(True, f"Set {name} (ch {req.channel}) -> {target:.1f} deg")
        except Exception as e:
            rospy.logerr("I2C error set_angle: %s", e)
            return SetAngleResponse(False, f"I2C error: {e}")

    def handle_enable(self, _req):
        try:
            self._apply_all(self.current_deg)
            self.enabled = True
            return TriggerResponse(success=True, message="Outputs enabled")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def handle_disable(self, _req):
        try:
            self._apply_behavior_all("off")
            self.enabled = False
            return TriggerResponse(success=True, message="Outputs disabled (off)")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def handle_home(self, _req):
        try:
            for idx in range(self.num_joints):
                self._move_to_neutral(idx)
            self.enabled = True
            return TriggerResponse(success=True, message="All joints neutral")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    # ================== JointState publish & watchdog ==================
    def _publish_joint_state(self, _evt):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = list(self.joint_names)
        js.position = [math.radians(d) for d in self.current_deg]
        self.pub_joint.publish(js)

    def _watchdog_tick(self, _evt):
        if self.command_timeout_sec <= 0:
            return
        now = rospy.get_time()
        for idx in range(self.num_joints):
            if (now - self.last_cmd_time[idx]) > self.command_timeout_sec:
                if self.timeout_behavior == "hold":
                    pass
                elif self.timeout_behavior == "neutral":
                    self._move_to_neutral(idx)
                elif self.timeout_behavior == "off":
                    self._turn_off(idx)
                self.last_cmd_time[idx] = now

    # ================== VL53 reading loop ==================
    def _range_msg(self, frame, min_r, max_r, dist_m):
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.52
        msg.min_range = min_r
        msg.max_range = max_r
        msg.range = dist_m if dist_m is not None else float('nan')
        return msg

    def _read_short(self):
        if not self.sensor_short:
            return None
        try:
            mm = self.sensor_short.range
            if mm is None or mm <= 0:
                return None
            return mm / 1000.0
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"VL53L0X read error: {e}")
            return None

    def _read_long(self):
        if not self.sensor_long:
            return None
        try:
            if hasattr(self.sensor_long, "data_ready"):
                if not self.sensor_long.data_ready:
                    return None
            mm = self.sensor_long.distance
            if mm is None or mm <= 0:
                return None
            return mm / 100.0
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"VL53L1X read error: {e}")
            return None

    def _vl53_loop(self):
        rate = rospy.Rate(self.vl53_rate_hz)
        while not rospy.is_shutdown():
            with self.lock:
                if self.use_mux:
                    self._select_mux(self.ch_short)
                d_short = self._read_short()
                if self.use_mux:
                    self._select_mux(self.ch_long)
                d_long = self._read_long()
            self.pub_short.publish(self._range_msg(self.frame_short, 0.03, self.vl53_max_short, d_short))
            self.pub_long.publish(self._range_msg(self.frame_long, 0.03, self.vl53_max_long, d_long))
            rate.sleep()

    # ================== Shutdown ==================
    def _on_shutdown(self):
        rospy.logwarn("Shutdown: apply behavior %s for all servo", self.shutdown_behavior)
        try:
            self._apply_behavior_all(self.shutdown_behavior)
            time.sleep(0.01)
            with self.lock:
                if self.use_mux:
                    self._select_mux(self.servo_mux_chan)
                self.pca.set_all_off()
            self.bus_smbus.close()
        except Exception as e:
            rospy.logerr("Shutdown error: %s", e)

def main():
    try:
        UnifiedRoboArmNode()
        rospy.spin()
    except Exception as e:
        rospy.logfatal("Unified node failed: %s", e)

if __name__ == "__main__":
    main()