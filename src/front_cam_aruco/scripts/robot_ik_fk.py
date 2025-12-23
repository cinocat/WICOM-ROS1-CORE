#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

class DroneArmController:
    def __init__(self, dry_run=False):
        # --- THÔNG SỐ VẬT LÝ ---
        self.L0 = 9.0
        self.L1 = 6.0
        self.L2 = 5.5
        self.L3 = 5.5
        
        # OFFSET CƠ KHÍ (giữ nguyên nếu robot lắp đúng)
        self.OFFSET_J3 = 20.0
        self.OFFSET_J4 = 10.0               
        
        # [QUAN TRỌNG] Tắt hoàn toàn tilt trong IK vì Matrix T đã lo rồi
        self.tilt_deg = 0.0  
        
        # [FIX Z] Nếu vẫn chọc thấp/cao, chỉnh nhẹ số này (cm)
        # Gợi ý: Bắt đầu từ 0.0, nếu thấp thì tăng lên 2.0
        self.Z_MANUAL_OFFSET = 0.3  
        
        # Giới hạn servo
        self.SOFT_LIMITS = {
            'base': (5.0, 175.0),
            'shoulder': (10.0, 170.0),
            'elbow': (10.0, 170.0),
            'wrist': (5.0, 175.0) 
        }
        
        self.SCALE_BASE = 1.5
        self.SCALE_ARM = 1.0
        self.SERVO_DEADBAND = 1.0 
        
        self.last_applied_servos = [90.0, 90.0, 90.0, 90.0]
        self.current_servos = [90.0, 90.0, 90.0, 90.0] 
        self.current_xyz = [0.0, 10.0, -self.L0]
        
        self.kit = None
        self.dry_run = dry_run
        
        if not dry_run:
            try:
                import board, busio
                from adafruit_servokit import ServoKit
                import adafruit_tca9548a
                i2c_bus = busio.I2C(board.SCL, board.SDA)
                tca = adafruit_tca9548a.TCA9548A(i2c_bus, address=0x70)
                self.kit = ServoKit(channels=16, i2c=tca[2], address=0x40)
                for i in range(4): self.kit.servo[i].set_pulse_width_range(600, 2400)
                print("Hardware connected – ServoKit ready")
            except Exception as e:
                print(f"Hardware Error: {e}")
                self.dry_run = True

    def _solve_geometry(self, r_target, z_target, wrist_angle_global):
        """Giải hình học phẳng"""
        wrist_rad = math.radians(wrist_angle_global)
        
        # Tính vị trí khớp Wrist
        r_wrist = r_target - self.L3 * math.cos(wrist_rad)
        z_wrist = z_target - self.L3 * math.sin(wrist_rad)
        
        d_sq = r_wrist**2 + z_wrist**2
        dist = math.sqrt(d_sq)
        
        # [AUTO-CLAMP] Kéo wrist về vùng làm việc nếu target hơi xa
        max_reach_2link = self.L1 + self.L2 - 0.05 
        if dist > max_reach_2link:
            scale = max_reach_2link / dist
            r_wrist *= scale
            z_wrist *= scale
            d_sq = r_wrist**2 + z_wrist**2
        
        min_reach = abs(self.L1 - self.L2) + 0.5
        if dist < min_reach: return None

        denom = 2 * self.L1 * self.L2
        cos_t2 = (d_sq - self.L1**2 - self.L2**2) / denom
        cos_t2 = max(-1.0, min(1.0, cos_t2))
            
        theta2 = -math.acos(cos_t2) # Elbow up/down
        
        k1 = self.L1 + self.L2 * math.cos(theta2)
        k2 = self.L2 * math.sin(theta2)
        theta1 = math.atan2(z_wrist, r_wrist) - math.atan2(k2, k1)
        
        return theta1, theta2

    def _calculate_servos(self, x, y, z, try_angle):
        # 1. Base (Không xoay trục tọa độ nữa)
        theta0 = math.atan2(x, y)
        sv_base = 90.0 + math.degrees(theta0) * self.SCALE_BASE 
        
        # 2. Arm Geometry
        r_total = math.sqrt(x**2 + y**2)
        z_rel = z + self.L0 
        
        # [CLAMP TOÀN CỤC]
        FULL_REACH = self.L1 + self.L2 + self.L3 - 0.2
        dist_total = math.sqrt(r_total**2 + z_rel**2)
        if dist_total > FULL_REACH:
            scale = FULL_REACH / dist_total
            r_total *= scale
            z_rel *= scale
        
        geom_result = self._solve_geometry(r_total, z_rel, try_angle)
        if not geom_result: return None
        
        t1, t2 = geom_result
        
        # 3. Servo angles
        sv_shoulder = 90.0 - math.degrees(t1) * self.SCALE_ARM
        sv_elbow = (180.0 - self.OFFSET_J3) + math.degrees(t2) * self.SCALE_ARM 
        
        theta1_real = math.radians((90.0 - sv_shoulder) / self.SCALE_ARM)
        theta2_real = math.radians((sv_elbow - 180.0 + self.OFFSET_J3) / self.SCALE_ARM)
        sum_theta123 = theta1_real + theta2_real
        
        theta_wrist_needed = math.radians(try_angle) - sum_theta123
        sv_wrist = 90.0 + self.OFFSET_J4 + math.degrees(theta_wrist_needed) * self.SCALE_ARM
        
        return [sv_base, sv_shoulder, sv_elbow, sv_wrist]

    def _ik_math(self, x, y, z):
        # Cộng offset Z thủ công (nếu cần tinh chỉnh cuối cùng)
        z_in = z + self.Z_MANUAL_OFFSET
        
        # Chiến thuật: Ưu tiên thẳng (0) -> Chúi nhẹ (-10) -> Chúi mạnh (-30) -> Ngóc lên (10)
        # Bỏ 165 độ vì nó hướng ngược ra sau
        candidate_angles = [0.0, -10.0, -20.0, -35.0, 10.0]
        
        for angle in candidate_angles:
            angles = self._calculate_servos(x, y, z_in, angle)
            if angles:
                if self._check_limits(angles, verbose=False):
                    return angles
        return None

    def get_current_position_fk(self):
        s = self.current_servos
        theta0 = math.radians((s[0] - 90.0) / self.SCALE_BASE)
        theta1 = math.radians((90.0 - s[1]) / self.SCALE_ARM)
        theta2 = math.radians((s[2] - 180.0 + self.OFFSET_J3) / self.SCALE_ARM)
        deg_j4_local = (90.0 + self.OFFSET_J4 - s[3]) 
        theta3_local = math.radians(deg_j4_local / self.SCALE_ARM)
        
        global_angle = theta1 + theta2 + theta3_local
        r_arm = (self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2) + self.L3 * math.cos(global_angle))
        z_arm = (self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2) + self.L3 * math.sin(global_angle))
        
        z_final = z_arm - self.L0
        x_res = r_arm * math.sin(theta0)
        y_res = r_arm * math.cos(theta0)
        
        self.current_xyz = [x_res, y_res, z_final]
        return x_res, y_res, z_final

    def _check_limits(self, angles, verbose=True):
        keys = ['base', 'shoulder', 'elbow', 'wrist']
        for i, val in enumerate(angles):
            low, high = self.SOFT_LIMITS[keys[i]]
            if val < low or val > high:
                return False
        return True

    def _apply_servos(self):
        if not self.dry_run and self.kit:
            moved = False
            for i in range(4):
                target = max(0, min(180, self.current_servos[i]))
                if abs(target - self.last_applied_servos[i]) >= self.SERVO_DEADBAND:
                    self.kit.servo[i].angle = target
                    self.last_applied_servos[i] = target
                    moved = True
            return moved
        return False

    def move_to(self, x, y, z):
        angles = self._ik_math(x, y, z) 
        if angles:
            self.current_servos = list(angles)
            return self._apply_servos()
        return False
        
    def park(self):
        # Tư thế park gọn gàng
        self.current_servos = [90.0, 45.0, 150.0, 90.0] 
        self._apply_servos()
