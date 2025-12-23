#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VERIFY MATRIX ACCURACY - V2.1 (ADAPTIVE & CONTINUOUS)
"""

import rospy
import numpy as np
import sys
import tty, termios
import time
import math
from geometry_msgs.msg import Vector3
from robot_ik_fk import DroneArmController

class MatrixVerifierV2:
    def __init__(self):
        rospy.init_node('matrix_verifier_v2', anonymous=False)
        
        try:
            self.T = np.load("T_cam_to_base_THEORETICAL.npy")
            print("✅ Đã load Ma trận Transform.")
        except Exception:
            # Fallback default nếu không có file
            self.T = np.eye(4)
            print("⚠️ Không tìm thấy file npy, dùng ma trận đơn vị!")

        self.bot = DroneArmController(dry_run=False)
        self.last_angles = self.bot.current_servos.copy()

        self.latest_cam_pos = None
        self.has_vision = False
        self.continuous_mode = False # Chế độ tự động bám
        
        rospy.Subscriber('/robust_pbvs_node/camera_position', Vector3, self.vision_cb)
        
        self.SOFT_LIMITS = self.bot.SOFT_LIMITS # Dùng chung limit với bot
        self.MAX_DELTA_ANGLE = 15.0  # Tăng nhẹ delta cho phép

    def vision_cb(self, msg):
        self.latest_cam_pos = np.array([msg.x, msg.y, msg.z, 1.0])
        self.has_vision = True

    def get_key(self):
        # Non-blocking key check cho chế độ continuous
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def setup_terminal(self):
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)

    def is_safe_angles(self, angles):
        delta = np.abs(np.array(angles) - np.array(self.last_angles))
        if np.max(delta) > self.MAX_DELTA_ANGLE:
            # Trong continuous mode, chỉ cảnh báo chứ không chặn (để mượt hơn)
            if not self.continuous_mode: 
                print(f"⚠️ SAFETY: Delta góc lớn {np.max(delta):.1f}°")
                return False
        return True

    def run(self):
        self.setup_terminal()
        print("\n" + "="*60)
        print("   VERIFIER V2.1 - ADAPTIVE WRIST IK")
        print("="*60)
        print(" [ENTER] : Move once (Step mode)")
        print(" [C]     : Toggle Continuous Mode (Auto follow)")
        print(" [ESC]   : Park & Exit")
        print("="*60)

        try:
            rate = rospy.Rate(10) # 10Hz
            while not rospy.is_shutdown():
                if self.has_vision and self.latest_cam_pos is not None:
                    # Tính toán Target
                    p_base_homo = self.T @ self.latest_cam_pos
                    x_target, y_target, z_target = p_base_homo[:3] * 100
                    
                    cur_x, cur_y, cur_z = self.bot.get_current_position_fk()
                    error = math.sqrt((x_target - cur_x)**2 + (y_target - cur_y)**2 + (z_target - cur_z)**2)
                    
                    status_str = (f"\r👁 Err:{error:4.1f}cm | "
                                  f"Tgt: [{x_target:4.1f}, {y_target:4.1f}, {z_target:4.1f}] | "
                                  f"Mode: {'AUTO 🔄' if self.continuous_mode else 'STEP ⏹️ '} ")
                    sys.stdout.write(status_str)
                    sys.stdout.flush()

                    # Logic điều khiển
                    should_move = False
                    key = self.get_key()
                    
                    if key == '\n': # ENTER
                        should_move = True
                        self.continuous_mode = False 
                    elif key == 'c':
                        self.continuous_mode = not self.continuous_mode
                        print(f"\n>>> Switched to {'CONTINUOUS' if self.continuous_mode else 'STEP'} mode")
                    elif key == '\x1b': # ESC
                        break
                    
                    if self.continuous_mode:
                        should_move = True

                    if should_move:
                        angles = self.bot._ik_math(x_target, y_target, z_target)
                        if angles:
                            # Bypass safety check cứng trong mode auto để mượt
                            # ServoKit sẽ tự lọc deadband
                            self.bot.current_servos = list(angles)
                            self.bot._apply_servos()
                            self.last_angles = angles
                            
                rate.sleep()

        except Exception as e:
            print(e)
        finally:
            print("\n🛑 Parking...")
            self.bot.park()
            self.restore_terminal()

if __name__ == "__main__":
    verifier = MatrixVerifierV2()
    verifier.run()