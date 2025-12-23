# PCA9685 Servo Control for Raspberry Pi 4 (ROS Clover-safe)

An toàn, không xung đột với flight stack Clover. Điều khiển 5 servo MG90S qua PCA9685, kèm các chương trình kiểm thử (sweep, sequence) và dịch vụ enable/disable/home.

## 1) Phần cứng và an toàn

- Không dùng 5V của Raspberry Pi để cấp servo. Dùng nguồn riêng 5–6V ≥3A (khuyến nghị 5A).
- GND nguồn servo phải nối chung với GND Raspberry Pi và GND PCA9685.
- Dây I2C:
  - PCA9685 VCC → 3.3V của Raspberry Pi (logic)
  - PCA9685 GND → GND Raspberry Pi
  - PCA9685 SDA → GPIO2 (Pin 3)
  - PCA9685 SCL → GPIO3 (Pin 5)
- V+ của PCA9685 → +5–6V từ nguồn servo riêng. Thêm tụ 1000µF/10V song song V+ và GND gần PCA9685, có cầu chì 3–5A nếu có.
- Mặc định địa chỉ PCA9685 là `0x40`. Nếu trùng với thiết bị I2C khác, đổi jumper A0–A5 để đổi địa chỉ (ví dụ `0x41`).

## 2) Chuẩn bị hệ thống trên Raspberry Pi 4 (ROS1/Clover)

Bật I2C và cài công cụ:

```bash
sudo apt-get update
sudo apt-get install -y i2c-tools python3-smbus python3-pip
# Khuyến nghị 100kHz nếu có nhiều thiết bị trên bus, 400kHz nếu ổn định:
sudo sed -i 's/^#\?dtparam=i2c_arm=.*/dtparam=i2c_arm=on\ndtparam=i2c_baudrate=100000/' /boot/firmware/config.txt
sudo usermod -aG i2c $USER
# Cài smbus2 (yêu cầu bởi node)
pip3 install --user -r requirements.txt
sudo reboot
```

Kiểm tra bus I2C sau reboot:

```bash
i2cdetect -y 1
# Phải thấy 40 (hoặc địa chỉ bạn cấu hình)
```

## 3) Cài đặt package

Nếu chưa có catkin workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Thêm package:
```bash
cd ~/catkin_ws/src
# Tạo thư mục pca9685_servo_control và đặt toàn bộ file từ phần Files vào đúng cấu trúc.
# Ví dụ:
# pca9685_servo_control/
# ├── CMakeLists.txt
# ├── package.xml
# ├── requirements.txt
# ├── config/{servos.yaml,demo_poses.yaml}
# ├── launch/{pca9685_servo.launch,demo_sweep.launch,demo_sequence.launch}
# ├── scripts/{pca9685_kill_all.py,demo_sweep.py,demo_sequence.py}
# ├── src/servo_controller_node.py
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 4) Chạy controller (an toàn, không ảnh hưởng flight stack)

Khởi chạy node điều khiển:
```bash
roslaunch pca9685_servo_control pca9685_servo.launch
```

- Node chạy với `nice -n 10` (ưu tiên CPU thấp).
- Tất cả topic/service được namespace `/pca9685_servo` để tránh va chạm Clover.

Dịch vụ và topic:
- Services:
  - `/pca9685_servo/enable` (std_srvs/Trigger)
  - `/pca9685_servo/disable` (std_srvs/Trigger)
  - `/pca9685_servo/home` (std_srvs/Trigger)
  - `/pca9685_servo/set_angle` (pca9685_servo_control/SetAngle)
- Topics:
  - `/pca9685_servo/joint_states` (sensor_msgs/JointState, radians)
  - `/pca9685_servo/command` (JointState đầu vào, tùy chọn)

Ví dụ gọi nhanh:
```bash
# Bật output và đưa về vị trí trung tính
rosservice call /pca9685_servo/enable
rosservice call /pca9685_servo/home
# Đặt kênh 0 về 90 độ
rosservice call /pca9685_servo/set_angle "channel: 0 angle_deg: 90.0"
# Xuất bản nhiều khớp cùng lúc (rad hoặc deg)
rostopic pub /pca9685_servo/command sensor_msgs/JointState "{name: ['base','shoulder'], position: [1.57, 0.7]}" -1
# Xem trạng thái
rostopic echo /pca9685_servo/joint_states
```

## 5) Chương trình chạy mẫu để kiểm thử

Sweep demo (quét từng kênh):
```bash
# Tự khởi động controller + chạy demo sweep
roslaunch pca9685_servo_control demo_sweep.launch
# Hoặc chạy riêng, tùy chọn tham số
rosrun pca9685_servo_control demo_sweep.py --ns /pca9685_servo --channels 0 1 2 3 4 --min 30 --max 150 --step 10 --dwell 0.25 --rounds 2
```

Sequence demo (chuỗi pose có tên):
```bash
# Tự khởi động controller + chạy demo sequence
roslaunch pca9685_servo_control demo_sequence.launch
# Hoặc chạy riêng với tham số
rosrun pca9685_servo_control demo_sequence.py --ns /pca9685_servo --poses_ns /pca9685_demo --repeat 2 --hold_scale 1.0
```

Emergency OFF:
```bash
rosnode kill /pca9685_servo_controller || true
rosrun pca9685_servo_control pca9685_kill_all.py --bus 1 --addr 0x40
```

## 6) Tránh xung đột I2C với Clover

- Địa chỉ: đảm bảo không trùng (PCA9685 mặc định 0x40). Nếu trùng, đổi A0..A5 để đổi địa chỉ (0x41..0x7F theo bảng datasheet).
- Tốc độ: nếu chung bus với nhiều thiết bị Clover, đặt `dtparam=i2c_baudrate=100000`. Chỉ tăng 400kHz khi chắc ổn định.
- Nếu vẫn xung đột/timing: tách bus bằng TCA9548A hoặc USB-I2C riêng và chỉnh `i2c_bus` trong `config/servos.yaml` tương ứng.

Kiểm tra nhanh:
```bash
i2cdetect -y 1   # thấy 0x40
rosnode list     # thấy /pca9685_servo_controller
rostopic list | grep pca9685_servo
```

## 7) Tích hợp an toàn với hệ thống bay

- Không sửa launch cốt lõi của Clover; dùng launch riêng và namespace `/pca9685_servo`.
- `enable_on_start: false` để tránh servo giật lúc boot; chỉ bật khi có lệnh hoặc demo.
- `shutdown_behavior: neutral` đưa về vị trí an toàn khi node dừng.
- CPU: node chạy với nice thấp, publish 10 Hz, I2C rất nhẹ → không ảnh hưởng MAVROS/FCU.

## 8) Hiệu chỉnh/cấu hình

- Cập nhật `config/servos.yaml`: `channels`, `limits_deg`, `pulse_us_min/max` nếu cần mở rộng/giới hạn hành trình. Điều chỉnh `neutral_deg` nếu 90° không đúng tâm thực tế.
- Với servo MG90S tím, khoảng 600–2400 µs là an toàn điển hình; tránh vượt để không kẹt cơ khí.

## 9) Xử lý lỗi thường gặp

- Không thấy 0x40 trong `i2cdetect`:
  - Kiểm tra dây SDA/SCL, GND chung, nguồn servo riêng, bật I2C, quyền nhóm i2c, địa chỉ chip.
- Lỗi `PermissionError: /dev/i2c-1`:
  - `sudo adduser $USER i2c` rồi reboot.
- `OSError: [Errno 121] Remote I/O error`:
  - Lỏng dây, không chung GND, địa chỉ sai, nhiễu nguồn. Thêm tụ 1000µF, giảm i2c_baudrate.
- Servo rung/giật:
  - Nguồn không đủ dòng; thêm tụ lớn; giảm tần số PWM về 50 Hz (mặc định); đặt `timeout_behavior=hold`.

## 10) Khởi động cùng hệ thống (tùy chọn)

Tạo service systemd chạy sau khi ROS master sẵn sàng, hoặc thêm vào launch tổng của bạn. Nên giữ riêng để không ảnh hưởng flight stack.

---

Với bộ mã và hướng dẫn trên, bạn có thể triển khai ngay trên Raspberry Pi 4 chạy Clover, kiểm thử an toàn bằng các demo có sẵn, và vận hành song song với các node ROS khác mà không phá vỡ functionality hiện có.# wicom_roboarm
# wicom_roboarm
