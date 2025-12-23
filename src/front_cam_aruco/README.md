# Front-Facing USB Camera ArUco Integration (Clover Drone)

## Mục tiêu
Tích hợp camera USB hướng trước vào hệ thống Clover để nhận diện ArUco trên mặt phẳng đứng, tính góc tới, yaw, pitch của bảng/marker và quan sát trực tiếp qua browser tương tự main camera.

## 1. Yêu cầu
- ROS Noetic
- Clover firmware & gói `aruco_detect` đã cài
- usb_cam driver (đã có trong Clover image hoặc cài: `sudo apt install ros-noetic-usb-cam`)
- web_video_server: `sudo apt install ros-noetic-web-video-server`
- rosbridge_server: `sudo apt install ros-noetic-rosbridge-server`
- OpenCV Python3: `sudo apt install python3-opencv`

## 2. Cài đặt
Sao chép vào catkin workspace:
```bash
cd ~/catkin_ws/src
git clone (hoặc copy thư mục) front_cam_aruco
cd ..
catkin_make
# hoặc: catkin build
source devel/setup.bash
```

## 3. Hiệu chỉnh Camera Intrinsics
Chạy công cụ chuẩn:
```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/front_cam/usb_cam/image_raw camera:=/front_cam/usb_cam
```
Sau khi có file `.yaml`, thay nội dung vào `config/front_cam_calibration.yaml`.

## 4. Launch
```bash
roslaunch front_cam_aruco front_cam_aruco.launch
```
Các node sẽ chạy trong namespace `front_cam`:
- `/front_cam/usb_cam/*`
- `/front_cam/aruco_detect`
- `/front_cam/calculate_front_angle/*`
- `/front_cam/aruco_front_overlay/*`

Static TF: `base_link -> front_cam/usb_cam_link` căn chỉnh để trục nhìn camera (Z_cam) trùng hướng tiến (+X của drone).

## 5. Topics chính
- Ảnh gốc rectified: `/front_cam/usb_cam/image_rect`
- Ảnh overlay (trục marker): `/front_cam/usb_cam/image_aruco`
- Góc tới: `/front_cam/calculate_front_angle/angle_of_incidence`
- Yaw marker: `/front_cam/calculate_front_angle/board_yaw`
- Pitch marker: `/front_cam/calculate_front_angle/board_pitch`
- Marker IDs thấy được: `/front_cam/aruco_front_overlay/visible_ids`
- TF frames: `aruco_marker_<id>`

## 6. Xem trên Browser
Truy cập:
```
http://<drone_ip>:8080/stream?topic=/front_cam/usb_cam/image_aruco
```
Hoặc dùng giao diện:
```
http://<drone_ip>/roswww/front_cam_aruco/index.html
```
Nếu không có roswww auto serve, mở file `www/index.html` manual (hoặc cài `roswww`):
```bash
sudo apt install ros-noetic-roswww
```

## 7. Điều chỉnh Transform
Nếu camera đặt lệch:
- Sửa `args="x y z yaw pitch roll base_link front_cam/usb_cam_link"` trong launch.
- Giữ yaw/pitch/roll như đã tính để bảo toàn hướng trục quang học.

## 8. Mở rộng
- Thêm service trả trạng thái marker.
- Kết hợp với controller để căn chỉnh drone giữ bảng ở góc thấp.
- Gộp nhiều marker ID cho một bảng → tính trung bình vector normal.

## 9. Kiểm thử nhanh
1. In marker ArUco (ID 0) dán lên bảng.
2. Đặt drone nhìn thẳng → `angle_of_incidence ≈ 0`.
3. Xoay bảng sang trái → `board_yaw` thay đổi dấu.
4. Nghiêng bảng lên/xuống → `board_pitch` dương khi ngửa lên (kiểm chứng; nếu ngược đổi dấu trong code).

## 10. Xử lý sự cố
- Không thấy TF: kiểm tra `publish_tf` trong `aruco_detect`.
- Ảnh overlay không đổi: xác nhận marker trong tầm nhìn và đủ sáng.
- Sai góc lớn: kiểm tra calibration và dùng ảnh rectify (`image_rect`).

## 11. Giải thích góc
- Incidence = acos(n_z) (với n_z là thành phần normal theo hướng nhìn). 0° khi mặt phẳng vuông góc với trục nhìn (marker đối diện), tăng khi nghiêng.
- Yaw Board = atan2(n_x, n_z)
- Pitch Board = atan2(-n_y, n_z)

## 12. License
MIT

Chúc bạn triển khai thành công!