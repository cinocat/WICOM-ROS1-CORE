# Clover Road-Following (End-to-End Regression)

Mục tiêu
- Drone nhận ảnh từ camera `/main_camera/image_raw`, mô hình CNN dự đoán trực tiếp yaw_rate (angular.z) để bám đường.
- Pipeline: thu dữ liệu -> train -> export ONNX -> inference ROS node -> kiểm thử mô phỏng/thực.

Cấu trúc
- scripts/
  - dataset_logger.py: Ghi ảnh + yaw từ /cmd_vel hoặc /mavros/setpoint_velocity/cmd_vel_unstamped vào dataset/images + data.csv
  - inference_node.py: Nạp model ONNX, đọc camera, dự đoán angular.z, xuất TwistStamped lên /mavros/setpoint_velocity/cmd_vel_unstamped, có EMA smoothing, confidence stop
- ai/
  - train.py: Train PyTorch (Small PilotNet), augmentation, TensorBoard, early stopping, export ONNX
- tools/
  - split_dataset.py: Chia train/val/test từ data.csv
  - rosbag_to_dataset.py: Trích rosbag -> dataset (ảnh + csv)
- launch/
  - dataset_logger.launch
  - inference.launch
- config/
  - inference.yaml: Tham số mặc định inference
- package.xml, CMakeLists.txt

Yêu cầu chủ đạo
- Input ảnh RGB 160x90
- Loss MSE, Adam lr=1e-3, weight_decay=1e-5
- Early stopping theo val loss
- TensorBoard theo dõi
- Xuất model ONNX
- Inference ≥ 10 Hz (khuyến nghị 20 Hz), có smoothing EMA và clamp max_yaw_rate
- Dừng tiến (linear.x=0) khi “không tự tin” (edge density thấp)

1) Thu thập dữ liệu (dataset_logger)
- Chạy launch:
  roslaunch wicom_path_following dataset_logger.launch out_dir:=/path/to/dataset
- Node đọc /main_camera/image_raw và lệnh từ:
  - /mavros/setpoint_velocity/cmd_vel_unstamped (TwistStamped) hoặc
  - /cmd_vel (Twist)
- Trên mỗi khung hình, nếu có lệnh gần nhất trong cửa sổ cmd_timeout (mặc định 0.5s), ghi:
  - images/<timestamp>.jpg
  - data.csv: file_path,yaw
- Có service toggle: /dataset_logger/toggle (std_srvs/SetBool) để bật/tắt ghi
- Tham số chính:
  - image_topic, cmd_topics, out_dir, save_stride, cmd_timeout

2) Xử lý dataset
- Chia train/val/test:
  python3 tools/split_dataset.py --csv /path/to/dataset/data.csv --out_dir /path/to/dataset --val 0.15 --test 0.15 --seed 42
- (Tùy chọn) rosbag -> dataset:
  python3 tools/rosbag_to_dataset.py --bag your.bag --out_dir /path/to/dataset --image-topic /main_camera/image_raw --twist-topics /mavros/setpoint_velocity/cmd_vel_unstamped /cmd_vel

3) Huấn luyện mô hình (PyTorch)
- Ví dụ:
  python3 ai/train.py \
    --data_root /path/to/dataset \
    --train_csv train.csv --val_csv val.csv --test_csv test.csv \
    --epochs 50 --batch_size 64 --lr 1e-3 --weight_decay 1e-5 \
    --log_dir runs/exp1 --out_dir outputs/exp1 \
    --flip_prob 0.5 --jitter 0.2 --noise_std 0.02 --motion_blur_prob 0.2 \
    --early_patience 8
- Xuất .onnx vào outputs/exp1/model_best.onnx

4) Inference (ROS node)
- Chạy:
  roslaunch wicom_path_following inference.launch \
    model_path:=/abs/path/to/model_best.onnx \
    fwd_speed:=0.3 max_yaw_rate:=0.8 smooth_alpha:=0.3 camera_rotate_deg:=0
- Node:
  - Subscribe /main_camera/image_raw
  - Nạp ONNXRuntime (CUDA nếu có), preprocess 160x90 RGB
  - Dự đoán yaw_rate, EMA smoothing, clamp theo max_yaw_rate
  - Heuristic confidence (edge density), nếu thấp: linear.x=0 (dừng), angular.z=0
  - Publish ≥ 20 Hz lên /mavros/setpoint_velocity/cmd_vel_unstamped (TwistStamped, body-frame)
  - Có service toggle: /path_following/toggle (std_srvs/SetBool)

5) Tinh chỉnh
- fwd_speed: 0.2–0.5 m/s tùy ổn định
- max_yaw_rate: 0.5–1.2 rad/s tùy độ gấp khúc đường
- smooth_alpha: 0.2–0.5 (EMA), alpha thấp => mượt hơn nhưng trễ
- Augment: tăng jitter/noise/motion blur nếu overfit/lóa sáng
- camera_rotate_deg: hiệu chỉnh nếu camera bị xoay
- min_edge_density: tăng để bảo thủ dừng khi khó nhận diện

6) Lưu ý an toàn
- Chạy OFFBOARD yêu cầu publish ≥ 2 Hz tối thiểu, khuyến nghị 20 Hz
- Khi “not confident” => dừng tiến; có thể thêm nhánh fail-safe để giữ vị trí/hover
- Thử nghiệm mô phỏng trước, sau đó mới lên drone thật

roslaunch wicom_path_following dataset_logger.launch out_dir:=/home/phuoc/datasets/path_following/data image_topic:=/main_camera/image_raw cmd_twist_topics:=/mavros/setpoint_velocity/cmd_vel_unstamped cmd_timeout:=0.5 save_stride:=1# wicom_path_following
