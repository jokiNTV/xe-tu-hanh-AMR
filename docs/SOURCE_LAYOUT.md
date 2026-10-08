# DANH MỤC SOURCE HỆ THỐNG AMR B2

## 1. Application
app/HT_V1_1_AMR_B2/

- HT_V1_START.sh: Khởi động ứng dụng
- HT_V1_STOP.sh: Dừng hệ thống
- HT_V1_GUI_AMR.py: Giao diện vận hành
- HT_V1_CORE_AMR.py: Điều phối hệ thống
- HT_V1_SUP_AMR.py: Quản lý nhiệm vụ
- HT_V1_B2_AMR.py: Wrapper tuyến B2
- HT_V1_ACT_AMR.py: Giao tiếp cơ cấu
- HT_V1_CAM_AMR.py: Camera broker

## 2. Mission
mission/B2_FROZEN/

Chứa source nhiệm vụ B2 đã được kiểm tra SHA256
với cấu hình ứng dụng.

## 3. ROS 2
ros2/

Chứa source navigation, STM32 bridge
và các file ROS package liên quan.

## 4. Localization
localization/

EKF, odometry adapter, IMU adapter,
scan mid-stamp và các script liên quan.

## 5. Navigation / Safety
navigation/
safety/
launchers/

Chứa cấu hình MPPI, các thành phần safety,
script khởi động và file cấu hình tương ứng.

## 6. Map
maps/

- mapVinh1_FIX12.yaml
- mapVinh1_FIX12.pgm

## 7. Firmware
firmware/STM32_1_Drive/
firmware/STM32_2_Actuator/
firmware/ESP32/

Chứa source cho hai STM32 và ESP32.

STM32 #2: V2_12_PICK_LIFT_FIX theo source đã kiểm tra.
STM32 #1: Project source ứng viên phù hợp thông số B2.
ESP32: tramnhiemvu.ino.

## 8. Third-party
third_party/

Chứa source LiDAR bên thứ ba.
Giữ các thông báo giấy phép gốc.

## 9. Kiểm tra
docs/SOURCE_MANIFEST.sha256

Danh sách SHA256 của các file trong bản sao.

## 10. Giới hạn xác minh
Kiểm tra source tĩnh và SHA256 không thay thế
kiểm chứng runtime hoặc so sánh firmware đang nạp.
Các tham chiếu B1 bên trong file B2 dùng chung
được giữ nguyên, không tự ý chỉnh sửa.
