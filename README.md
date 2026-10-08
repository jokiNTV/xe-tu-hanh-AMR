# AMR FINAL B2

Source code của hệ thống xe tự hành AMR vận chuyển hàng
đã hoàn thiện và được người vận hành xác nhận nghiệm thu.

## Trạng thái dự án

- Tuyến vận hành chính thức: B2
- Tuyến B1: Không còn thuộc phạm vi vận hành
- Trạng thái B2: PASS / FROZEN / STABLE
- Nền tảng: Ubuntu 22.04, ROS 2 Humble
- Bản đồ: mapVinh1_FIX12
- Ngày tạo bản sao GitHub: 08/10/2026

## Chức năng chính

- Định vị bằng AMCL, EKF và odometry
- Lập đường toàn cục bằng Theta*
- Điều khiển bám đường bằng MPPI
- Giới hạn vận tốc qua Velocity Smoother
- Bảo vệ chuyển động bằng Turn Safety Gate
- Giám sát va chạm bằng Collision Monitor
- Nhận nhiệm vụ từ ESP32
- Xác nhận kiện hàng bằng mã QR
- Điều khiển cơ cấu kẹp/nâng qua STM32 #2
- Điều khiển hai bánh qua STM32 #1
- Giao diện vận hành và giám sát hệ thống

## Cấu trúc source

| Thư mục | Vai trò |
|---|---|
| app | GUI, CORE, Supervisor và ứng dụng |
| mission | Nhiệm vụ B2 đã nghiệm thu |
| ros2 | Các ROS 2 package |
| localization | EKF, odometry, scan timing |
| navigation | Cấu hình MPPI |
| safety | Turn Safety Gate, Collision Monitor |
| launchers | Các script khởi động T2/T3/T4 |
| maps | Bản đồ FIX12 |
| firmware | STM32 #1, STM32 #2, ESP32 |
| tools | Các công cụ hỗ trợ |
| third_party | Source thư viện bên thứ ba |
| docs | Tài liệu và danh sách SHA256 |

## Nguyên tắc bảo toàn

Đây là bản sao source để lưu trữ và quản lý phiên bản.

Không tự ý thay đổi:
- Thuật toán điều hướng B2
- Các thông số điều khiển đã nghiệm thu
- Chuỗi Velocity Smoother / Safety Gate / Collision Monitor
- Logic PICK / DROP
- Firmware đang hoạt động trên xe

## Lưu ý khôi phục

Repository chứa source và cấu hình, không phải bộ cài
có thể chạy ngay trên một máy Ubuntu mới.

Các launcher và config vẫn giữ đường dẫn gốc.
Việc khôi phục yêu cầu chuẩn bị ROS 2, dependencies,
thiết bị phần cứng và ánh xạ đúng đường dẫn runtime.

Source STM32 #1 được sao lưu từ project có thông số
phù hợp bản nghiệm thu; cần đối chiếu riêng với firmware
thực tế đang nạp nếu yêu cầu chứng minh trùng tuyệt đối.

## Kiểm tra tính toàn vẹn

Chạy trong thư mục repository:

    sha256sum -c docs/SOURCE_MANIFEST.sha256

## Tài liệu bổ sung

Xem docs/SOURCE_LAYOUT.md.
