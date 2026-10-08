HT_V1.1_AMR_B2

Phiên bản chốt thử nghiệm tuyến B2 - Nguyễn Thái Vinh.
Mục tiêu vận hành: 1 lệnh + 1 nút.

1) Chạy: ./HT_V1_START.sh
2) Trên GUI nhấn: BẮT ĐẦU LÀM VIỆC B2
3) Chờ HỆ THỐNG SẴN SÀNG rồi mới đưa vật vào Cảm biến 1.

HT_V1.1 sửa các lỗi đã quan sát trên HT_V1:
- ESP32/Supervisor mở sớm để ACK ngay từ đầu, nhưng Work Mode vẫn khóa tới khi toàn hệ thống đạt.
- ros2 param get có retry hữu hạn, tránh fail giả do một lần CLI timeout.
- preflight B2 dùng wrapper HT_V1_B2_AMR.py, giữ nguyên mission gốc và chỉ gia cố đọc tham số ROS2.
- kiểm tra đứng yên chờ mẫu /odom đầu tiên riêng; không còn hiển thị giả 999 m/s khi n=0.
- lỗi preflight hiển thị nguyên nhân thật.
- GUI hiển thị TIẾN ĐỘ KHỞI ĐỘNG thay vì phần trăm sức khỏe gây hiểu nhầm.

Không thay T1/T2/T3/T4, FIX12, safety chain hay thuật toán tuyến B2 gốc.
Chưa được coi là FINAL vật lý cho tới khi full B2 PASS và 3 vòng liên tiếp PASS.
