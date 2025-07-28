# Hướng dẫn Package ROS Noetic cho Raspberry Pi 3 và Laptop

Package này hỗ trợ thiết lập ROS Noetic phân tán: **Laptop** chạy `roscore` và `teleop` (điều khiển bàn phím), **Raspberry Pi 3** chạy camera và động cơ.  
Làm theo hướng dẫn dưới để clone và sử dụng.

---

## Yêu cầu

**Laptop:**
- Ubuntu 20.04 với ROS Noetic (Cài đặt ROS Noetic).

**Raspberry Pi 3:**
- Ubuntu 20.04 (hoặc Raspbian với ROS Noetic desktop).
- ROS Noetic đã cài.
- Camera USB (Camera V2.1).
- Động cơ Dynamixel kết nối với OPENRB 150
- Gói: `dynamixel_sdk`, `dynamixel_sdk_examples`.

**Mạng:**  
Cả hai thiết bị cùng mạng Wi-Fi.

**Workspace:**  
Không gian làm việc ROS (ví dụ: `~/catkin_ws`) trên cả hai thiết bị.

---

## Cài đặt

### Clone Repository
Trên **cả laptop và Raspberry Pi 3**, clone package vào workspace ROS:

```bash
cd ~/catkin_ws/src
git clone https://github.com/EBang2k4/DynamixelSDK.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### Cài đặt Dependencies
Trên **cả hai thiết bị**, cài các gói cần thiết:

```bash
sudo apt update
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench
```

---

### Cấu hình Môi trường Mạng

- Trên laptop, tìm địa chỉ IP:

```bash
ifconfig
```

Ghi lại IP của PI (ví dụ: `192.168.1.100`).
Ghi lại IP của Laptop (ví dụ: `192.168.1.122`).

- Trên **Pi 3**, thêm biến môi trường vào `~/.bashrc`:

```bash
echo "export ROS_MASTER_URI=http://LAPTOP_IP" >> ~/.bashrc
echo "export ROS_HOSTNAME=PI_IP" >> ~/.bashrc
source ~/.bashrc
```

- Trên **LAPTOP**, thêm biến môi trường vào `~/.bashrc`:

```bash
echo "export ROS_MASTER_URI=http://LAPTOP_IP" >> ~/.bashrc
echo "export ROS_HOSTNAME=LAPTOP_IP" >> ~/.bashrc
source ~/.bashrc
```

**Lưu ý:** Thay `PI_IP` và `LAPTOP_IP` bằng IP của Pi 3 và Laptop tương ứng.
 
---

## Sử dụng

### Trên Laptop:

- Chạy `roscore`:

```bash
roscore
```

- Mở terminal mới, chạy `teleop` để điều khiển:

```bash
rosrun teleop teleop_node
```

---

### Trên Raspberry Pi 3:

- Chạy node camera để phát video:

```bash
roslaunch raspicam_node camerav2_1280x760.launch
```

- Chạy node điều khiển động cơ:

```bash
rosrun dynamixel_sdk_examples velocity_node
```

---

### Kiểm tra:

- Trên laptop, xem video từ camera:

```bash
rqt
```
Chọn Plugin -> Visualization -> Image View -> raspicam_node/image/compressed

- Dùng phím (theo hướng dẫn `teleop_node`) để điều khiển động cơ trên Pi 3.

---

## Lưu ý

- Đảm bảo camera và động cơ được kết nối đúng (xem tài liệu trong package).
- Check hoạt động của động cơ khi khởi động bằng DXL Torque (nếu là 1 thì hoạt động bình thường).
- Nếu gặp lỗi mạng, kiểm tra IP và firewall (cổng `11311`).

---

## Hỗ trợ

- Xem thêm: [ROS Wiki](http://wiki.ros.org/).
- Báo lỗi tại: Issues trên GitHub Repository của bạn.

