# Hướng dẫn Package ROS Noetic cho Raspberry Pi 5 và Laptop

Package này hỗ trợ thiết lập ROS Noetic phân tán: **Laptop** chạy `roscore` và `teleop` (điều khiển bàn phím), **Raspberry Pi 5** chạy camera và động cơ (bánh xe và tay máy).  
Hướng dẫn bao gồm hai phần chính:
1. Điều khiển động cơ bánh xe bằng `velocity_node` và `teleop`.
2. Điều khiển động cơ tay máy bằng `position_node`.

---

## Bảng Thành Phần Phần Cứng

### Linh kiện chính cho robot

| STT | Tên                                      | Số lượng | Giá (ước lượng)   | Tổng giá       |
|-----|------------------------------------------|----------|-------------------|----------------|
| 1   | Combo Raspberry Pi 5 (Pi 5 + thẻ nhớ + nguồn + vỏ) | 1        | 3.571.000 VND     | 3.571.000 VND  |
| 2   | Module camera đôi 8MP, camera stereo IMX219-83 | 1        | 1.500.000 VND     | 1.500.000 VND  |
| 3   | Raspberry Pi Pico 2W kèm header          | 1        | 290.000 VND       | 290.000 VND    |
| 4   | PIN LI-PO 3s 11.1v 2200mAh 8C           | 1        | 225.000 VND       | 225.000 VND    |
| 5   | Module Hạ Áp Buck DC-DC Vin 9-36V Vout 5V 5A XY-3606 | 1        | 64.000 VND        | 64.000 VND     |

### Linh kiện ROBOTIS

| STT | Tên                                      | Số lượng | Giá (ước lượng)   | Tổng giá       |
|-----|------------------------------------------|----------|-------------------|----------------|
| 1   | OPENRB-150                               | 1        | $27,39            | $27,39         |
| 2   | Động cơ Dynamixel XL430-W250T            | 2        | $54,89            | $109,78        |
| 3   | Bánh bi điều hướng                       | 2        | $3,85             | $7,70          |
| 4   | Dây cắm động cơ TTL 18cm                 | 1        | $20,90            | $20,90         |
| 5   | Plate Turtlebot                          | 3        | $20,90            | $62,70         |
| 6   | Bánh xe Turtlebot3                       | 1        | $10,34            | $10,34         |
| 7   | PCB Supporter                            | 1        | $7,70             | $7,70          |

**Lưu ý:** Giá có thể thay đổi tùy theo nhà cung cấp và thời điểm mua.

---

## Yêu cầu

**Laptop:**
- Hệ điều hành: Ubuntu 20.04 với ROS Noetic đã cài đặt.

**Raspberry Pi 5:**
- Hệ điều hành: Ubuntu 20.04 (hoặc Raspbian với ROS Noetic desktop).
- ROS Noetic đã cài.
- Phần cứng:
  - Camera USB (Camera V2.1).
  - Động cơ Dynamixel kết nối với OPENRB 150.
- Gói phần mềm: `dynamixel_sdk`, `dynamixel_sdk_examples`.

**Mạng:**
- Cả hai thiết bị phải cùng mạng Wi-Fi.

**Workspace:**
- Không gian làm việc ROS (ví dụ: `~/catkin_ws`) trên cả hai thiết bị.

---

## Cài đặt

### Clone Repository
Trên **cả laptop và Raspberry Pi 5**, clone package vào workspace ROS:

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

1. **Tìm địa chỉ IP**:
   - Trên laptop, chạy lệnh:
     ```bash
     ifconfig
     ```
     Ghi lại IP của Laptop (ví dụ: `192.168.1.122`) và IP của Raspberry Pi 5 (ví dụ: `192.168.1.100`).

2. **Trên Raspberry Pi 5**, thêm biến môi trường vào `~/.bashrc`:
   ```bash
   echo "export ROS_MASTER_URI=http://LAPTOP_IP:11311" >> ~/.bashrc
   echo "export ROS_HOSTNAME=PI_IP" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Trên Laptop**, thêm biến môi trường vào `~/.bashrc`:
   ```bash
   echo "export ROS_MASTER_URI=http://LAPTOP_IP:11311" >> ~/.bashrc
   echo "export ROS_HOSTNAME=LAPTOP_IP" >> ~/.bashrc
   source ~/.bashrc
   ```

**Lưu ý:** Thay `PI_IP` và `LAPTOP_IP` bằng địa chỉ IP thực tế của Raspberry Pi 5 và Laptop.

---

## Sử dụng

### Phần 1: Điều Khiển Động Cơ Bánh Xe bằng `velocity_node` và `teleop`

#### Mục đích
- Sử dụng `teleop` trên **laptop** để gửi lệnh điều khiển tốc độ đến `velocity_node` trên **Raspberry Pi 5**, từ đó điều khiển động cơ bánh xe.

#### Các bước thực hiện

##### Trên Laptop:
1. **Chạy `roscore`** (nếu chưa chạy):
   ```bash
   roscore
   ```

2. **Chạy `teleop_node`** để điều khiển bằng bàn phím:
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```
   - Node này publish các lệnh tốc độ (`Twist`) đến topic `/cmd_vel`.
   - Sử dụng các phím: `w` (tiến), `s` (lùi), `a` (quay trái), `d` (quay phải).

##### Trên Raspberry Pi 5:
1. **Chạy `velocity_node`** để nhận lệnh tốc độ và điều khiển động cơ bánh xe:
   ```bash
   rosrun dynamixel_sdk_examples velocity_node
   ```
   - Node này subscribe topic `/cmd_vel` và chuyển đổi lệnh `Twist` thành tốc độ cho động cơ bánh xe.

#### Cách hoạt động
- `teleop_twist_keyboard` gửi lệnh tốc độ (`linear` và `angular`) đến topic `/cmd_vel`.
- `velocity_node` nhận lệnh từ `/cmd_vel` và điều khiển tốc độ động cơ bánh xe.

---

### Phần 2: Điều Khiển Động Cơ Tay Máy bằng `position_node`

#### Mục đích
- Điều khiển vị trí của động cơ tay máy bằng cách gửi lệnh vị trí đến `position_node` trên **Raspberry Pi 5**.

#### Các bước thực hiện

##### Trên Raspberry Pi 5:
1. **Chạy `position_node`** để điều khiển vị trí động cơ tay máy:
   ```bash
   rosrun dynamixel_sdk_examples position_node
   ```
   - Node này:
     - Subscribe topic `/set_position` để nhận lệnh vị trí.
     - Cung cấp service `/get_position` để lấy vị trí hiện tại.

##### Trên Laptop (hoặc thiết bị trong mạng ROS):
1. **Gửi lệnh đặt vị trí mục tiêu**:
   ```bash
   ros2 topic pub -1 /set_position dynamixel_sdk_custom# Hướng dẫn Package ROS Noetic cho Raspberry Pi 5 và Laptop

Package này hỗ trợ thiết lập ROS Noetic phân tán: **Laptop** chạy `roscore` và `teleop` (điều khiển bàn phím), **Raspberry Pi 5** chạy camera và động cơ (bánh xe và tay máy).  
Hướng dẫn bao gồm hai phần chính:
1. Điều khiển động cơ bánh xe bằng `velocity_node` và `teleop`.
2. Điều khiển động cơ tay máy bằng `position_node`.

---

### Phần 3: Chạy Node Camera

#### Mục đích
- Phát video từ camera USB trên Raspberry Pi 5 để theo dõi từ xa.

#### Các bước thực hiện
##### Trên Raspberry Pi 5:
1. **Chạy node camera**:
   ```bash
   roslaunch raspicam_node camerav2_1280x760.launch
   ```

##### Trên Laptop:
1. **Xem video từ camera**:
   ```bash
   rqt
   ```
   - Chọn Plugin -> Visualization -> Image View -> `/raspicam_node/image/compressed`.

---

## Kiểm tra

- **Kiểm tra video**:
  - Trên laptop, sử dụng `rqt` để xem hình ảnh từ camera qua topic `/raspicam_node/image/compressed`.
- **Kiểm tra điều khiển bánh xe**:
  - Sử dụng `teleop_twist_keyboard` trên laptop để điều khiển động cơ bánh xe trên Raspberry Pi 5.
- **Kiểm tra điều khiển tay máy**:
  - Sử dụng lệnh `ros2 topic pub` hoặc `ros2 service call` để đặt và kiểm tra vị trí động cơ tay máy.

---

## Lưu ý

- Đảm bảo camera và động cơ được kết nối đúng (xem tài liệu trong package).
- Kiểm tra trạng thái Torque của động cơ (giá trị `1` là hoạt động bình thường).
- Nếu gặp lỗi mạng:
  - Kiểm tra IP của cả hai thiết bị.
  - Đảm bảo cổng `11311` không bị chặn bởi firewall.
- Đảm bảo các gói `dynamixel_sdk` và `dynamixel_sdk_examples` được cài đặt đúng.

---

## Hỗ trợ

- Xem thêm: [ROS Wiki](http://wiki.ros.org/).
- Báo lỗi tại: Issues trên GitHub Repository của bạn.
