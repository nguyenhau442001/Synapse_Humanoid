# Mini humanoid 20 DoF (Mico) — mô hình mô phỏng v0.7

Mô hình hộp (box) dựng từ đúng kích thước và khối lượng trong tài liệu thiết kế:
cao 303 mm, nặng ~536 g, trọng tâm cao ~138 mm. Servo chọn theo tải từng khớp, toàn bộ mua được ở TP.HCM:

| Servo | Khớp | Lực kẹt (giới hạn trong sim) |
| --- | --- | --- |
| TowerPro MG92B × 10 | hông roll, hông pitch, gối, cổ chân pitch, cổ chân roll | 3,5 kg·cm = 0,343 N·m |
| MG90S × 4 | hông yaw, vai pitch | 2,2 kg·cm = 0,216 N·m |
| SG90 × 6 | vai roll, khuỷu, cổ yaw, đầu pitch | 1,8 kg·cm = 0,177 N·m |

Thay đổi so với v0.5: chân dùng MG92B (bán trong nước) thay cho Corona DS843MG (phải nhập),
nặng hơn 55 g. Muốn so sánh với DS843MG: `LEG_HEAVY=DS843MG python gen_robot.py`.

Mô phỏng chạy bằng **ROS 2 Jazzy + Gazebo Harmonic** trong máy ảo Ubuntu (OrbStack trên Mac), xem bằng
**Foxglove** trên Mac. MuJoCo đã bỏ.

| File | Dùng cho |
| --- | --- |
| `gen_robot.py` | Nguồn duy nhất: sửa kích thước/khối lượng ở đây rồi chạy lại để sinh URDF và xacro Gazebo |
| `humanoid.urdf` | Foxglove / RViz / ROS 2 (`robot_description`), không cần Gazebo |
| `../../src/humanoid_description/urdf/mico.urdf.xacro` | Gazebo: `gz_ros2_control` 20 khớp, IMU, camera, ToF, odometry |
| `../../src/humanoid_control/scripts/mico_gait.py` | Dáng đi: 8 bước → xoay trái 60° → vẫy tay, gửi một `JointTrajectory` |
| `check_gz.py` | Chạy kèm Gazebo: có ngã không, đi được bao xa, tải từng servo so với lực kẹt |
| `kin_dyn.py` | Bảng DH 5 chuỗi + kiểm FK với URDF, bảng khâu, tải tĩnh, gộp tải đo trong Gazebo → `tables/*.csv` |
| `check_urdf.py` | URDF, xacro, YAML controller và dáng đi khớp nhau (tên khớp, thứ tự, cảm biến, khung) |

## Chạy: Gazebo trong OrbStack, xem bằng Foxglove trên Mac

```zsh
# trên Mac, một lần: tạo máy Ubuntu 24.04, cài ROS 2 Jazzy + Gazebo Harmonic, build workspace
./setup_orbstack.sh

# trong máy ảo (orb -m synapse-humanoid), ở thư mục repo
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch humanoid_description mico_gz.launch.py gait:=true
```

Gazebo chạy không giao diện (server + `--headless-rendering`). `foxglove_bridge` mở cổng 8765 trong máy ảo,
OrbStack chuyển cổng ra Mac: mở Foxglove → Open connection → `ws://localhost:8765`.

Trong panel **3D**: chọn display frame `odom` để thấy robot đi trên sàn (`/tf` có `odom → base_link` từ Gazebo
và các khớp từ `robot_state_publisher`). Thêm panel **Image** cho `/camera/image_raw`, **Plot** cho
`/imu.linear_acceleration.z` hoặc `/joint_states.effort[...]`.

Tham số launch:

| Tham số | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `gait` | `false` | chạy `mico_gait.py` sau khi controller sẵn sàng |
| `camera` | `true` | mô phỏng camera đầu và ToF (cần render; máy không có GPU thì đặt `false`, IMU vẫn chạy) |
| `foxglove` | `true` | bật `foxglove_bridge` |
| `gui` | `false` | mở giao diện Gazebo (cần màn hình, không dùng trong OrbStack) |

Topic: `/joint_states`, `/imu`, `/odom`, `/tf`, `/camera/image_raw`, `/tof`, `/clock`.
Điều khiển: `joint_trajectory_controller` (đang bật) nhận `/joint_trajectory_controller/joint_trajectory`;
`position_controller` (nạp sẵn, tắt) nhận 20 góc trên `/position_controller/commands` khi cần gửi lệnh liên tục.

Kiểm tra không cần nhìn (mở terminal thứ hai trong máy ảo):

```zsh
ros2 launch humanoid_description mico_gz.launch.py camera:=false foxglove:=false gait:=true
python3 sim/mini_humanoid/check_gz.py     # ~38 s thời gian mô phỏng, ghi tables/gz_servo_loads.csv
python3 sim/mini_humanoid/kin_dyn.py      # chỉ cần numpy, gộp bảng tải Gazebo vào tables/joints_kin_dyn.csv
python3 sim/mini_humanoid/check_urdf.py   # không cần gì ngoài Python
```

## Hệ trục: URDF và Gazebo

`gen_robot.py` sinh cả hai từ cùng một cây khớp, nên mọi khung trùng nhau tuyệt đối
(`check_urdf.py` so từng khớp, giới hạn và khung).

Quy ước: REP-103 (x tới trước, y sang trái, z lên trên), tên khung theo REP-120. `base_link` nằm ở điểm hông H,
cách sàn 115 mm khi đứng thẳng. Khung cảm biến:

| Khung | Gắn vào | Vị trí so với khung cha (mm) | Ghi chú |
| --- | --- | --- | --- |
| `imu_link` | `base_link` | 0, 0, 30 (cao 145 mm so với sàn) | MPU6050 trên tấm đỉnh hông, trục chip trùng trục robot |
| `camera_link` → `camera_optical_frame` | `head_pitch_link` | 31, 0, 42, chúi 15° | khung quang học: z nhìn tới, x sang phải, y xuống |
| `tof_link` | `head_pitch_link` | 31, 8, 45, chúi 20° | VL53L0X, tia dọc trục x |
| `l_mic_link`, `r_mic_link` | `head_pitch_link` | 2, ±38,5, 24 | x hướng ra ngoài tai |
| `l_sole`, `r_sole` | `*_ankle_roll_link` | 0, 0, −25 | trên mặt sàn, ngay dưới trục cổ chân A |
| `l_gripper`, `r_gripper` | `*_elbow_link` | 0, 0, −44 | tâm tay kẹp |

IMU đứng yên đo gia tốc (0; 0; +9,81) m/s² trong `imu_link` — firmware MPU6050 phải xuất đúng chiều này
(gắn mạch GY-521 mặt linh kiện lên trên, mũi tên X của mạch chỉ về phía trước robot; lắp khác thì sửa `rpy`
của `imu_link` trong `gen_robot.py`).

Xem riêng mô hình, không cần Gazebo:

```zsh
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat humanoid.urdf)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui   # thanh trượt 20 khớp (cần màn hình)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Kết quả kiểm tra (Gazebo Harmonic 8, ROS 2 Jazzy, DART, bước 1 ms)

- `check_gz.py`, dáng đi đầy đủ, `camera:=false`: không ngã, đi tới 187–190 mm, xoay +60…61° (mục tiêu 60°),
  thân nghiêng tối đa 7–12°, hông thấp nhất 112 mm. Bốn lần chạy liên tiếp đều đứng vững.
- IMU đứng yên đo +9,80 m/s² trên trục z.
- Tải servo trong lúc đi (phân vị 99): cổ chân pitch 49% lực kẹt, hông roll 46%, gối 39%. Hông roll vượt
  mức liên tục (1/3 lực kẹt) khoảng 7–8 s trong 36 s bài chạy, vì giữ thân nghiêng suốt pha một chân.
  Đây là khớp cần theo dõi nhiệt độ trên robot thật.
- `kin_dyn.py`: động học thuận từ bảng DH khớp với URDF, sai số 0,000 mm trên 300 tư thế ngẫu nhiên.
  Tải tĩnh lớn nhất: hông roll khi đứng một chân 0,83 kg·cm, gối khi ngồi xổm 0,57 kg·cm.
- Va chạm giữa hai bàn chân được bật. Bàn chân đặt lệch ra ngoài trục cổ chân 6 mm (khe hở nhỏ nhất 3,8 mm).

Bài học khi chuyển sang Gazebo:

- Gửi góc từ node Python 100 Hz (`/position_controller/commands`) làm mỗi lần chạy một khác, có lần ngã:
  lệnh tới lệch nhịp với vòng vật lý, chân chạm đất bị giật. Gửi cả bài thành một `JointTrajectory` cho
  `joint_trajectory_controller` (nội suy trong vòng ros2_control theo giờ mô phỏng) thì lần nào cũng như nhau,
  và tải đỉnh giảm từ ~100% xuống dưới 50% lực kẹt.
- Bước xoay chậm lại: 4 × 15°, mỗi bước 2,0 s (thay vì 3 × 20°, 1,6 s). Bàn chân trụ xoay dưới tải dễ trượt
  trong mô hình tiếp xúc của Gazebo.
- Camera và ToF cần render (ogre2). Máy ảo không có GPU vẫn chạy được với `--headless-rendering` nếu có EGL
  (Mesa); không được thì đặt `camera:=false`.

## Thông số có thể chỉnh

- `gen_robot.py`: `DAMPING = 0.02`, `FOOT_OUT`, `HIP_Z`, `THIGH`, `SHIN`, `ANKLE_Z`, `LEG_HEAVY`.
- `mico_gait.py`: `STEP` (24 mm), `LIFT` (10 mm), `SWAY` (0,20 rad), `LEAN` (−0,10 rad),
  `T_STEP` (1,6 s), `H0` (87 mm), `TURN` (15° mỗi cặp bước), `TURN_PAIRS` (4), `T_TURN_STEP` (2,0 s).
- `config/mico_controllers.yaml`: `update_rate` 200 Hz của controller_manager.

## Quy ước dấu khớp

Khung toạ độ: x tới trước, y sang trái, z lên trên. Góc dương theo quy tắc bàn tay phải:
hông yaw dương = mũi chân xoay sang trái; gối dương = gập; hông pitch âm = đưa chân ra trước;
vai roll trái dương = dang tay ra ngoài (tay phải ngược dấu); khuỷu âm = gập tay về trước.
Firmware STM32 cần dùng đúng quy ước này để bảng góc từ mô phỏng chạy được trên robot thật.
