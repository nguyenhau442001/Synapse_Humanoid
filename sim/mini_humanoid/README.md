# Mini humanoid 20 DoF — mô hình mô phỏng v0.6

Mô hình hộp (box) dựng từ đúng kích thước và khối lượng trong tài liệu thiết kế:
cao 303 mm, nặng ~536 g, trọng tâm cao ~138 mm. Servo chọn theo tải từng khớp, toàn bộ mua được ở TP.HCM:

| Servo | Khớp | Lực kẹt (giới hạn trong sim) |
| --- | --- | --- |
| TowerPro MG92B × 10 | hông roll, hông pitch, gối, cổ chân pitch, cổ chân roll | 3,5 kg·cm = 0,343 N·m |
| MG90S × 4 | hông yaw, vai pitch | 2,2 kg·cm = 0,216 N·m |
| SG90 × 6 | vai roll, khuỷu, cổ yaw, đầu pitch | 1,8 kg·cm = 0,177 N·m |

Thay đổi so với v0.5: chân dùng MG92B (bán trong nước) thay cho Corona DS843MG (phải nhập),
nặng hơn 55 g. Muốn so sánh với DS843MG: `LEG_HEAVY=DS843MG python gen_robot.py`.

| File | Dùng cho |
| --- | --- |
| `gen_robot.py` | Nguồn duy nhất: sửa kích thước/khối lượng ở đây rồi chạy lại để sinh 2 file dưới |
| `humanoid.xml` | MuJoCo (MJCF): 20 position actuator có giới hạn lực, IMU, 4 cảm biến lực bàn chân (gót, mũi), keyframe `stand` |
| `humanoid.urdf` | Foxglove / RViz / ROS 2 (`robot_description`) |
| `demo.py` | Đi thẳng 8 bước → xoay trái 60° → vẫy tay; `--check` in báo cáo tải servo |
| `kin_dyn.py` | Bảng DH 5 chuỗi + kiểm FK với MuJoCo, bảng khâu và bảng khớp → `tables/*.csv` |

## Chạy trên Mac

```zsh
python3 -m venv ~/humanoid-venv && source ~/humanoid-venv/bin/activate
pip install mujoco numpy

python gen_robot.py                         # sinh lại humanoid.xml + humanoid.urdf
python -m mujoco.viewer --mjcf=humanoid.xml # xem mô hình, kéo từng khớp ở panel Control
mjpython demo.py                            # xem robot đi + xoay (trên macOS phải dùng mjpython)
python demo.py --check                      # chạy không cửa sổ, in báo cáo
python kin_dyn.py                           # kiểm DH, sinh tables/*.csv
```

Trong viewer: nhấn **Space** để tạm dừng, **Backspace** để reset, bật **Contact force**
ở tab Rendering để thấy lực chân.

## Hệ trục: URDF, MuJoCo, Gazebo

`gen_robot.py` sinh cả ba từ cùng một cây khớp, nên mọi khung trùng nhau tuyệt đối:

| File | Dùng cho |
| --- | --- |
| `humanoid.xml` | MuJoCo (MJCF), có IMU, ToF (`rangefinder`), camera `head_cam`, 4 FSR |
| `humanoid.urdf` | ROS / RViz / Foxglove; MuJoCo cũng nạp thẳng được (`mujoco.MjModel.from_xml_path`) |
| `src/humanoid_description/urdf/mico.urdf.xacro` | Gazebo Harmonic: cảm biến IMU, camera, ToF + `gz_ros2_control` 20 khớp |

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

Kiểm tra: `python check_urdf.py` đọc URDF bằng `urdf_parser_py`, so động học thuận URDF với MJCF trên 200 tư thế
ngẫu nhiên (sai số 0,0000 mm), và nạp URDF vào MuJoCo.

Gazebo (ROS 2 Jazzy + Gazebo Harmonic, trên Ubuntu): `colcon build --packages-select humanoid_description`, rồi
`ros2 launch humanoid_description mico_gz.launch.py`. Topic ra: `/imu`, `/camera/image_raw`, `/tof`, `/joint_states`;
lệnh khớp gửi vào `/position_controller/commands` (20 góc, đúng thứ tự trong `config/mico_controllers.yaml`).
Bản Gazebo chưa được chạy thử trong môi trường này, mới kiểm tra ở mức xacro và trình đọc URDF.

## Xem trong Foxglove

Cách chắc chắn nhất là qua ROS 2 (cùng stack sẽ chạy trên Radxa):

```zsh
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat humanoid.urdf)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui   # thanh trượt 20 khớp
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Mở Foxglove → Open connection → `ws://localhost:8765` → panel **3D**: robot hiện từ topic
`/robot_description` + `/tf`, kéo thanh trượt để cử động. URDF chỉ dùng hình hộp,
không cần file mesh.

## Kết quả kiểm tra

- Va chạm giữa hai bàn chân được bật. Bàn chân đặt lệch ra ngoài trục cổ chân 6 mm; khi còn đặt giữa,
  hai đế (rộng 42, cách nhau 44) cọ vào nhau suốt lúc đi và xoay (lún 3,7 mm). Giờ khe hở nhỏ nhất là 3,8 mm.
- Cảm biến bàn chân `l_fsr_toe`, `l_fsr_heel`, `r_fsr_toe`, `r_fsr_heel` (FSR402): khi đứng yên tổng bốn cảm biến
  bằng đúng trọng lượng 5,25 N; gót 1,73 N, mũi 0,90 N mỗi chân.

- `demo.py --check`: đi 8 bước tiến 167 mm, xoay 62° (mục tiêu 60°), thân nghiêng tối đa 6,5°, không ngã.
  Hông roll là khớp căng nhất: đỉnh 38% lực kẹt, vượt mức liên tục (1/3) khoảng 1 s trong cả bài chạy;
  gối 35%. Các khớp khác dưới 1/3.
- `kin_dyn.py`: động học thuận từ bảng DH khớp với MuJoCo, sai số 0,0 mm trên 300 tư thế ngẫu nhiên.
  Hệ số dư nhỏ nhất (mức liên tục / mô-men cần) ở ca đứng một chân khi ngồi thấp: hông roll 0,89, gối 0,94,
  vai pitch phải 1,20. Dưới 1 nghĩa là không nên giữ tư thế đó lâu; dáng đi hiện tại chỉ đứng một chân ngắn.

## Thông số có thể chỉnh

- `gen_robot.py`: `KP = 8` (độ cứng servo, N·m/rad), `DAMPING = 0.02`, `ARMATURE = 5e-4`,
  `HIP_Z`, `THIGH`, `SHIN`, `ANKLE_Z`. KP là ước lượng — khi có servo thật, đo độ lệch góc
  dưới tải để chỉnh cho khớp.
- `demo.py`: `STEP` (24 mm), `LIFT` (10 mm), `SWAY` (0,20 rad), `LEAN` (−0,10 rad),
  `T_STEP` (1,6 s), `H0` (87 mm), `TURN` (20° mỗi cặp bước), `TURN_PAIRS` (3).

## Quy ước dấu khớp

Khung toạ độ: x tới trước, y sang trái, z lên trên. Góc dương theo quy tắc bàn tay phải:
hông yaw dương = mũi chân xoay sang trái; gối dương = gập; hông pitch âm = đưa chân ra trước;
vai roll trái dương = dang tay ra ngoài (tay phải ngược dấu); khuỷu âm = gập tay về trước.
Firmware STM32 cần dùng đúng quy ước này để bảng góc từ mô phỏng chạy được trên robot thật.
