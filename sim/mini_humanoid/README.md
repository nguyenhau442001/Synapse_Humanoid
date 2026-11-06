# Mini humanoid 20 DoF — mô hình mô phỏng v0.5

Mô hình hộp (box) dựng từ đúng kích thước và khối lượng trong tài liệu thiết kế:
cao 303 mm, nặng ~481 g, trọng tâm cao ~146 mm. Servo chọn theo tải từng khớp:

| Servo | Khớp | Lực kẹt (giới hạn trong sim) |
| --- | --- | --- |
| Corona DS843MG × 10 | hông roll, hông pitch, gối, cổ chân pitch, cổ chân roll | 4,8 kg·cm = 0,471 N·m |
| MG90S × 4 | hông yaw, vai pitch | 2,2 kg·cm = 0,216 N·m |
| SG90 × 6 | vai roll, khuỷu, cổ yaw, đầu pitch | 1,8 kg·cm = 0,177 N·m |

Thay đổi so với v0.4: servo theo tải (nhẹ hơn 74 g), đầu to theo vỏ A, thêm bảng DH,
bảng động học và động lực học (`kin_dyn.py`, thư mục `tables/`). Phương án trộn rẻ hơn:
`VARIANT=mix python gen_robot.py` (MG92B cho hông pitch và cổ chân).

| File | Dùng cho |
| --- | --- |
| `gen_robot.py` | Nguồn duy nhất: sửa kích thước/khối lượng ở đây rồi chạy lại để sinh 2 file dưới |
| `humanoid.xml` | MuJoCo (MJCF): 20 position actuator có giới hạn lực, IMU, keyframe `stand` |
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

- `demo.py --check`: đi 8 bước tiến 167 mm, xoay 62° (mục tiêu 60°), thân nghiêng tối đa 6,8°, không ngã.
  Mọi servo dưới 1/3 lực kẹt; cao nhất vai pitch phải 28% (khi vẫy tay), hông roll 24%.
- `kin_dyn.py`: động học thuận từ bảng DH khớp với MuJoCo, sai số 0,0 mm trên 300 tư thế ngẫu nhiên.
  Hệ số dư nhỏ nhất (mức liên tục / mô-men cần): vai pitch phải 1,20, hông roll 1,37, gối 1,42.

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
