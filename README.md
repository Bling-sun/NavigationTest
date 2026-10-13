# NavigationTest

本项目用于评估机器人 A/B 点循环导航的重复定位精度,提供两个独立指标:

1. **视觉精度** —— 用相机拍摄的 AprilTag 标定板,通过 PnP 解算相机相对标定板的真实物理位姿,在多次到达同一目标时计算位姿离散度(衡量"机器人实际到达的位置一致不一致")。
2. **里程计精度** —— 比较"下发的目标位姿"与"机器人到达后自报的里程计位姿",衡量"机器人自认为有没有到点"。

两者结合,可以同时回答:
- 机器人**自认为**到点了吗? → 里程计误差
- 机器人**实际上**有没有到同一个位置? → 视觉精度

---

## 获取代码

```bash
git clone https://github.com/Bling-sun/NavigationTest.git
cd NavigationTest
```

## 目录结构

```
NavigationTest/
├── nav_env/                              # Python 虚拟环境 自行创建
├── run_nav_roundtrip_capture.py          # 数据录制脚本样例 (ROS2)，自行替换
├── calculate_accuracy.py                 # 视觉重复定位精度
├── calculate_target_odom_error.py        # 下发目标 vs 里程计误差
├── nav_roundtrip_captures/               # 录制结果目录
│   ├── round01_A_x0.56_y1.59_yaw14.15/
│   │   ├── image.jpg
│   │   └── metadata.json
│   ├── round02_B_.../
│   └── ...
└── README.md
```

每个 `roundNN_<点位>_x<x>_y<y>_yaw<yaw>/` 子目录包含一次到点后采集的相机图像和 metadata。

---

## 一、环境安装

### 1.1 系统依赖

- Linux (项目验证环境为 Ubuntu / Linux 6.17)
- ROS 2 (录制脚本依赖 `rclpy` 与自定义消息 `rbs_msgs`)
- Python 3.10+

### 1.2 创建 Python 虚拟环境

```bash
cd NavigationTest
python3 -m venv nav_env
source nav_env/bin/activate
```

### 1.3 安装 Python 依赖

```bash
pip install --upgrade pip
pip install opencv-contrib-python numpy
```

> ⚠️ 必须用 `opencv-contrib-python`(而非 `opencv-python`),否则缺少 `cv2.aruco` 模块。
>
> 当前验证版本: `cv2 4.13.0`, `numpy 2.4.6`。

### 1.4 ROS 2 环境(仅录制脚本需要)

```bash
source /opt/ros/<distro>/setup.bash
source <你的 rbs_msgs 工作空间>/install/setup.bash
```

录制脚本还需要外部的 `capture_nav_pose_image` 模块（当前仓库未包含，需从机器人平台工作空间提供）(被 `run_nav_roundtrip_capture.py` import),它提供 `CaptureNode` / `wait_for_first_image` / `safe_name`。

---

## 二、相机和标定板配置

### 2.1 标定板

- **类型**: AprilTag 36h11(代码默认值,在 `calculate_accuracy.py:27` 配置)。
- **真实边长**: 默认 12 cm(`MARKER_SIZE = 0.12`)。如果你打印的码不是 12 cm,**必须**修改这个值,否则视觉测距会有比例误差。
- **粘贴位置**: 仅 A 点贴标定板,B 点不贴。这也是为什么视觉脚本只评估 A 点。

### 2.2 相机内参

`calculate_accuracy.py` 顶部写死了相机内参矩阵 `CAMERA_MATRIX` 和畸变系数 `DIST_COEFFS`:

```python
CAMERA_MATRIX = np.array([
    [606.020874023438, 0.0, 327.752685546875],
    [0.0, 605.950622558594, 245.438659667969],
    [0.0, 0.0, 1.0]
])
DIST_COEFFS = np.zeros((5, 1))   # 相机已做畸变矫正,系数全 0
```

**换相机务必重新填这两个矩阵**,否则解算出的距离与实际不符。
内参可以从相机驱动 `camera_info` 话题或厂商标定文件直接读出 K / D。

---

## 三、数据录制 —— `run_nav_roundtrip_capture.py`

脚本会向 `/navigation/topic/goal` 发布导航目标,等待 `/navigation/topic/result` 反馈到点,延迟一段时间后采集 RGB 图像与里程计位姿,并写入 metadata。

### 3.1 启动

```bash
source /opt/ros/<distro>/setup.bash
source <rbs 工作空间>/install/setup.bash
source nav_env/bin/activate

python3 run_nav_roundtrip_capture.py --rounds 50 --start A
```

### 3.2 常用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--rounds` | `1` | A→B→A 循环次数 |
| `--start` | `A` | 起始点(`A` 或 `B`) |
| `--image-topic` | `/cameras/head/rgb/image_raw` | RGB 图像话题 |
| `--target-frame` | `map` | TF 目标坐标系 |
| `--source-frame` | `rbs_base_link` | TF 源坐标系 |
| `--nav-frame` | `odom` | 导航目标使用的坐标系 |
| `--timeout-sec` | `300.0` | 单次到点超时 |
| `--acceptable-radius-m` | `0.03` | 到点容差 |
| `--settle-sec` | `1.0` | 到点后稳定时间(再采图) |
| `--output-root` | `nav_roundtrip_captures/` | 录制根目录 |

### 3.3 目标点

A、B 两点写在脚本头部([run_nav_roundtrip_capture.py:35-36](run_nav_roundtrip_capture.py#L35-L36)):

```python
POINT_A = NavPoint("A", 0.56, 1.59, 15.5)
POINT_B = NavPoint("B", 0.97, -0.5, 12.65)
```

修改坐标后重新启动即可。每次到点的子目录命名为
`roundNN_<点位>_x<实际x>_y<实际y>_yaw<实际yaw>/`。

### 3.4 metadata.json 字段

每个子目录里 `metadata.json` 包含:

```jsonc
{
  "navigation": {
    "round_index": 1,
    "point_name": "A",
    "target": { "x": 0.56, "y": 1.59, "yaw_degree": 15.5 },
    "error":  { "dx": -0.0024, "dy": -0.0039, "yaw_degree": -1.35 }
  },
  "image": { "file": "image.jpg", "frame_id": "head_color_optical_frame", ... },
  "pose": {
    "translation": { "x": 0.5575, "y": 1.5861, "z": -0.3667 },
    "rotation_rpy_degree": { "yaw": 14.149, ... }
  },
  "summary": {
    "target": { "x": 0.56, "y": 1.59, "yaw_degree": 15.5 },
    "actual": { "x": 0.56, "y": 1.59, "yaw_degree": 14.15 }   // 已四舍五入到 0.01
  }
}
```

> 评估脚本读 `pose.translation` 而不是 `summary.actual`,因为后者已被四舍五入到 1 cm,会把毫米级误差吃掉。

---

## 四、视觉重复定位精度 —— `calculate_accuracy.py`

衡量机器人多次到达 A 点时,**真实物理位置**有多一致(不依赖里程计自报)。

### 4.1 启动

**启动前要把录制好的数据拷贝替换到 NavigationTest 目录下。**

```bash
source nav_env/bin/activate
python3 calculate_accuracy.py
```

### 4.2 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--captures_dir` | `nav_roundtrip_captures` | 录制根目录 |
| `--ref` | `round01_A_x0.56_y1.59_yaw14.15` | 参考位置子目录名(以它的相机位姿作基准) |
| `--point` | `A` | 仅评估该点位的 round |

### 4.3 输出示例

```
[round03_A_x0.56_y1.59_yaw14.13] 物理平移误差:   0.00 mm  旋转:  0.00°  (...)
[round05_A_x0.58_y1.58_yaw15.43] 物理平移误差:  20.89 mm  旋转:  2.30°  (...)
...
==================================================
             机器人重复定位精度统计报告
==================================================
 有效测试次数 : 37
【平移误差 (3D物理欧氏距离)】
  平均误差 :  28.10 mm
  最大误差 :  55.65 mm
  标准差(1σ):  12.18 mm
【各轴绝对误差 (基于标定板坐标系)】
  X轴 (左右) 平均 :   7.21 mm
  Y轴 (上下) 平均 :  10.51 mm
  Z轴 (前后) 平均 :  23.76 mm
【旋转误差】
  平均角度误差 :  1.846 度  最大: 4.178 度  σ: 0.873 度
```

---

## 五、目标 vs 里程计误差 —— `calculate_target_odom_error.py`

衡量机器人**自认为**到点的精度(下发目标 vs 里程计自报位姿)。

### 5.1 启动

```bash
source nav_env/bin/activate
python3 calculate_target_odom_error.py
```

### 5.2 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--captures_dir` | `nav_roundtrip_captures` | 录制根目录 |
| `--point` | `A` | 仅评估该点位 |
| `--target_x` | 无 | 可选,只筛选指定目标 x |
| `--target_y` | 无 | 可选,只筛选指定目标 y |
| `--target_tol` | `0.05` | 上述 x/y 过滤的容差(米) |

### 5.3 多目标提示

如果录制目录下同点位但目标不一致,脚本会提示:

```
[注意] 检测到多个不同的下发目标，建议用 --target_x/--target_y 过滤同一目标:
   target=(0.56, 1.59, yaw=15.5°)  共 38 次
   target=(0.18, 1.0,  yaw=56.5°)  共 14 次
```

只想看 (0.56, 1.59) 这一组:

```bash
python3 calculate_target_odom_error.py --target_x 0.56 --target_y 1.59
```

### 5.4 输出示例

```
round01_A_x0.56_y1.59_yaw14.15        -2.42    -3.88      4.58   -1.351
round03_A_x0.56_y1.59_yaw14.13        -4.72    -4.75      6.70   -1.375
...
========================================================================
                       Target vs Odom 误差统计 (A 点)
========================================================================
 有效样本数: 38
【平面距离误差 √(dx²+dy²)】
  平均   30.25 mm   最大   78.04 mm   最小    4.58 mm   σ   15.93 mm
【X 方向偏差】平均 -7.21 mm   σ 30.06 mm
【Y 方向偏差】平均 -9.40 mm   σ 11.18 mm
【Yaw 偏差】平均 -0.704°   |Yaw| 平均 0.816°   最大 1.391°
```

---

## 六、测试原理

### 6.1 视觉重复定位精度的解算流程

1. **检测 AprilTag**: `cv2.aruco.ArucoDetector` 在图像中找到码的四个角点。
2. **PnP 解算相机位姿**: 已知码的四个角点在标定板坐标系下的真实 3D 坐标
   (`±MARKER_SIZE/2`),配合相机内参 `K`,用 `cv2.solvePnP` 解出
   `T_board_camera` —— 即 **相机在标定板坐标系下的位置和姿态**(rvec, tvec)。
3. **求相机在标定板系下的位置**: PnP 解出的是"相机系到标定板系"的变换;
   把相机原点反投影到标定板系: `P_cam = -R.T @ tvec`。
4. **多次拍摄对比**: 选一张图作为参考(默认 round01_A),其它 round 的相机位置
   都和它比:
   - 平移误差: `‖P_cam_cur - P_cam_ref‖`
   - 旋转误差: `R_diff = R_cur · R_ref.T`,取 `arccos((tr(R_diff) - 1) / 2)`。

> ⚙️ **为什么这能反映机器人重复定位精度**: 相机刚性安装在机器人上,标定板固定
> 在 A 点环境里。如果机器人每次到达 A 点的位姿一致,那么相机在标定板坐标系
> 下的位置也一致。任何离散度都直接来自机器人到点位姿的离散度。

### 6.2 视觉精度 vs 里程计自报误差的差异

| 维度 | 视觉精度 (`calculate_accuracy.py`) | 里程计误差 (`calculate_target_odom_error.py`) |
|------|------------------------------------|------------------------------------------------|
| 数据来源 | 相机图像 + AprilTag PnP | metadata.json 中的 target 与 pose |
| 参考点 | 某次到点的图像 (round01) | 下发的导航目标 |
| 衡量对象 | 多次到点的真实物理一致性 | 机器人自报与下发目标的一致性 |
| 受影响因素 | 机器人定位/控制误差(真实) | 机器人定位/控制误差 + 里程计漂移 |
| 是否依赖标定板 | 是 | 否 |

如果两者数值差距大,通常意味着 **里程计漂移** 已经不可忽视(机器人自认为到了
,但实际位置和上一次差很多;或反过来)。

### 6.3 坐标系约定

- **相机光学系**: x 向右, y 向下, z 向前(OpenCV 默认)。
- **标定板系**: 取码中心为原点, x 向右, y 向上, z 出板面。
- 视觉脚本中"X 左右 / Y 上下 / Z 前后"指标定板坐标系。
- 里程计脚本中 `dx, dy` 为 `actual - target`,坐标系跟随 `metadata.pose.target_frame`(默认 `map`)。

### 6.4 已知局限

1. AprilTag 单标记 PnP 在距离较远或角度刁钻时,**Z(深度)方向**误差会显著大于
   X/Y。本项目实测 Z 平均误差(24 mm)就明显大于 X/Y(7-10 mm),属正常现象。
   想压低 Z 误差,可以换多 marker 标定板或 ChArUco。
2. `MARKER_SIZE` 配置错误会让所有距离按比例缩放,**先用尺子量一下打印的码**。
3. 视觉脚本目前只检测画面中第一个 ArUco 码;场景里多码时请确保只贴一个,或扩展代码做 ID 过滤。
4. 里程计脚本依赖 `pose.translation` 是已经过 TF 查询到 `map` 系的位置;如果
   录制时 TF 不稳定,这部分误差会混入结果。

---

## 七、常见问题

**Q1: 跑 `calculate_accuracy.py` 报 "[警告] xxx 未检测到标定板"**

- 检查图像是否完整包含码、码是否被遮挡或反光。
- 若所有 round 都失败,检查 `ARUCO_DICT_ID` 是否与你打印的码字典一致(本项目用 36h11)。

**Q2: 平移误差大得离谱(米级)?**

- 99% 是 `MARKER_SIZE` 写错了,或者相机内参矩阵不匹配当前相机。

**Q3: 录制时 `wait_result` 一直等不到结果?**

- 确认 `/navigation/topic/result` 上有发布,且 `goal_id` 一致。
- 用 `ros2 topic echo /navigation/topic/result` 排查导航节点是否在工作。

**Q4: 录制目录里出现了不同目标的 round?**

- `run_nav_roundtrip_capture.py` 的 `POINT_A / POINT_B` 是固定的,但你可能在不同
  时间用不同坐标跑过。`calculate_target_odom_error.py` 检测到多目标会提示,用
  `--target_x/--target_y` 过滤即可。
