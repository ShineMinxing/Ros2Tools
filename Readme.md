# Ros2Tools 🛠️

一组面向 **Unitree Go2 / 双足机器人** 的 **ROS 2 Humble / Ubuntu 22.04** 实用工具包。

| Package              | 功能简述                                                                 |
| -------------------- | -------------------------------------------------------------------- |
| **bt\_imu**          | 通过蓝牙读取 *VIT 智能眼镜* IMU，将姿态信息发布为 ROS2 话题 (`SMX/BTIMU` / `SMX/BTAngle`) |
| **control\_message** | 监听手柄 `Joy` 话题，映射为统一的运动 / 姿态控制指令 (`SMX/SportCmd` 等)                   |
| **control\_loop**    | 吊舱与机器人头部双闭环：<br>• 外环 – 头部 IMU → 目标角度<br>• 内环 – 实时云台伺服 & 机体运动补偿       |
| **gimbal\_record**   | 录制相机、IMU、云台状态等多源数据，同步保存为 **H.264 MP4 + CSV**，方便科研分析                  |
| **drone\_estimator** | 基于 YOLO-OBB 观测的无人机状态估计，调用 C 估计算法端口，输出 SMX/DroneStateEstimate 与 TF      |

> 📝 这些包均可 **独立使用**，亦可与 [Ros2Go2Base](https://github.com/ShineMinxing/Ros2Go2Base) / [Ros2Go2Estimator](https://github.com/ShineMinxing/Ros2Go2Estimator) 等仓库协同。

---

## ✨ 核心特点

* **即插即用** – 纯 ROS2 实现，无外部依赖（`bt_imu` 需蓝牙 4.0+）。
* **高效 C++17** – 关键节点手工内存管理，低延时、低 CPU 占用。
* **可配置** – 统一 `config.yaml`，话题名 / 串口 / IMU UUID / 控制增益均可热更新。
* **科研友好** – `gimbal_record` 同步写入帧精确时间戳，方便后期 Matlab / Python 分析。
* **融合估计** – `drone_estimator` 支持多目标评分筛选、状态/预测 TF 广播。

---

## 📂 目录结构

```text
Ros2Tools/
├── bt_imu/                 # 蓝牙 IMU → ROS2
├── control_message/        # Joy → SportCmd 映射
├── control_loop/           # 头部 IMU ↔ 吊舱 / 机体闭环
├── gimbal_record/          # 数据采集器
├── drone_estimator/        # 无人机观测与估计
├── config.yaml             # 全局参数示例
└── Readme.md               # ← 当前文档
```

---

## ⚙️ 快速安装

```bash
# 1. clone
cd ~/ros2_ws/LeggedRobot/src
git clone --recursive https://github.com/ShineMinxing/Ros2Tools.git

# 2. 编译所需包
cd .. && colcon build --packages-select \
  bt_imu control_message control_loop gimbal_record
source install/setup.bash
```

---

## 📝 主要节点概要

### 1. bt\_imu\_node

| 话题            | 类型                           | 方向 | 说明                 |
| ------------- | ---------------------------- | -- | ------------------ |
| `SMX/BTIMU`   | `sensor_msgs/Imu`            | 发布 | 完整四元数 + 角速度 + 线加速度 |
| `SMX/BTAngle` | `std_msgs/Float64MultiArray` | 发布 | Roll、Pitch、Yaw（°）  |

### 2. control\_message\_node

| 输入                         | 输出                                            | 描述                         |
| -------------------------- | --------------------------------------------- | -------------------------- |
| `sensor_msgs/Joy` (`/joy`) | `std_msgs/Float64MultiArray` (`SMX/SportCmd`) | 按键 → 16××××× / 25××××× 控制码 |

详细映射请查看 `control_message/src/control_message_node.cpp`。默认 LT+RT 解锁，RT+左摇杆移动，RT+右摇杆旋转。

### 3. control\_loop\_node

* 订阅：`SMX/BTAngle`、`SMX/GimbalState`、`SMX/Odom`…
* 内部双环：

  1. **IMU 外环** – 计算目标 Pitch/Yaw
  2. **云台内环** – PID 驱动 G1 吊舱，同时给 `SMX/SportCmd` 微量位移补偿

### 4. gimbal\_record\_node

| 数据源              | 文件                  | 说明           |
| ---------------- | ------------------- | ------------ |
| `SMX/Camera_Raw` | `Camera_<time>.mp4` | H.264 硬编实时录制 |
| 各状态话题            | `Msg_<time>.csv`    | 每帧一行同步记录     |

### 5. control\_loop\_node

* 订阅：`/SMX/YOLO_Target` (Float64MultiArray, N×6: 方位角,俯仰角,距离,roll,pitch,置信度)
* 发布：`/SMX/DroneStateEstimate` ((Float64MultiArray, nx=9 状态))
* 广播：`TF map -> uav` (估计状态), `map -> uav_pre` (预测状态)
* 参数：支持score_threshold, g, drag_k, c_int, parent_frame_id, child_frame_id

## ⚙️ 运行示例

```bash
# 1. 蓝牙 IMU
ros2 run bt_imu bt_imu_node

# 2. 云台闭环（读取 IMU & GimbalState）
ros2 run control_loop control_loop_node

# 3. 手柄控制（需先启动 joy_node）
ros2 run control_message control_message_node

# 4. 数据采集
ros2 run gimbal_record gimbal_record_node

# 5. 无人机状态估计
ros2 run drone_estimator drone_estimator_node
```

> `control_message_node` 可选参数文件：`--params-file Ros2Tools/config.yaml`

---

## 🎥 视频演示

| 主题               | 点击图片观看                                                                                                                                |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| 纯里程计建图 (站立/四足切换) | [![img](https://i1.hdslb.com/bfs/archive/4f60453cb37ce5e4f593f03084dbecd0fdddc27e.jpg)](https://www.bilibili.com/video/BV1UtQfYJExu)  |
| 室内行走误差 0.5 %‑1 %     | [![img](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/) |
| 爬楼梯高度误差 < 5 cm      | [![img](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/) |
| 户外行走380m误差 3.3 %     | [![img](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/) |
| 语音交互 + 地图导航        | [![img](https://i2.hdslb.com/bfs/archive/5b95c6eda3b6c9c8e0ba4124c1af9f3da10f39d2.jpg)](https://www.bilibili.com/video/BV1HCQBYUEvk/) |
| 人脸识别跟踪 + 光点跟踪     | [![img](https://i0.hdslb.com/bfs/archive/5496e9d0b40915c62b69701fd1e23af7d6ffe7de.jpg)](https://www.bilibili.com/video/BV1faG1z3EFF/) |
| AR眼镜头部运动跟随         | [![img](https://i1.hdslb.com/bfs/archive/9e0462e12bf77dd9bbe8085d0d809f233256fdbd.jpg)](https://www.bilibili.com/video/BV1pXEdzFECW) |
| YOLO无人机识别与跟随（初测1）| [![img](https://i1.hdslb.com/bfs/archive/9e0462e12bf77dd9bbe8085d0d809f233256fdbd.jpg)](https://www.bilibili.com/video/BV18v8xzJE4G) |
| 多图像源融合估计（初测2）    | [![img](https://i1.hdslb.com/bfs/archive/68fa17f6b90c36137e32dc6553bb66b48c6836ea.jpg)](https://www.bilibili.com/video/BV13we1zEEED/) |
| 多种神经网络位置预测        | [![img](https://i1.hdslb.com/bfs/archive/650062a4aeb28cb7bfdd15e658de1523f537efb7.jpg)](https://www.bilibili.com/video/BV1ytMizEEdG) |

---

## 📄 深入阅读

* 技术原理笔记：[https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22](https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22)
* ROS1 版本参考：[https://github.com/ShineMinxing/FusionEstimation](https://github.com/ShineMinxing/FusionEstimation)

---

## 📨 联系我们

| 邮箱                                          | 单位           |
| ------------------------------------------- | ------------ |
| [401435318@qq.com](mailto:401435318@qq.com) | 中国科学院光电技术研究所 |

> 📌 **本仓库仍在持续开发中** — 欢迎 Issue / PR 交流、贡献！
