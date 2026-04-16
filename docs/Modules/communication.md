# communication 模块

数据通信模块，负责 Ubuntu 控制端与 Windows 仿真系统之间的 UDP 双向通信。

## 概述

| 项目 | 说明 |
|------|------|
| 协议 | UDP |
| Windows端地址 | `192.168.1.102:12380` |
| Ubuntu端地址 | `192.168.1.101` |

## 文件结构

```
communication/
├── scripts/
│   ├── data_recieve.py   # UDP接收器 → ROS Topics
│   ├── data_send.py      # UDP发送器（预留）
│   └── GVF_ode.py        # GVF向量场编队控制
├── msg/                  # ROS消息定义
└── CMakeLists.txt
```

## 核心功能

### data_recieve.py

接收 Windows 仿真系统发送的 UDP 数据包，解析后发布到 ROS Topics。

**ROS Topics 输出**：

| Topic | 消息类型 | 数据内容 |
|-------|---------|----------|
| `/missile_data` | `AgentsData` | 巡飞弹状态（位置、姿态、速度、损毁状态） |
| `/target_data` | `TargetsData` | 敌方目标状态（位置、类型、摧毁状态） |
| `/region_data` | `RegionsData` | 作战区域定义 |

**数据流向**：

```
Windows仿真 → UDP → data_recieve.py → ROS Topics → start_simulation.py
```

### UDP 数据包格式

**巡飞弹/目标数据**：

```python
struct.pack("i3f4f", missile_id, x, y, z, rx, ry, rz, rw)
# 或
struct.pack("i3f", target_id, x, y, z)
```

**控制指令**：

| Command | 功能 | 数据格式 |
|---------|------|----------|
| 101 | 发送无人机位置 | `(uav_id, x, y, z, rx, ry, rz, rw)` × N |
| 5 | 打击指令 | `(uav_id, target_id)` × N |
| 6 | 损毁指令 | `[uav_id, ...]` |

## GVF_ode.py

广义向量场（Generalized Vector Field）编队控制，用于无人机编队飞行。

**主要类**：`GVF_ode`

**关键方法**：

| 方法 | 说明 |
|------|------|
| `update_waypoint()` | 设置航点（起点、终点） |
| `calculate_path()` | 计算编队路径 |
| `cnt` | 当前路径点计数器 |
| `is_finish_pub` | 是否完成发布 |

**向量化编队示例**：

```python
gvf_ode = GVF_ode(leader_id, uav_num, x_coords, y_coords, x_init, y_init, z_init)
gvf_ode.update_waypoint(trajectory_list)
gvf_ode.calculate_path()
```
