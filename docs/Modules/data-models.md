# 数据模型

## Robot (巡飞弹/无人机)

位置: `XFD_allocation/scripts/CBPA/lib/Robot.py`

```python
@dataclass
class Robot:
    robot_id: int           # 唯一标识符
    robot_type: int         # 类型: 0=侦查, 1=打击, 等
    x, y, z: float          # 位置坐标 (米)
    rx, ry, rz, rw: float  # 四元数姿态
    nom_velocity: float     # 巡航速度 (m/s)
    robot_status: bool      # True=存活, False=已损毁
    str_capability: float   # 打击能力
    rec_capability: float  # 侦查能力
```

## Task (敌方目标)

位置: `XFD_allocation/scripts/CBPA/lib/Task.py`

```python
@dataclass
class Task:
    task_id: int           # 唯一标识符
    task_type: int          # 目标类型 (见下表)
    x, y, z: float         # 位置坐标 (米)
    task_status: bool      # True=未摧毁, False=已摧毁
    robot_need: int         # 完成任务所需巡飞弹数量
    str_need: float         # 打击需求
    rec_need: float         # 侦查需求
```

**目标类型对照表**：

| task_type | 含义 | robot_need |
|-----------|------|------------|
| 0 | 高价值目标 | 4 |
| 1 | 中价值目标 | 3 |
| 2 | 防空目标 | 2 |
| 3 | 低价值目标 | 1 |
| 4-7 | 其他类型 | 2-3 |

## Region (作战区域)

位置: `XFD_allocation/scripts/CBPA/lib/Region.py`

```python
class Region:
    region_id: int
    pos1, pos2, pos3, pos4  # 矩形四个角点坐标 [x, y, z]
    xmin, xmax              # X轴边界
    ymin, ymax              # Y轴边界
    center: tuple           # 区域中心点 (x, y)
    area: float             # 区域面积（用于按比例分配无人机）
    uav_id_list: list       # 分配给该区域的无人机ID列表
    task_id_list: list      # 该区域内的目标ID列表
```

**区域定义示意**：

```
pos1 ------------- pos4
  |                 |
  |    region_id    |
  |                 |
pos2 ------------- pos3
```

## WorldInfo (世界边界)

位置: `XFD_allocation/scripts/CBPA/lib/WorldInfo.py`

```python
@dataclass
class WorldInfo:
    limit_x: list  # [min, max]
    limit_y: list  # [min, max]
    limit_z: list  # [min, max]
```

## ROS 消息类型

位置: `group6_interface/msg/`

### AgentData (巡飞弹状态)

| 字段 | 类型 | 说明 |
|------|------|------|
| missile_id | int32 | 巡飞弹ID |
| x, y, z | float32 | 位置 |
| rot_x, rot_y, rot_z, rot_w | float32 | 四元数姿态 |
| missile_flight_speed | float32 | 飞行速度 |
| is_destroy | bool | 是否已损毁 |

### TargetData (敌方目标)

| 字段 | 类型 | 说明 |
|------|------|------|
| target_id | int32 | 目标ID |
| x, y, z | float32 | 位置 |
| target_type | int32 | 目标类型 (0-7) |
| is_destroy | bool | 是否已被摧毁 |

### RegionData (作战区域)

| 字段 | 类型 | 说明 |
|------|------|------|
| region_id | int32 | 区域ID |
| position1-4 | float32[3] | 四个角点坐标 |

## 全局状态变量

位置: `start_simulation.py`

| 变量 | 类型 | 说明 |
|------|------|------|
| `RobotList` | list[Robot] | 当前存活巡飞弹列表 |
| `TaskList` | list[Task] | 当前敌方目标列表 |
| `RegionList` | list[Region] | 作战区域列表 |
| `TaskDefenceList` | list[Task] | 防空目标列表（优先级打击） |
| `AttckRobotSet` | set | 已分配打击任务的巡飞弹ID |
| `DestoryRobotSet` | set | 已损毁的巡飞弹ID |

## 时间记录变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `Detect_step_start_time` | rospy.Time | 侦查阶段开始时间 |
| `Detect_step_end_time_map` | dict | 各区域侦查完成时间 |
| `Cover_step_end_time_map` | dict | 各区域覆盖完成时间 |
| `AttckStartTime` | dict | 各巡飞弹开始打击时间 |
| `AttckEndTime` | dict | 各巡飞弹完成打击时间 |
| `AttackTargetStartTime` | dict | 各目标开始被打击时间 |
| `AttackTargetEndTime` | dict | 各目标被摧毁时间 |
