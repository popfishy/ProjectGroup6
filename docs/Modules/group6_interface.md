# group6_interface 模块

自定义ROS消息类型定义模块，定义系统内各组件间通信的数据结构。

## 文件结构

```
group6_interface/
├── msg/
│   ├── AgentData.msg        # 单个巡飞弹状态
│   ├── AgentsData.msg       # 巡飞弹数组
│   ├── TargetData.msg       # 单个目标状态
│   ├── TargetsData.msg      # 目标数组
│   ├── RegionData.msg       # 区域定义
│   └── RegionsData.msg      # 区域数组
└── CMakeLists.txt
```

## 消息定义

### AgentData

巡飞弹/无人机状态消息。

```
Header header       # ROS标准头
int32 missile_id    # 巡飞弹ID
float32 x          # X坐标
float32 y          # Y坐标
float32 z          # Z坐标
float32 rot_x      # 姿态四元数 x
float32 rot_y      # 姿态四元数 y
float32 rot_z      # 姿态四元数 z
float32 rot_w      # 姿态四元数 w
float32 missile_flight_speed  # 飞行速度
bool is_destroy    # 是否已损毁
```

### AgentsData

巡飞弹数组消息（容器类型）。

```
AgentData[] agents_data
```

### TargetData

敌方目标状态消息。

```
Header header
int32 target_id    # 目标ID
float32 x          # X坐标
float32 y          # Y坐标
float32 z          # Z坐标
int32 target_type  # 目标类型 (0-7)
bool is_destroy    # 是否已被摧毁
```

### TargetsData

目标数组消息（容器类型）。

```
TargetData[] targets_data
```

### RegionData

作战区域定义消息。

```
Header header
int32 region_id                    # 区域ID
float32[3] position1               # 角点1坐标
float32[3] position2               # 角点2坐标
float32[3] position3               # 角点3坐标
float32[3] position4               # 角点4坐标
```

### RegionsData

区域数组消息（容器类型）。

```
RegionData[] regions_data
```

## 使用方法

### 发布消息

```python
from group6_interface.msg import AgentData, AgentsData

pub = rospy.Publisher('/missile_data', AgentsData, queue_size=10)

agent = AgentData()
agent.missile_id = 1
agent.x = 100.0
agent.y = 200.0
agent.z = 300.0
# ... 设置其他字段

agents_data = AgentsData()
agents_data.agents_data.append(agent)
pub.publish(agents_data)
```

### 订阅消息

```python
from group6_interface.msg import AgentsData

def callback(data):
    for agent in data.agents_data:
        rospy.loginfo(f"巡飞弹 {agent.missile_id} 位置: ({agent.x}, {agent.y}, {agent.z})")

sub = rospy.Subscriber('/missile_data', AgentsData, callback)
```

## Topic 汇总

| Topic | 消息类型 | 方向 | 说明 |
|-------|---------|------|------|
| `/missile_data` | AgentsData | ← 从仿真系统 | 巡飞弹状态 |
| `/target_data` | TargetsData | ← 从仿真系统 | 敌方目标状态 |
| `/region_data` | RegionsData | ← 从仿真系统 | 作战区域定义 |
