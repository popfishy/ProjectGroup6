# 智能集群弹药分布式感知虚拟仿真系统

基于 ROS Noetic 的无人机集群作战仿真平台，实现 OODA（观察-定向-决策-行动）完整打击流程。

## 系统概述

本系统协同管理巡飞弹与固定翼无人机编队，完成区域侦查、目标打击与封控任务。

```mermaid
flowchart LR
    A[准备阶段] --> B[侦查阶段]
    B --> C[打击阶段]
    C --> D[封控阶段]
```

### 核心功能

| 功能 | 说明 |
|------|------|
| 编队控制 | GVF向量场实现多机编队飞行 |
| 区域覆盖 | Fields2Cover扫描覆盖路径规划 |
| 任务分配 | CBPA算法优化多目标分配 |
| 封控巡逻 | 圆填充算法生成封控区域 |

## 快速开始

### 环境要求

- **操作系统**: Ubuntu 20.04 / Windows
- **ROS**: Noetic
- **Python**: 3.8+
- **仿真系统版本**: uav11.17版本

### IP配置

| 节点 | IP地址 |
|------|--------|
| Ubuntu控制端 | `192.168.1.101` |
| Windows仿真端 | `192.168.1.102` |

### 依赖安装

**Ubuntu端**：

```bash
# or-tools
cmake -S. -Bbuild -DBUILD_DEPS:BOOL=ON
cmake --build build && sudo make install

# Fields2Cover
# 见 docs/Development/installation.md
```

### 运行仿真

**1. Ubuntu端启动ROS**：

```bash
roscore
```

**2. 启动数据接收**（在 `communication/scripts/` 目录）：

```bash
python data_recieve.py
```

**3. 配置参数**（编辑 `start_simulation.py`）：

```python
UAV_NUM = 125          # 无人机数量
DAMAGE_RATIO = 0.5     # 损毁比例
IS_RANDOM_DAMAGE = True # 是否随机损毁
```

**4. 启动主仿真**（在项目根目录）：

```bash
python start_simulation.py
```

**5. Windows仿真端**：选择仿真区域（右上角），设置无人机数量，点击开始仿真。

## 项目结构

```
src/
├── start_simulation.py     # 主控制程序
├── communication/          # 数据通信模块
├── region2cover/           # 区域覆盖模块
├── XFD_allocation/          # 任务分配模块
├── group6_interface/        # ROS消息定义
└── docs/                   # 文档
```

### 模块文档

| 模块 | 说明 | 文档 |
|------|------|------|
| `communication/` | 与仿真系统UDP通信 | [docs/Modules/communication.md](docs/Modules/communication.md) |
| `region2cover/` | 路径规划与封控 | [docs/Modules/region2cover.md](docs/Modules/region2cover.md) |
| `XFD_allocation/` | CBPA任务分配 | [docs/Modules/XFD_allocation.md](docs/Modules/XFD_allocation.md) |
| `group6_interface/` | ROS消息类型 | [docs/Modules/group6_interface.md](docs/Modules/group6_interface.md) |

## 系统架构

详见 [系统架构文档](docs/Architecture/system-architecture.md)

## 效能指标

仿真结束后自动计算并写入Excel：

| 指标 | 名称 | 说明 |
|------|------|------|
| O1 | 识别效率 | 目标识别数量/识别时间 |
| D1 | 决策效率 | 加权决策收益 |
| A1 | 打击效率 | 打击目标数/最大打击时间 |

## 文档目录

```
docs/
├── Architecture/
│   ├── system-architecture.md   # 系统架构
│   └── ooda-flow.md            # OODA流程详解
├── Modules/
│   ├── data-models.md          # 数据模型
│   ├── communication.md         # 通信模块
│   ├── region2cover.md          # 区域覆盖模块
│   ├── XFD_allocation.md        # 任务分配模块
│   └── group6_interface.md      # ROS消息模块
└── Development/
    └── installation.md         # 安装指南
```
