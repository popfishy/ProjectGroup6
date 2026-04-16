# 系统架构文档

## 1. 系统概述

智能集群弹药分布式感知虚拟仿真系统实现了完整的 OODA（Observe-Orient-Decide-Act）打击任务流程，协同管理巡飞弹与固定翼无人机编队完成区域封控作战任务。

## 2. 系统架构图

```mermaid
graph TB
    subgraph Windows仿真端["Windows 仿真系统 (192.168.1.102)"]
        Sim[仿真软件]
    end

    subgraph Ubuntu控制端["Ubuntu 控制节点 (192.168.1.101)"]
        comm[communication<br/>数据通信模块]
        rosbridge[ROS Topics<br/>missile_data/target_data]
        main[start_simulation.py<br/>主控模块]
        region2cover[region2cover<br/>路径规划模块]
        XFD_alloc[XFD_allocation<br/>任务分配模块]
    end

    Sim <-->|UDP 12380| comm
    comm --> rosbridge
    rosbridge --> main
    main --> region2cover
    main --> XFD_alloc
    main --> comm
    comm --> Sim
```

## 3. 模块职责

| 模块 | 职责 | 关键接口 |
|------|------|----------|
| `communication/` | 与Windows仿真系统UDP双向通信，收发巡飞弹状态与控制指令 | ROS Topics: `/missile_data`, `/target_data`<br/>UDP: `192.168.1.102:12380` |
| `region2cover/` | 区域覆盖路径规划、封控圆生成 | `RegionCover`, `UavPath`, `generate_circles_in_rectangle()` |
| `XFD_allocation/` | CBPA算法任务分配 | `CBPA_REC.solve_centralized()` |
| `group6_interface/` | ROS消息类型定义 | `AgentData`, `TargetData`, `RegionData` 等 |
| `start_simulation.py` | OODA流程编排、主控制逻辑 | - |

## 4. OODA流程映射

```mermaid
flowchart LR
    subgraph 准备阶段["① 准备阶段"]
        prep[编队分组分簇]
    end

    subgraph 侦查阶段["② 侦查阶段"]
        recon[覆盖式侦查]
        detect[目标检测]
    end

    subgraph 打击阶段["③ 打击阶段"]
        alloc[CBPA任务分配]
        attack[执行打击]
    end

    subgraph 封控阶段["④ 封控阶段"]
        circle[生成封控圆]
        isolate[区域封控巡逻]
    end

    prep --> recon --> detect --> alloc --> attack --> circle --> isolate
```

### 4.1 各阶段代码位置

| 阶段 | 主函数/类 | 文件位置 |
|------|----------|----------|
| 准备阶段 | `divide_robot()`, `generate_gvf_ode_set()`, `generate_region_conver_set()` | `start_simulation.py` |
| 侦查阶段 | `UavSimProcess.detect_step()` | `start_simulation.py:149-301` |
| 打击阶段 | `UavSimProcess.attack_step()` | `start_simulation.py:303-337` |
| 封控阶段 | `UavSimProcess.isolate_step()` | `start_simulation.py:339-515` |

## 5. 数据流

```mermaid
sequenceDiagram
    participant Sim as Windows仿真
    participant Comm as communication
    participant ROS as ROS Topics
    participant Main as start_simulation
    participant Alloc as XFD_allocation
    participant Path as region2cover

    Sim->>Comm: UDP: 巡飞弹状态
    Sim->>Comm: UDP: 目标状态
    Comm->>ROS: /missile_data
    Comm->>ROS: /target_data
    ROS->>Main: 订阅数据
    Main->>Path: 请求路径规划
    Main->>Alloc: 请求任务分配
    Main->>Comm: UDP: 位置/打击/损毁指令
    Comm->>Sim: 发送指令
```

## 6. 关键数据模型

参见 [数据模型](../Modules/data-models.md)

## 7. 外部依赖

| 依赖 | 版本 | 用途 |
|------|------|------|
| ROS Noetic | - | 中间件、节点管理 |
| fields2cover | - | 覆盖路径规划 |
| or-tools | - | 优化算法 |
| numpy | - | 数值计算 |
| scipy | - | GVF常微分方程求解 |
