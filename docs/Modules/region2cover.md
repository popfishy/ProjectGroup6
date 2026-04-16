# region2cover 模块

区域覆盖与封控路径规划模块，提供区域扫描覆盖和封控圆巡逻功能。

## 概述

| 依赖库 | 用途 |
|--------|------|
| Fields2Cover | 覆盖路径规划（扫描线、Dubins曲线） |
| dubins | 最短路径计算 |

## 文件结构

```
region2cover/
├── region_cover.py      # 区域覆盖路径规划（Fields2Cover）
├── region_isolation.py  # 封控圆生成与巡逻路径
├── dubins_generate.py   # Dubins路径工具
└── __init__.py
```

## RegionCover

区域覆盖路径规划器，使用 Fields2Cover 库生成扫描覆盖路径。

**位置**：`region_cover.py`

### 初始化

```python
RegionCover(num_rows, num_cols, pos1, pos2, pos3, pos4, is_plot=False)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| num_rows | int | 区域行分块数 |
| num_cols | int | 区域列分块数 |
| pos1-pos4 | list | 矩形区域四个角点坐标 [x, y, z] |
| is_plot | bool | 是否绘制路径图 |

### 关键方法

| 方法 | 说明 |
|------|------|
| `cover_run(log_time, cov_width)` | 生成分块覆盖路径 |
| `start_point_list` | 各分块起点列表 |
| `all_path` | 所有覆盖路径段 |

### 使用示例

```python
region_cover = RegionCover(
    num_rows=1,
    num_cols=1,
    pos1=[8081, 0, 3752],
    pos2=[8081, 0, 2802],
    pos3=[9519, 0, 2802],
    pos4=[9519, 0, 3752],
    is_plot=True
)
region_cover.cover_run(log_time, cov_width=100)
```

## generate_circles_in_rectangle

圆填充算法，在矩形区域内生成等半径圆的最佳分布。

**位置**：`region_isolation.py`

### 函数签名

```python
circle_radius, circle_centers = generate_circles_in_rectangle(
    robot_alive_num,    # 可用无人机数量
    length,             # 矩形长度
    width,              # 矩形宽度
    center_x,           # 区域中心X
    center_y            # 区域中心Y
)
```

### 返回值

| 返回值 | 类型 | 说明 |
|--------|------|------|
| circle_radius | float | 封控圆半径 |
| circle_centers | list | 各圆圆心坐标 `[[x, y], ...]` |

## UavPath

无人机封控路径生成器，为每架无人机生成：
1. 从当前位置到目标圆的 Dubins 路径
2. 圆上巡逻路径

**位置**：`region_isolation.py`

### 关键属性

| 属性 | 类型 | 说明 |
|------|------|------|
| dubins_path | list | Dubins路径点 `[[x, y, theta], ...]` |
| circle_path | list | 圆巡逻路径点 |
| dubins_cnt | int | 当前Dubins路径进度 |
| circle_cnt | int | 当前圆巡逻进度 |
| is_dubins_finish | bool | Dubins阶段是否完成 |

### 关键方法

```python
isolate_path.calculate_path(
    start_x,        # 起点X
    start_y,        # 起点Y
    start_theta,    # 起点朝向角
    center_x,       # 圆心X
    center_y,       # 圆心Y
    circle_radius   # 圆半径
)
```

## dubins_generate.py

Dubins路径工具函数，封装了dubins库的使用。

### 主要函数

| 函数 | 说明 |
|------|------|
| `dubins_path_single()` | 生成单架无人机Dubins路径 |
| `dubins_path_multi()` | 生成多架无人机Dubins路径 |
