# 安装指南

## 系统要求

| 项目 | 版本要求 |
|------|----------|
| 操作系统 | Ubuntu 20.04 LTS |
| ROS | Noetic |
| Python | 3.8+ |
| CMake | 3.10+ |

## IP配置

运行前需正确配置网络IP：

| 节点 | IP地址 | 说明 |
|------|--------|------|
| Ubuntu控制端 | `192.168.1.101` | 本机 |
| Windows仿真端 | `192.168.1.102` | 仿真软件 |

## ROS依赖

确保已安装 ROS Noetic Desktop Full：

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```

## 外部依赖安装

### 1. or-tools

OR-Tools 是 Google 的运筹优化库，用于任务分配算法。

```bash
# 克隆仓库
git clone https://github.com/google/or-tools.git
cd or-tools

# 编译（必须加 -DBUILD_DEPS:BOOL=ON）
cmake -S. -Bbuild -DBUILD_DEPS:BOOL=ON
cmake --build build -j$(nproc)
sudo make install
```

### 2. Fields2Cover

Fields2Cover 是农业机器人覆盖路径规划库，本项目用于生成区域扫描路径。

```bash
# 安装系统依赖
sudo apt-get update
sudo apt-get install --no-install-recommends software-properties-common
sudo add-apt-repository ppa:ubuntugis/ppa
sudo apt-get update
sudo apt-get install --no-install-recommends build-essential ca-certificates cmake \
     doxygen g++ git libeigen3-dev libgdal-dev libpython3-dev python3 python3-pip \
     python3-matplotlib python3-tk lcov libgtest-dev libtbb-dev swig libgeos-dev \
     gnuplot libtinyxml2-dev nlohmann-json3-dev

# 安装Python测试覆盖工具
python3 -m pip install gcovr

# 编译Fields2Cover
cd /path/to/fields2cover
mkdir build && cd build
cmake -DBUILD_PYTHON=ON ..
make -j$(nproc)
sudo make install

# 更新库缓存
sudo ldconfig
```

### 3. Python依赖

```bash
pip install numpy scipy matplotlib networkx
```

## 源码编译

将项目克隆到 ROS 工作空间：

```bash
cd ~/catkin_ws/src
git clone https://github.com/your-repo/ProjectGroup6.git
cd ..
catkin_make
source devel/setup.bash
```

## 安装验证

```bash
# 检查ROS消息是否可用
rosmsg show AgentData
rosmsg show TargetData

# 检查Python模块导入
python3 -c "from region2cover.region_cover import RegionCover"
python3 -c "from XFD_allocation.scripts.CBPA.lib.CBPA_REC import CBPA_REC"
```
