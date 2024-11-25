## README

### 一.安装和使用

#### 安装

仿真系统版本:uav11.17版本   环境:windows环境   IP地址设置为:192.168.1.102 

ubuntu代码测试: IP地址设置为:192.168.1.101 



ubuntu端需要安装:or-tools和Fields2Cover包. 其中or-tools安装:

```
# -DBUILD_DEPS:BOOL=ON 一定要加上，否则很多依赖库不会编译
cmake -S. -Bbuild  -DBUILD_DEPS:BOOL=ON
cmake --build build
sudo make install
```

Fields2Cover包需要修改Fields2Cover/src/fields2cover/path_planning/path_planning.cpp文件.

提供的Fields2Cover已经修改完毕,安装:

```
sudo apt-get update
sudo apt-get install --no-install-recommends software-properties-common
sudo add-apt-repository ppa:ubuntugis/ppa
sudo apt-get update
sudo apt-get install --no-install-recommends build-essential ca-certificates cmake \
     doxygen g++ git libeigen3-dev libgdal-dev libpython3-dev python3 python3-pip \
     python3-matplotlib python3-tk lcov libgtest-dev libtbb-dev swig libgeos-dev \
     gnuplot libtinyxml2-dev nlohmann-json3-dev
python3 -m pip install gcovr


cd build;
cmake -DBUILD_PYTHON=ON ..;
make -j$(nproc);
sudo make install;
```



#### 使用



仿真系统端点击仿真参数,只需要修改无人机数量,用来匹配代码中设置的数量.其他无需修改.

仿真区域选择右上角.点击开始仿真即可.



Ubuntu端,终端运行roscore；进入/ProjectGroup6/src/communication/scripts,运行 python data_recieve.py ；修改mian.py中的参数,需要修改的为

1. UAV_NUM,即无人机数量  
2. DAMAGE_DATIO:即损毁比例
3. IS_RANDOM_DAMAGE:是否随机损毁. 不开启随机损毁的时候请设置为Flase

进入/ProjectGroup6/src,运行python main.py 进行仿真

![image-20241120205710439](/home/yjq/.config/Typora/typora-user-images/image-20241120205710439.png)



