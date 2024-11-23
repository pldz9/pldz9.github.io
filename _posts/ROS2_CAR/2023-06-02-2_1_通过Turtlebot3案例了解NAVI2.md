---
title: 2 通过 Turtlebot3 案例了解 NAVI2
category: ROS2_CAR
layout: post
---


# 1.1 Ubuntu20.04 安装 ROS2(Foxy)

> 参考内容：
>
> [官网给出的教程](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

1. 换源，输入`sudo gedit /etc/apt/sources.list`这里选择的是中科大的源，如果你是在 Ubuntu 内部选择的软件源，笔者遇到过安装 ROS2 时候提示没有 depends 的情况，于是这里手动换源，将文件源更改如下：

```shell
deb https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-proposed main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal-proposed main restricted universe multiverse
```

2. 更新源和软件：`sudo apt-get update && sudo apt-get upgrade`

3. 配置语言，需要支撑`UTF-8`：

```shell
pldz@pldz-pc:~$ locale
LANG=en_US.UTF-8
LANGUAGE=
LC_CTYPE="en_US.UTF-8"
LC_NUMERIC=zh_CN.UTF-8
LC_TIME=zh_CN.UTF-8
LC_COLLATE="en_US.UTF-8"
LC_MONETARY=zh_CN.UTF-8
LC_MESSAGES="en_US.UTF-8"
LC_PAPER=zh_CN.UTF-8
LC_NAME=zh_CN.UTF-8
LC_ADDRESS=zh_CN.UTF-8
LC_TELEPHONE=zh_CN.UTF-8
LC_MEASUREMENT=zh_CN.UTF-8
LC_IDENTIFICATION=zh_CN.UTF-8
LC_ALL=
pldz@pldz-pc:~$
```

4. 获取 GPG 秘钥：`sudo apt update && sudo apt install curl -y`然后`sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`，此时如果提示无法访问到网站，则添加 host：

- 1. 访问 `https://tool.lu/ip/` 并输入域名 `raw.githubusercontent.com`，查询 ip 地址，这里查询到的是 `185.199.108.133`

![IP地址查询](/assets/pics/ROS2_CAR/2_get_ip_address.png)

- 2. 修改 `sudo gedit /etc/hosts`文件,并手动添加 DNS 解析：

![手动添加IP](/assets/pics/ROS2_CAR/2_add_ip_into_linux_host.png)

5. 添加 ROS2 的仓库：`echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`

6. 更新源和软件：`sudo apt-get update && sudo apt-get upgrade`

7. 下载 ROS2 Foxy：`sudo apt install ros-foxy-desktop python3-argcomplete`

8. 下载完成后下载 ROS2 开发工具：`sudo apt install ros-dev-tools`

9. 将 ROS2 添加到环境变量：`sudo echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc`

10. 激活环境变量验证安装：`source ~/.bashrc`，然后输入`ros2 run turtlesim turtlesim_node`如果能够看到小乌龟则代表成功

![ROS2 小乌龟例子](/assets/pics/ROS2_CAR/2_ros2_turtlesim_demo.png)

# 1.2 运行 ROS2 NAVI 例子

## 1.2.1 安装仿真的依赖项和软件

> 参考内容
>
> [[ROS2 基础] Navigation2 导航系统](https://fishros.org.cn/forum/topic/303/ros2-%E5%9F%BA%E7%A1%80-navigation2%E5%AF%BC%E8%88%AA%E7%B3%BB%E7%BB%9F)
>
> [《动手学 ROS2》10.10 通过 Nav2API 进行导航](https://zhuanlan.zhihu.com/p/526385552)
>
> [ROS2 与 Navigation2 入门教程-构建和安装 Nav2](https://www.ncnynl.com/archives/202110/4702.html)

1. 下载功能包：需要的功能包包括`turtlebot3`的模型和`cartographer`,以及`rviz2`和`gazebo`环境，其中的`gazebo`主要用于模拟真实的物理环境，`rviz2`是`ROS2`通讯的可视化工具

```shell
sudo apt install ros-foxy-turtlebot3*
sudo apt install ros-foxy-cartographer

# 如果没有gazebo和rviz2的需要再次安装这两个软件
sudo apt install ros-foxy-rviz2
sudo apt install ros-foxy-gazebo*
```

3. 在运行仿真环境的时候，有时候会卡住在 gazebo 打开的界面，gazebo 初始化打开时候是会连接网络下载模型库,否则可能会出现**打开了环境，但是机器人一直在缩小的情况**

4. 到 gazebo 的 github 官方网页: [osrf/gazebo_models](https://github.com/osrf/gazebo_models)，下载全部模型为 zip 文件

![gazebo模型仓库](/assets/pics/ROS2_CAR/2_gazebo_models_repo.png)

5. 解压 zip 文件夹并重命名为`models`，然后拷贝到`~/.gazebo`的目录下

![移动gazebo模型](/assets/pics/ROS2_CAR/2_put_gazebo_models.png)

## 1.2.2 建立导航的地图(Mapping)

1. 启动导航的物理仿真环境：创建一个启动 gazebo 物理仿真环境的脚本，比方说这里的`naviWorld.sh`（赋予可执行权限）

```shell
# naviWorld.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

![启动trutlebot3的gazebo物理环境](/assets/pics/ROS2_CAR/2_run_gazebo.png)

第一次启动，可能会出现一直在等待的状态，并且提示`Spawn service failed. Exiting.`， 错误的解决办法可以参考 1.3.1 的内容，杀死 gazebo 再次启动即可

2. 启动建图指令：创建一个`RVIZ2`环境用于显示小车的状态，并建立起导航的地图，例如这里的`mapping.sh`（赋予可执行权限）：

```shell
# mapping.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

启动成功之后，会出现 rviz2 的界面：

![建图的rviz2画面](/assets/pics/ROS2_CAR/2_gazebo_rviz2_map.png)

3. 控制机器人移动生成地图文件，输入命令：`ros2 run turtlebot3_teleop teleop_keyboard`启动键盘控制小车运动的节点，在此之前可以指定小车的类型：`export TURTLEBOT3_MODEL=burger`，例如下面的 telep.sh

```shell
# telep.sh
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

![控制机器人运动生成大概的地图文件](/assets/pics/ROS2_CAR/2_generate_map.png)

4. 完成建图，生成导航的地图文件和信息，通过`teleop_keyboard`打开的终端控制机器人移动，等待图片大部分被扫描成功后，输入命令行调用`map_server`存储地图，其中的地图名称为 test，之后会生成以该名字命名的`.pgm`/`yaml`等文件：`ros2 run nav2_map_server map_saver_cli -f testMap`

```shell
pldz@pldz-pc:/mnt/hgfs/VMware/ROS2_NAVI/1_Chapter/code$ ros2 run nav2_map_server map_saver_cli -f testMap
[INFO] [1684547384.636100069] [map_saver]:
	map_saver lifecycle node launched.
	Waiting on external lifecycle transitions to activate
	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [1684547384.636296866] [map_saver]: Creating
[INFO] [1684547384.636480318] [map_saver]: Configuring
[INFO] [1684547384.638094058] [map_saver]: Saving map from 'map' topic to 'testMap' file
[WARN] [1684547384.638189556] [map_saver]: Free threshold unspecified. Setting it to default value: 0.250000
[WARN] [1684547384.638247635] [map_saver]: Occupied threshold unspecified. Setting it to default value: 0.650000
[WARN] [map_io]: Image format unspecified. Setting it to: pgm
[INFO] [map_io]: Received a 125 X 119 map @ 0.05 m/pix
[INFO] [map_io]: Writing map occupancy data to testMap.pgm
[INFO] [map_io]: Writing map metadata to testMap.yaml
[INFO] [map_io]: Map saved
[INFO] [1684547384.984216612] [map_saver]: Map saved successfully
[INFO] [1684547384.985701952] [map_saver]: Destroying
pldz@pldz-pc:/mnt/hgfs/VMware/ROS2_NAVI/1_Chapter/code$ ls
mapping.sh  naviWorld.sh  simNavi2.sh  telep.sh  testMap.pgm  testMap.yaml
```

## 1.2.3 使用地图文件进行导航

1. 当有了地图文件之后，就可以根据地图文件，进行小车的导航仿真，这里启动`turtlebot3`的物理仿真环境，也就是建图时候的`gazebo`，实际上也就是之前的`naviWorld.sh`：

```shell
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. 有了物理仿真环境之后，再加载地图进行导航的仿真，**其中的`map:`后面的参数为之前的`map_server`生成的地图文件位置**，创建导航的脚本`simNavi2.sh`，此时会提示`Frame map does not exit.`是因为没有给出起始的位置建立`odom`和`map`直接的计算:

```shell
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=testMap.yaml
```

![启动RVIZ导航](/assets/pics/ROS2_CAR/2_run_navi.png)

3. 给出起始点：在`rviz2`工具上拖动`2D Pose Estimate`，给出小车位置，完成坐标的变换：

![RVIZ2启动导航仿真](/assets/pics/ROS2_CAR/2_navi_demo.gif)

## 1.2.4 可能出现的问题汇总（持续更新）

## 1.2.4.1 ROS2 gazebo：Spawn service failed. Exiting.

- 错误的提示如下：

```shell
[spawn_entity.py-4] [INFO] [1684419460.753634677] [spawn_entity]: Loading entity XML from file /opt/ros/foxy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
[spawn_entity.py-4] [INFO] [1684419460.755174212] [spawn_entity]: Waiting for service /spawn_entity, timeout = 30
[spawn_entity.py-4] [INFO] [1684419460.755469679] [spawn_entity]: Waiting for service /spawn_entity
[spawn_entity.py-4] [ERROR] [1684419490.844590431] [spawn_entity]: Service %s/spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?
[spawn_entity.py-4] [ERROR] [1684419490.844908061] [spawn_entity]: Spawn service failed. Exiting.
[ERROR] [spawn_entity.py-4]: process has died [pid 8128, exit code 1, cmd '/opt/ros/foxy/lib/gazebo_ros/spawn_entity.py -entity burger -file /opt/ros/foxy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf -x -2.0 -y -0.5 -z 0.01 --ros-args'].
```

- 尝试解决办法：

1. 杀死全部的`gazebo server`在命令行输入：`lsof -i`查看是否还有在运行的`gzserver`，记录程序的 PID 号，对输入`kill -9 <pid号>`杀死程序

2. 再次输入脚本指令打开`gazebo`

## 1.2.4.2 [map_saver]: Failed to spin map subscription

- 错误的提示如下：

```shell
pldz@pldz-pc:/mnt/hgfs/VMware/ROS2_NAVI/1_Chapter/code$ rerver map_saver_cli -os2 run nav2_map_server map_saver_cli -f testMap
[INFO] [1684547337.249208762] [map_saver]:
	map_saver lifecycle node launched.
	Waiting on external lifecycle transitions to activate
	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [1684547337.249693695] [map_saver]: Creating
[INFO] [1684547337.250079483] [map_saver]: Configuring
[INFO] [1684547337.256994454] [map_saver]: Saving map from 'map' topic to 'testMap' file
[WARN] [1684547337.257098278] [map_saver]: Free threshold unspecified. Setting it to default value: 0.250000
[WARN] [1684547337.257179799] [map_saver]: Occupied threshold unspecified. Setting it to default value: 0.650000
[ERROR] [1684547339.261336746] [map_saver]: Failed to spin map subscription
[INFO] [1684547339.262984556] [map_saver]: Destroying
```

- 尝试解决办法，造成的原因是因为`map_server`没有办法找到`/map`的话题来订阅，可能是之前你在打开`RVIZ2`或者其他内容时，没有启动完全，尝试退出终端，再次启动，其中之后输入`ros2 topic list | grep map`查看是否有`/map`的话题，有则可以再次输入保存地图文件的命令。
