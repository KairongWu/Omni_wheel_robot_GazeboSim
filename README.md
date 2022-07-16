[TOC]

# 三轮全向底盘 Gazbeo 仿真（全向轮-omni）

<br/>

## Write at the beginning：

该仓库项目代码原作者为：robot mania

该仓库仅为本人的学习记录仓库



## How to run 

第一步 场景模型放置：

由于 Gazebo 仿真场景中使用了额外的模型 omni_stage，不包含在 Gazebo 原有模型中

为保证仿真的正常运行，请将文件中的 <font color=red>**omni_stage**</font> 文件拷贝至 <font color=red>**.gazebo/models**</font> 目录下

<br/>

第二步 构建工作空间：

基于如下功能包构建工作空间

- commander
- omni_robot_control
- omni_robot_description
- omni_robot_gazebo

```bash
$ mdkir -p ~/catkin_ws/src
# 拷贝上述功能包至工作空间 src 目录下
# 在 catkin_ws 目录下进行编译
$ catkin_make
```

<br/>

第三步 启动仿真：

如下命令在工作空间 catkin_ws 目录下进行

```bash
$ source devel/setup.bash
$ roslaunch omni_robot_gazebo omni_world.launch
```



# Thanks

Original author: robot mania

此处特别感谢原作者提供的全向轮仿真 Demo

