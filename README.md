# 1.简介
   此仓库为HIROP的软件平台，包含华数机器人部分机械臂的URDF模型、moveit配置、gazebo配置、ROS-I驱动包、夹爪驱动包、rviz控制面板、抓取Demo、抓取仿真环境、基础Demo等示例开发包。

   此平台遵循BSD开源协议。

   支持indigo、kinetic版本。如需下载kinetic版本，请将分支切换至kinetic分支。

   维护者: Kunlin Xu(1125290220@qq.com)


# 2.准备工作

## 2.1 环境安装
   请先对ROS环境进行安装，支持indigo以及kinetic版本。

   ROS相关教程可参考：[完整的安装教程](https://github.com/HSRobot/hsr_irp/blob/master/ROS-install.md)

## 2.2安装gazebo模型
   由于indigo版本的gazebo模型源地址更换，所以无法自动获取完整的gazebo模型，按照以下步骤进行离线安装：

   1.从[此链接](https://bitbucket.org/osrf/gazebo_models/downloads/)处下载完整的gazebo模型压缩包

   2.将模型解压至 ~/.gazebo/modles 文件夹中

# 3.实验之前说明
   HIROP平台支持多型号机械臂，因此以下所有实验均使用***br606***机械臂作为示例说明。若用户想进行其它支持的机械臂的仿真，则只需将指令中的br606替换成对应型号即可。

# 4.HIROP的基础使用教程

## 4.1 加载机械臂模型
   ```bash
   $ roslaunch hsr_description br606_upload.launch
   ```

## 4.2 在rviz中显示模型
   ```bash
   $ roslaunch hsr_description display.launch
   ```

## 4.3 启动仿真环境

### 4.3.1 一键启动：	
   ```bash
   $ roslaunch hsr_bringup br606_sim.launch
   ```

### 4.3.2 单独启动（推荐）:
   ```bash
   $ roslaunch hsr_description br606_upload.launch 
   $ roslaunch hsr_gazebo br606_gazebo.launch
   $ roslaunch br606_moveit_config hsr_br606_moveit_planning_execution.launch sim:=true
   $ roslaunch br606_moveit_config moveit_rviz.launch
   ```

## 4.4 使用gazebo中的相机
   ```bash
   $ roslaunch hsr_description br606_with_kinect_upload.launch
   $ roslaunch hsr_gazebo robot_gazebo.launch
   ```

## 4.5 虚拟抓取
   ```bash
   $ roslaunch hsr_description br606_with_gripper_upload.launch
   $ roslaunch br606_moveit_config hsr_br606_moveit_planning_execution.launch sim:=true
   $ roslaunch br606_moveit_config moveit_rviz.launch
   $ rosrun hsr_demo moveit_pick_and_place_demo.py
   ```

## 4.6 连接实际的机器人

### 4.6.1 一键启动：	
   ```bash
   $ roslaunch hsr_bringup br606_go.launch robot_ip:=10.10.56.214
   ```

### 4.6.2 单独启动（推荐）：
   ```bash
   $ roslaunch hsr_description br606_upload.launch 
   $ roslaunch br606_moveit_config br606_moveit_planning_execution.launch sim:=false
   $ roslaunch hsr_rois_client robot_full_interface_download.launch robot_ip:=10.10.56.214
   $ roslaunch br606_moveit_config moveit_rviz.launch
   ```

## 4.7 启动pick（已知物体位姿）
   ```bash
   $ roslaunch hsr_description br606_with_gripper_upload.launch
   $ roslaunch br606_moveit_config br606_moveit_planning_execution.launch sim:=false
   $ roslaunch hsr_rosi_client robot_full_interface_download.launch robot_ip:=10.10.56.214
   $ rosrun hsr_gripper_driver gripper_control_srv
   $ rosrun hsr_gripper_driver gripper_action.py
   $ rosrun hsr_pick moveit_pick_and_place_demo.py
   ```

## 4.8 启动pick and ork（前提需要对相机进行标定）
   ```bash
   $ roslaunch kinect2_bridge  kinect2_bridge.launch
   $ roslaunch hsr_description br606_with_gripper_upload.launch
   $ roslaunch br606_moveit_config br606_moveit_planning_execution.launch sim:=false
   $ roslaunch hsr_rosi_client robot_full_interface_download.launch robot_ip:=10.10.56.214
   $ rosrun hsr_gripper_driver gripper_control_srv
   $ rosrun hsr_gripper_driver gripper_action.py
   $ roslaunch hsr_bringup ork_base_tf.launch
   $ rosrun hsr_pick tf_camera_to_base.py
   $ rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork	
   $ rosrun hsr_pick moveit_pick_and_place_demo.py
   ```

## 4.9 启动pick and ork (RVIZ GUI)
   ```bash
   $ roslaunch hsr_bringup pick_gui.launch
   ```

