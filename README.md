## 加载模型
   ```bash
   $ roslaunch hsr_description <robot>_upload.launch
   ```
## 显示模型
   ```bash
   $ roslaunch hsr_description display.launch
   ```

## 启动仿真环境
#### 1, 一键启动：	
   ```bash
   $ roslaunch hsr_bringup <robot>_sim.launch
   ```
#### 2, 单独启动（推荐）:
   ```bash
   $ roslaunch hsr_description <robot>_upload.launch 
   $ roslaunch hsr_gazebo <robot>_gazebo.launch
   $ roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=true
   $ roslaunch co602_moveit_config moveit_rviz.launch
   ```

## 使用实际机器人
#### 1，一键启动：	
   ```bash
   $ roslaunch hsr_bringup <robot>_go.launch robot_ip:=<robot_ip>
   ```
#### 2，单独启动（推荐）：
   ```bash
   $ roslaunch hsr_description <robot>_upload.launch 
   $ roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=false
   $ roslaunch industrial_robot_client robot_full_interface_download.launch robot_ip:=<robot_ip>
   $ roslaunch <robot>_moveit_config moveit_rviz.launch
   ```

## 使用gazebo中的相机
   ```bash
   $ roslaunch hsr_description <robot>_with_kinect_upload.launch
   $ roslaunch hsr_gazebo robot_gazebo.launch
   ```

## 启动pick 虚拟
   ```bash
   $ roslaunch hsr_description co602_with_gripper_upload.launch
   $ roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=true
   $ roslaunch <robot>_moveit_config moveit_rviz.launch
   $ rosrun hsr_pick moveit_pick_and_place_demo.py
   ```


## 启动pick（已知物体位姿）
   ```bash
   $ roslaunch hsr_description co602_with_gripper_upload.launch
   $ roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=false
   $ roslaunch industrial_robot_client robot_full_interface_download.launch robot_ip:=<robot_ip> 
   $ rosrun hsr_gripper_driver gripper_control_srv
   $ rosrun hsr_gripper_driver gripper_action.py
   $ rosrun hsr_pick moveit_pick_and_place_demo.py
   ```

## 启动pick and ork（前提需要对相机进行标定）
   ```bash
   $ roslaunch kinect2_bridge  kinect2_bridge.launch
   $ roslaunch hsr_description co602_with_gripper_upload.launch
   $ roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=false
   $ roslaunch industrial_robot_client robot_full_interface_download.launch robot_ip:=<robot_ip> 
   $ rosrun hsr_gripper_driver gripper_control_srv
   $ rosrun hsr_gripper_driver gripper_action.py
   $ roslaunch hsr_bringup ork_base_tf.launch
   $ rosrun hsr_pick tf_camera_to_base.py
   $ rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork	
   $ rosrun hsr_pick moveit_pick_and_place_demo.py
   ```


#### 启动pick and ork (RVIZ GUI)
   ```bash
   $ roslaunch hsr_bringup pick_gui.launch
   ```



