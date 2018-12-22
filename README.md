#### 加载模型
	``` roslaunch hsr_description <robot>_upload.launch

#### 显示模型
	``` roslaunch hsr_description display.launch

#### 启动仿真环境
	1, 一键启动：	``` roslaunch hsr_bringup <robot>_sim.launch
	2, 单独启动（推荐）:
		``` roslaunch hsr_description <robot>_upload.launch 										# 加载模型
		``` roslaunch hsr_gazebo <robot>_gazebo.launch												# 启动gazebo
		``` roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=true		# 启动moveit核心
		``` roslaunch co602_moveit_config moveit_rviz.launch										# 启动rviz

#### 使用实际机器人
	1，一键启动：	``` roslaunch hsr_bringup <robot>_go.launch robot_ip:=<robot_ip>
	2，单独启动（推荐）：
		``` roslaunch hsr_description <robot>_upload.launch 											# 加载模型
		``` roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=false			# 启动moveit核心
		``` roslaunch industrial_robot_client robot_full_interface_download.launch robot_ip:=<robot_ip> # 启动ros-i客户端
		``` roslaunch <robot>_moveit_config moveit_rviz.launch											# 启动可视化界面

#### 使用gazebo中的相机
	``` roslaunch hsr_description <robot>_with_kinect_upload.launch
	``` roslaunch hsr_gazebo robot_gazebo.launch

#### 启动pick 虚拟
	``` roslaunch hsr_description co602_with_gripper_upload.launch										# 加载模型
	``` roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=true				# 启动moveit核心
	``` roslaunch <robot>_moveit_config moveit_rviz.launch												# 启动rviz
	``` rosrun hsr_pick moveit_pick_and_place_demo.py													# 启动抓取测试程序


#### 启动pick（已知物体位姿）
	``` roslaunch hsr_description co602_with_gripper_upload.launch										# 加载模型
	``` roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=false				# 启动moveit核心
	``` roslaunch industrial_robot_client robot_full_interface_download.launch robot_ip:=<robot_ip> 	# 启动ros-i客户端
	``` rosrun hsr_gripper_driver gripper_control_srv													# 启动夹爪服务端
	``` rosrun hsr_gripper_driver gripper_action.py														# 启动夹爪action
	``` rosrun hsr_pick moveit_pick_and_place_demo.py													# 启动抓取测试程序

#### 启动pick and ork（前提需要对相机进行标定）
	``` roslaunch kinect2_bridge  kinect2_bridge.launch													# 启动kinect
	``` roslaunch hsr_description co602_with_gripper_upload.launch										# 加载模型
	``` roslaunch <robot>_moveit_config <robot>_moveit_planning_execution.launch sim:=false				# 启动moveit核心
	``` roslaunch industrial_robot_client robot_full_interface_download.launch robot_ip:=<robot_ip> 	# 启动ros-i客户端
	``` rosrun hsr_gripper_driver gripper_control_srv													# 启动夹爪服务端
	``` rosrun hsr_gripper_driver gripper_action.py														# 启动夹爪action（将标定数据替换进去）
	``` roslaunch hsr_bringup ork_base_tf.launch														# 发布相机相关的TF映射
	``` rosrun hsr_pick tf_camera_to_base.py															# 将物体相机坐标映射到世界坐标上
	``` rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork	# 启动检测程序
	``` rosrun hsr_pick moveit_pick_and_place_demo.py													# 启动ORK抓取测试程序


#### 启动pick and ork (RVIZ GUI)
	``` roslaunch hsr_bringup pick_gui.launch



