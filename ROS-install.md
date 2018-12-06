###基础环境
	1, vmware
	2, ubuntu 14.04 x86_64

_*注意事项*_
	ubuntu 版本必须使用64位14.04

###1，设置源
	sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

###2，设置密钥
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

###3，更新源
	sudo apt-get update

###4，安装ROS
	sudo apt-get install ros-indigo-desktop-full

###5，初始化rosdep
	sudo rosdep init

###6，更新rosdep
	rosdep update

###7，设置环境变量
	echo source /opt/ros/indigo/setup.bash >> ~/.bashrc

_*注意事项*_
如果执行第4步出现下面的错误，使用如下方式解决,切勿重启，否则会导致系统无法正常启动
安装错误：
Errors were encountered while processing:
 /var/cache/apt/archives/python-rosdistro-modules_0.7.0-1_all.deb
 /var/cache/apt/archives/python-rosdistro_0.7.0-100_all.deb
E: Sub-process /usr/bin/dpkg returned an error code (1)
解决方案：
1, sudo apt-get purge ros-*
2, sudo apt-get update
3, sudo apt-get upgrade
4, sudo apt-get install ros-indigo-desktop-full

### 安装平台的驱动
####1, 创建工作区： 
	cd ~; mkdir catkin_ws
	cd catkin_ws; mkdir src
	catkin_make
	echo source ~/catkit_ws/devel/setup.bash >> ~/.bashrc


####2, 下载源码：
	sudo apt-get install git	# 安装git
	cd ~/catkin_ws/src
	git clone git@github.com:HSRobot/hsr_irp.git

####3, 下载依赖项：
	sudo apt-get install ros-indigo-industrial-core		# 安装ROS-I
	sudo apt-get install ros-indigo-moveit-full		# 安装Moveit
	sudo apt-get install ros-indigo-serial			# 安装serial
	
####4, 编译：
	catkin_make -DCATKIN_WHITELIST_PACKAGES="hsr_msgs;gripper_control;ork_interface;hsr_pick"
	catkin_make -DCATKIN_WHITELIST_PACKAGES=""
