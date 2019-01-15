## 基础环境
	1, vmware
	2, ubuntu 14.04 x86_64

# ROS-indigo环境安装

###1，设置源
   ```bash
   sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

###2，设置密钥
   ```bash
   $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
   ```

###3，更新源
   ```bash
   $ sudo apt-get update
   $ sudo apt-get upgrade
   ```

###4，安装ROS
   ```bash
   $ sudo apt-get install ros-indigo-desktop-full
   ```

###5，初始化rosdep
   ```bash
   $ sudo rosdep init
   ```

###6，更新rosdep
   ```bash
   $ rosdep update
   ```

###7，设置环境变量
   ```bash
   $ echo source /opt/ros/indigo/setup.bash >> ~/.bashrc
   ```

_*注意事项*_
如果执行第4步出现下面的错误，使用如下方式解决,切勿重启，否则会导致系统无法正常启动
安装错误：
Errors were encountered while processing:
 /var/cache/apt/archives/python-rosdistro-modules_0.7.0-1_all.deb
 /var/cache/apt/archives/python-rosdistro_0.7.0-100_all.deb
E: Sub-process /usr/bin/dpkg returned an error code (1)
解决方案：
   ```bash
   $ sudo apt-get purge ros-*
   $ sudo apt-get update
   $ sudo apt-get upgrade
   $ sudo apt-get install ros-indigo-desktop-full
   ```

# 安装平台的驱动
###1, 创建工作区：
   ```bash
   $ cd ~; mkdir catkin_ws
   $ cd catkin_ws; mkdir src
   $ catkin_make
   $ echo source ~/catkin_ws/devel/setup.bash >> ~/.bashrc
   ```

###2, 下载源码：
   ```bash
   $ sudo apt-get install git	# 安装git
   $ cd ~/catkin_ws/src
   $ git clone git@github.com:HSRobot/hsr_irp.git
   ```

###3, 下载依赖项：
   ```bash
   $ sudo apt-get install ros-indigo-industrial-core		# 安装ROS-I
   $ sudo apt-get install ros-indigo-moveit-full		# 安装Moveit
   $ sudo apt-get install ros-indigo-serial			# 安装serial
   ```
	
###4, 编译：
   ```bash
   $ catkin_make
   ```
