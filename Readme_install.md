#Linux系统+Kinectv2摄像头
#一、首先下载libfreenect2包
$git clone https://github.com/OpenKinect/libfreenect2.git
$cd libfreenect2/depends
#下载upgrade deb 文件
./download_debs_trusty.sh

#以下安装依赖库都需要在libfreenect2/depends文件夹中
#装编译工具
$sudo apt-get install build-essential cmake pkg-config
$sudo dpkg -i debs/libusb*deb
$sudo apt-get install libturbojpeg libjpeg-turbo8-dev

#装 OpenGL
$sudo dpkg -i debs/libglfw3*deb
$sudo apt-get install -f
$sudo apt-get install libgl1-mesa-dri-lts-vivid

#装 OpenCL
#Intel GPU:
$sudo apt-add-repository ppa:floe/beignet
$sudo apt-get update
$sudo apt-get install beignet-dev
$sudo dpkg -i debs/ocl-icd*deb

#**将libfreenect2下的depends下的install_ubuntu.sh文件中的版本号为3.2.1
$cd libfreenect2/depends
$sh install_ubuntu.sh
$sudo dpkg -i libglfw3*_3.2.1-1_*.deb

#编译库
$cd ..
$mkdir build && cd build
$cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -DENABLE_CXX11=ON
$make
$sudo make install

$sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

#测试，如果没有图像，把kinect的USB拔插一下(kinect只能使用usb3.0)
$./bin/Protonect

#二、下载iai_kinect2包
$cd ~/catkin_ws/src/
$git clone https://github.com/code-iai/iai_kinect2.git
$cd iai_kinect2
$rosdep install -r --from-paths .

#编译iai_kinect包
$cd ~/catkin_ws
$catkin_make -DCMAKE_BUILD_TYPE="Release"
$rospack profile
#这一步可能会报错，但这个报错是正常的，不用管，不会影响最终结果。
$rosdep install -r --from-paths .

#测试
#打开新终端
$roslaunch kinect2_bridge kinect2_bridge.launch
#再打开一个新终端
$rosrun kinect2_viewer kinect2_viewer
#正常会有图象显示

#三、easy_handeye安装
#在安装之前首先要按照以下步骤安装依赖包（注意版本号）：
#1，安装
$sudo apt-get install ros-indigo-visp

#这个和第一个一样！
cd ~/catkin_ws/src
git clone -b indigo-devel https://github.com/lagadic/vision_visp.git
cd ..
catkin_make --pkg visp_hand2eye_calibration

#3，源码安装
#如编译发生opencv版本不匹配，则修改aruco_ros/aruco/CMakeLists.txt的第六行：“OpenCV 3 REQUIRED” 改为 “OpenCV REQUIRED”
#和aruco_ros/aruco/package.xml的<depend>标签中：“opencv3“改为libopencv-dev，再重新catkin_make
$cd ~/catkin_ws/src
$git clone -b indigo-devel https://github.com/pal-robotics/aruco_ros
$cd ..
$catkin_make

#安装easy_handeye
$cd ~/catkin_ws/src
$git clone https://github.com/IFL-CAMP/easy_handeye
$cd ..
$catkin_make

#三、安装ORK和linemod算法
#install ecto
sudo apt-get install ros-indigo-ecto
sudo apt-get install ros-indigo-ecto-image-pipeline
sudo apt-get install ros-indigo-opencv-candidate
sudo apt-get install ros-indigo-ecto-ros
sudo apt-get install libsdl1.2-dev 
#安装ORk用到的package,下载源码
$cd catkin_ws/src
$git clone http://github.com/wg-perception/object_recognition_msgs
$git clone http://github.com/wg-perception/object_recognition_ros
$git clone http://github.com/wg-perception/object_recognition_ros_visualization
$git clone http://github.com/wg-perception/object_recognition_core
$git clone http://github.com/wg-perception/linemod
$git clone http://github.com/wg-perception/ork_renderer
#编译
$cd ..
$catkin_make

#如果编译不成功因為是少了 GL/osmesa.h，所以需要額外下一個指令 sudo apt-get install libosmesa6-dev 來安裝
$roscd object_recognition_core
#如果可以代表成功了
#如果失败 可能是roscd沒有被加進 ROS_PACKAGE_PATH 中
#先 vim ~/.bashrc 一下，然後在最下面補上一行:
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/rosindigo/catkin_ws/src"

#安裝 CouchDB服务器工具
$sudo apt-get install couchdb
#接下來檢查一下是否有安裝成功
$curl -X GET http://localhost:5984
#服务器网址
http://localhost:5984/_utils/database.html?object_recognition/_design/objects/_view/by_object_name 

#下载可乐罐模型
$git clone https://github.com/wg-perception/ork_tutorials
$cd .. && catkin_make

#源码修改
#可直接替换我司的package中的linmod和ork_renderer,然后编译
#详情请参考https://github.com/wg-perception/linemod/issues/28

#四、训练模型
#添加需要训练的模型名称及描述
$rosrun object_recognition_core object_add.py -n "coke " -d "A universal can of coke" --commit
#添加stl模型至服务器
$rosrun object_recognition_core mesh_add.py $id$ /home/fshs/catkin_ws/src/ork_tutorials/data/coke.stl --commit
#linemod算法训练模型
$rosrun object_recognition_core training -c `rospack find object_recognition_linemod`/conf/training.ork
#识别
#发布kinect外参
$roslaunch easy_handeye publish.launch eye_on_hand:=false namespace_prefix:=hsr_kinect_handeyecalibration 
#启动kinect驱动
$roslaunch kinect2_bridge kinect2_bridge.launch #启动kinect驱动
#识别模型
$ rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork 

#四、利用solidworks和blender创建新的ork训练模型（stl）
#首先在SolidWorks建模
#然后另存为.stl，注意在选项中设置导出的单位（以以实际尺寸的单位为准，应为stl中只会记录数值，不会记录单位信息）
#在Linux环境下，将.stl到入blender软件中，并检查模型尺寸是否准确。
#另存为.stl,重新生成模型，注意生成的stl文件大小，如果大小相差较大，则属于生成错误，检查操作是否有误
