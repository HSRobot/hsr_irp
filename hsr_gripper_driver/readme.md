、、把该文件夹放在ros工作目录下

、、文件夹src存放的是夹抓服务端程序，文件夹srv存放的是定义的各种服务类型

、、通过rosrun gripper_control gripper_control_srv 可启动夹抓动作服务端

、、通过rosservice list，可查看启动的服务

、、与夹抓工作相关的服务如下：
    1 /gripper_close     调用示例 rosservice call /gripper_close 500 100 ，其中500为夹持关闭速度，100为夹持力矩
    2 /gripper_open      调用示例 rosservice call /gripper_open 500 ，其中500为夹持关闭速度
    3 /gripper_stop      调用示例 rosservice call /gripper_stop, 无其他参数
    4 /gripper_set_open_size  调用示例 rosservice call /gripper_close 1000 0 ，其中1000为最大张口值，0为最小张口值
