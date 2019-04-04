/** 
* @version 	v1.0.1 
* @file		gripper_close_srv.cpp
* @brief	夹抓相关动作的服务端
* @details	实现夹抓如下动作：
*               1、以设定的速度、力矩关闭夹抓
*               2、以设定的速度打开夹抓
*               3、急停
*               4、设置开口参数
*               v1.0.1变更内容：将串口打开程序段封装成服务
* @autor 	wanghaoqing
* @date		2018/10/10
*/

#include "ros/ros.h"

#include "hsr_gripper_driver/serial_open_srv.h"
#include "hsr_gripper_driver/close_srv.h"
#include "hsr_gripper_driver/open_srv.h"
#include "hsr_gripper_driver/stop_srv.h"
#include "hsr_gripper_driver/open_size_srv.h"
#include "hsr_gripper_driver/read_open_size_srv.h"

#include "serial/serial.h"

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))

/* USB转串口设备号 */
#define SERIAL_DEV "/dev/ttyUSB0"
/* 串口波特率 */
#define SERIAL_BAUDRATE 115200

/* 夹抓相关动作的默认串口输入值 */
#define ARRAY_CLOSE_GRIPPER {0xEB,0x90,0x01,0x05,0x10,0xF4,0x01,0x64,0x00,0x6F}
#define ARRAY_OPEN_GRIPPER {0xEB,0x90,0x01,0x03,0x11,0xF4,0x01,0x0A}
#define ARRAY_STOP_GRIPPER {0xEB,0x90,0x01,0x01,0x16,0x18}
#define ARRAY_SET_OPEN_SIZE_GRIPPER {0xEB,0x90,0x01,0x05,0x12,0xE8,0x03,0x70,0x00,0x73}
#define ARRAY_GET_OPEN_SIZE_GRIPPER {0xEB,0x90,0x01,0x01,0x13,0x15}

serial::Serial ros_ser;

/** 
* @brief	    串口打开服务端函数
* @details		实现串口打开服务
* @param[in]	req,其中req.serialNo是串口号,req.baudrate是波特率
* @param[out]	res
* @exception	
* @return		true
* @autor		wanghaoqing
* @date			2018/10/10
*/
bool serial_open(hsr_gripper_driver::serial_open_srv::Request  &req,
                 hsr_gripper_driver::serial_open_srv::Response &res)
{
     if(ros_ser.isOpen())
     {
         ROS_INFO_STREAM("Serial Port opened");
     }
     else
     {
	/*设置串口参数，打开串口*/
	try
	{
	    ros_ser.setPort(req.serialNo);
	    ros_ser.setBaudrate((int)req.baudrate);
	    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	    ros_ser.setTimeout(to);
	    ros_ser.open();
	}
	catch (serial::IOException& e)
	{
	    ROS_ERROR_STREAM("Unable to open port ");
            return false;
	}
         
     }
     
     if(ros_ser.isOpen()){
         ROS_INFO_STREAM("Serial Port opened");
     }
     else{
         return false;
     }

     return true;

}

/** 
* @brief	    关闭夹抓的服务端函数
* @details		实现以一定的速度和力矩关闭夹抓
* @param[in]	req,其中req.speed取值范围是1-1000,req.force取值范围是50-1000
* @param[out]	res
* @exception	
* @return		true
* @autor		wanghaoqing
* @date			2018/09/18
*/
bool close(hsr_gripper_driver::close_srv::Request  &req,
           hsr_gripper_driver::close_srv::Response &res)
{
  if(req.speed > 1000 || req.speed <1)
  {
      ROS_INFO("The input speed is invalid. Please enter the number between 1-1000.");
      return false;
  }
  if(req.force > 1000 || req.force <50)
  {
      ROS_INFO("The input force is invalid. Please enter the number between 50-1000.");
      return false;
  }

  unsigned char close_buffer[] = ARRAY_CLOSE_GRIPPER;
  
   int size_close_buffer = ARRAY_SIZE(close_buffer);

  //将输入值req.speed保存为临时变量，以备后面数值处理
  int temp = req.speed;
  //将低8位存入close_buffer[5]
  close_buffer[5]= (unsigned char)(temp&0xFF);
  //将高8位存入close_buffer[6]
  close_buffer[6]= (unsigned char)((temp&0xFF00)>>8);

  //将输入值req.force保存为临时变量，以备后面数值处理
  temp = req.force;
  //将低8位存入close_buffer[7]
  close_buffer[7]= (unsigned char)(temp&0xFF);
  //将高8位存入close_buffer[8]
  close_buffer[8]= (unsigned char)((temp&0xFF00)>>8);
  //close_buffer[9]为校验和位 （（B2+B3+…+B8）&0xFF）
  close_buffer[size_close_buffer-1]= (unsigned char)((close_buffer[2] + close_buffer[3] + \
                                                      close_buffer[4] + close_buffer[5] + \
                                                      close_buffer[6] + close_buffer[7] + close_buffer[8])&0xFF);

  ros_ser.write(close_buffer,size_close_buffer);
  ROS_ERROR_STREAM("THE COMMAND OF --CLOSE-- HAS BEEN LOADED!!!");
/*
  for(int i=0;i<10;i++)
  {
     //ROS_ERROR_STREAM(i);
     ROS_ERROR_STREAM("CLOSE:");
     ROS_ERROR_STREAM(close_buffer[i]);
  }

  ROS_ERROR_STREAM("close_buffer[7]");
  ROS_ERROR_STREAM(close_buffer[7]);
  ROS_ERROR_STREAM("close_buffer[5]");
  ROS_ERROR_STREAM(close_buffer[5]);
  ROS_ERROR_STREAM("close_buffer[6]");
  ROS_ERROR_STREAM(close_buffer[6]);
  ROS_ERROR_STREAM("close_buffer[9]");
  ROS_ERROR_STREAM(close_buffer[9]);
*/
  ROS_ERROR_STREAM("gripper closed!");
  res.closed = true;
 
 return true;
}

/** 
* @brief	    打开夹抓的服务端函数
* @details		实现以一定的速度和力矩关闭夹抓
* @param[in]	req,其中req.speed取值范围是1-1000
* @param[out]	res
* @exception	
* @return		true
* @autor		wanghaoqing
* @date			2018/09/18
*/
bool open(hsr_gripper_driver::open_srv::Request  &req,
          hsr_gripper_driver::open_srv::Response &res)
{
  if(req.speed > 1000 || req.speed <1)
  {
      ROS_INFO("The input speed is invalid. Please enter the number between 1-1000.");
      return false;
  }

   unsigned char open_buffer[] = ARRAY_OPEN_GRIPPER;
  
   int size_open_buffer = ARRAY_SIZE(open_buffer);
   //将输入值req.speed保存为临时变量，以备后面数值处理
   int temp = req.speed;
   //将低8位存入open_buffer[5]
   open_buffer[5]= (unsigned char)(temp&0xFF);
   //将高8位存入open_buffer[6]
   open_buffer[6]= (unsigned char)((temp&0xFF00)>>8);
   //close_buffer[7]为校验和位 （（B2+B3+…+B8）&0xFF）
   open_buffer[size_open_buffer-1]= (unsigned char)((open_buffer[2] + open_buffer[3] + \
                                                     open_buffer[4] + open_buffer[5] + \
                                                     open_buffer[6])&0xFF);
   ros_ser.write(open_buffer,size_open_buffer);
   ROS_INFO("THE COMMAND OF --OPEN-- HAS BEEN LOADED!!!");

   res.opened = true;
   ROS_ERROR_STREAM("gripper open!");

 return true;
}

/** 
* @brief	    夹抓急停的服务端函数
* @details		实现以一定的速度和力矩关闭夹抓
* @param[in]	   
* @param[out]	        
* @exception	
* @return		true
* @autor		wanghaoqing
* @date			2018/09/18
*/
bool stop(hsr_gripper_driver::stop_srv::Request  &req,
          hsr_gripper_driver::stop_srv::Response &res)
{

  unsigned char stop_buffer[] = ARRAY_STOP_GRIPPER;

  int size_stop_buffer = ARRAY_SIZE(stop_buffer);
  
  ros_ser.write(stop_buffer,size_stop_buffer);
  ROS_INFO("THE COMMAND OF --STOP-- HAS BEEN LOADED!!!");
  
  return true;
}

/** 
* @brief	    设置夹抓开口大小服务端函数
* @details		设置主控单元设置夹爪的最大和最小开口参数
* @param[in]	req，B5B6 为开口最大值，数据范围 0—1000，
*                    B7B8 为开口最小值，数据范围 0—1000
* @param[out]	res
* @exception	
* @return		true
* @autor		wanghaoqing
* @date			2018/09/18
*/
bool set_open_size(hsr_gripper_driver::open_size_srv::Request  &req,
                   hsr_gripper_driver::open_size_srv::Response &res)
{

  if(req.max > 1000 || req.max <0)
  {
      ROS_INFO("The input req.max is invalid. Please enter the number between 0-1000.");
      return false;
  }
  if(req.min > 1000 || req.min <0)
  {
      ROS_INFO("The input req.min is invalid. Please enter the number between 0-1000.");
      return false;
  }
  if(req.max < req.min || req.max == req.min)
  {
      ROS_INFO("The input req.min and req.max are both invalid. Please ensure that req.min is less than req.max.");
      return false;
  }
  unsigned char buffer[] = ARRAY_SET_OPEN_SIZE_GRIPPER;

   int size_buffer = ARRAY_SIZE(buffer);

  //将输入值req.max保存为临时变量，以备后面数值处理
  int temp = req.max;
  //将低8位存入close_buffer[5]
  buffer[5]= (unsigned char)(temp&0xFF);
  //将高8位存入close_buffer[6]
  buffer[6]= (unsigned char)((temp&0xFF00)>>8);

  //将输入值req.force保存为临时变量，以备后面数值处理
  temp = req.min;
  //将低8位存入close_buffer[7]
  buffer[7]= (unsigned char)(temp&0xFF);
  //将高8位存入close_buffer[8]
  buffer[8]= (unsigned char)((temp&0xFF00)>>8);
  //close_buffer[9]为校验和位 （（B2+B3+…+B8）&0xFF）
  buffer[size_buffer-1]= (unsigned char)((buffer[2] + buffer[3] + \
                                                      buffer[4] + buffer[5] + \
                                                      buffer[6] + buffer[7] + buffer[8])&0xFF);

  ros_ser.write(buffer,size_buffer);
  ROS_INFO("THE COMMAND OF --SET_OPEN_SIZE-- HAS BEEN LOADED!!!");
  res.openSetFinished = true;  

  return true;
}

/** 
* @brief	    读取夹抓开口大小服务端函数
* @details		实现以一定的速度和力矩关闭夹抓
* @param[in]	   
* @param[out]	        
* @exception	
* @return		true
* @autor		wanghaoqing
* @date			2018/09/18
*/
bool get_open_size(hsr_gripper_driver::read_open_size_srv::Request  &req,
                   hsr_gripper_driver::read_open_size_srv::Response &res)
{

  unsigned char buffer[] = ARRAY_GET_OPEN_SIZE_GRIPPER;
  int size_buffer = ARRAY_SIZE(buffer);
  ros_ser.write(buffer,size_buffer);
  ROS_INFO("THE COMMAND OF --GET OPEN SIZE-- HAS BEEN LOADED!!!");
 
  //for(int i=0;i<1000;i++)
  //{
  //}
  sleep(1000);

  unsigned char get_open_size_buffer[10] ={};
  ros_ser.read(get_open_size_buffer,10);
  for(int i=0;i<10;i++)
  {
     ROS_INFO("[0x%02x]",get_open_size_buffer[i]);
  }

  return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gripper_control_server");
  ros::NodeHandle n;

  /* 串口通信服务端 */
  ros::ServiceServer serial_open_srv = n.advertiseService("serial_open", serial_open);
  /* 夹抓关闭服务端 */
  ros::ServiceServer close_srv = n.advertiseService("gripper_close", close);
  /* 夹抓打开服务端 */
  ros::ServiceServer open_srv = n.advertiseService("gripper_open", open);
  /* 夹抓急停服务端 */
  ros::ServiceServer stop_srv = n.advertiseService("gripper_stop", stop);
  /* 设置夹抓开口大小服务端 */
  ros::ServiceServer open_size_srv = n.advertiseService("gripper_set_open_size", set_open_size);
  /* 读取夹抓开口大小服务端 */
  ros::ServiceServer read_open_size_srv = n.advertiseService("gripper_get_open_size", get_open_size);

  ros::spin();

  return 0;
}

