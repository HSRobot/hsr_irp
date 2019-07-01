/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Foshan Huashu Robotics Co.,Ltd
 * All rights reserved.
 * 
 * Author: CanJun Shen <1252773119@qq.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <rviz_pickplace_commander/voicePickCoke.h>
#include <rviz_pickplace_commander/voicePickDiamond.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

int main(int argc, char** argv)
{
  if(argc < 2){
     printf("请先输入参数");
     return 0;}
  char *agv = argv[1];
  //printf("agv:%s",agv);
  ros::init (argc, argv, "voice_pickplace");//initialize node 
  ros::NodeHandle np;                       //node Handle
  ros::ServiceClient client_voice_coke = np.serviceClient<rviz_pickplace_commander::voicePickCoke>("VoicePickCoke");
  ros::ServiceClient client_voice_diamond = np.serviceClient<rviz_pickplace_commander::voicePickDiamond>("VoicePickDiamond");
  rviz_pickplace_commander::voicePickCoke voice_pickcoke_srv;
  rviz_pickplace_commander::voicePickDiamond voice_pickdiamond_srv;
 if (strcmp(agv,"pickcoke")==0)
 {
  if(client_voice_coke.call(voice_pickcoke_srv))
     ROS_ERROR("pick coke succeeful!!!");  
  else
     ROS_ERROR("pick coke faile!!!");  
}
else if(strcmp(agv,"pickdiamond")==0)
{
  if(client_voice_diamond.call(voice_pickdiamond_srv))
      ROS_ERROR("pick diamond succeeful!!!");  
  else
     ROS_ERROR("pick diamond faile!!!");
}
 return 0;
}
