/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Foshan Huashu Robotics Co.,Ltd
 * All rights reserved.
 * 
 * Author: Kunlin Xu <1125290220@qq.com>
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

#ifndef JOINT_TRAJECTORY_FULL_DOWNLOADER_H
#define JOINT_TRAJECTORY_FULL_DOWNLOADER_H

#include "joint_trajectory_full_interface.h"
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>


// 粗插补周期，单位ms
#define INTER_RATE 100

namespace industrial_robot_client{
namespace joint_trajectory_downloader{

using industrial_robot_client::joint_trajectory_interface::JointTrajectoryFullInterface;
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::shared_types::shared_real;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

/**
 * @brief The JointTrajectoryCubicDownloader class 华数机器人的五次多项式插补 关于ROS
 */
class JointTrajectoryCubicDownloader : public JointTrajectoryFullInterface{

public:

    bool send_to_robot(const std::vector<JointTrajPtFullMessage>& messages);


    /**
      * @brief		init虚函数的实现
      * @details		多态，在初始化过程中将我们的回调函数进行注册
      * @param[in]  default_ip：机器人IP地址 default_port：机器人端口
      * @param[out]	null
      * @exception	null
      * @return		null
      * @autor		xukunlin
      * @date			2018/11/01
      */
    bool init(std::string default_ip = "", int default_port = StandardSocketPorts::MOTION);


    /**
      * @brief		关节偏移话题回调函数
      * @details		关节偏移话题回调函数
      * @param[in]	关节偏移信息
      * @param[out]	null
      * @exception	null
      * @return		null
      * @autor		xukunlin
      * @date			2018/11/01
      */
    void impedanceCB(const sensor_msgs::JointState::ConstPtr& msg);


    /**
    * @brief		粗插补
    * @details		粗插补
    * @param[in]	points:需要插补的轨迹  time:时间
    * @param[out]	null
    * @exception	null
    * @return		目标点位
    * @autor		xukunlin
    * @date			2018/10/31
    */
    JointTrajPtFullMessage sample_inter(std::vector<JointTrajPtFullMessage>& points, int time);

private:
    /**
    * @brief		三次多项式插补
    * @details		三次多项式插补
    * @param[in]	star:起始点  end：目标点 time:时间
    * @param[out]	null
    * @exception	null
    * @return		目标点位
    * @autor		xukunlin
    * @date			2018/10/31
    */
    JointTrajPtFullMessage quintic_inter(JointTrajPtFullMessage start, JointTrajPtFullMessage end, shared_real time);

    // 插补偏移值
    std::vector<double> impedanceErr;

    // 回调函数队列 nodehandle
    ros::NodeHandle queuenode_;

    // 偏移值回调函数队列
    ros::CallbackQueue *impeQueue;

};

}	//namespace joint_trajectory_downloader
}	//namespace industrial_robot_client

#endif
