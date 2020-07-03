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

#include "joint_trajectory_cubic_downloader.h"

namespace industrial_robot_client
{
namespace joint_trajectory_downloader
{
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
using industrial::simple_message::SimpleMessage;  
namespace SpecialSeqValues = industrial::joint_traj_pt::SpecialSeqValues;


bool JointTrajectoryCubicDownloader::init(std::string default_ip, int default_port){

    //  调用父类的init函数
    std::string ip_0 = "10.10.56.214";
//    int port = 23234;
    JointTrajectoryInterface::init(ip_0, default_port);

    impedanceErr.resize(6);
    for(int i = 0; i < 6; i++)
        impedanceErr[i] = 0.0;

    LOG_INFO("JointTrajectoryCubicDownloader initing");

    // 使用单独的队列来获取插补偏移值
    this->impeQueue = new ros::CallbackQueue();
    this->queuenode_.setCallbackQueue(impeQueue);
    this->queuenode_.subscribe("impedance_err", 1, &JointTrajectoryCubicDownloader::impedanceCB, this);
    LOG_INFO("JointTrajectoryCubicDownloader init finsh");
}

void JointTrajectoryCubicDownloader::impedanceCB(const sensor_msgs::JointState::ConstPtr &msg){

    ROS_ERROR("impedanceCB .......");
    for(int i = 0; i < 6; i++)
        impedanceErr[i] =  msg->position[i];

}

bool JointTrajectoryCubicDownloader::send_to_robot(const std::vector<JointTrajPtFullMessage>& messages){

    ROS_ERROR(" JointTrajectoryCubicDownloader enter ...... ");
    bool rslt=true;
    std::vector<JointTrajPtFullMessage> points(messages);
    SimpleMessage msg;
    JointTrajPtFullMessage targetPoint;

    // 第一个点和第二个点的间隔周期
    ros::Duration startDur(0.05);

    // 粗插补周期
    ros::Rate cubicRate(10);

    int pointSize = points.size();
    if (pointSize < 2)
        points.push_back(JointTrajPtFullMessage(points[0]));

    // The first and last points are assigned special sequence values
    points.begin()->setSequence(SpecialSeqValues::START_TRAJECTORY_DOWNLOAD);
    points.back().setSequence(SpecialSeqValues::END_TRAJECTORY);

    if (!this->connection_->isConnected())
    {
        ROS_ERROR("Attempting robot reconnection");
        this->connection_->makeConnect();
    }

    ROS_ERROR("Sending trajectory points, size: %d", (int)points.size());

    int t = 0;
    int seq = 1;
    // send the first point and after 50ms will send second point
    targetPoint = sample_inter(points, t);
    targetPoint.toTopic(msg);
    this->connection_->sendMsg(msg);

    // 休眠50ms马上发送第二点，为控制器预留50MS的窗口
    startDur.sleep();

    // 当seq为END_TRAJECTORY，表明最后一个点已经发送
    while(targetPoint.point_.getSequence() != SpecialSeqValues::END_TRAJECTORY){

        // 获取一次插补偏移值
        impeQueue->callOne();

        // 获取插补点
        t += INTER_RATE;
        targetPoint = sample_inter(points, t);

        // 当处于最后一个点时，不修改其seq
        if(targetPoint.point_.getSequence() != SpecialSeqValues::END_TRAJECTORY)
            targetPoint.setSequence(seq);
        seq++;

        // 发送插补点
        targetPoint.toTopic(msg);
        ROS_ERROR(" connection_ sendMsg ...... ");
        bool ptRslt = this->connection_->sendMsg(msg);
        if (ptRslt)
            ROS_ERROR("Point[%d] sent to controller",seq);
        else
            ROS_ERROR("Failed sent joint point, skipping point");
        rslt &= ptRslt;

        // 休眠100ms，此处会导致轨迹运行比预先的慢100ms
        cubicRate.sleep();
    }

    ROS_ERROR(" JointTrajectoryCubicDownloader exit ...... ");
    return rslt;
}

JointTrajPtFullMessage JointTrajectoryCubicDownloader::sample_inter(std::vector<JointTrajPtFullMessage>& points, int time){
    int pointSize = points.size();
    shared_real endTime, pointTime;

    //  将秒转换为MS
    shared_real t = time / 1000.0;

    // 获取结束点位的时间
    points[pointSize -1].point_.getTime(endTime);

    // 第一个点位
    if(t == 0)
        return points[0];
    if(t >= endTime)
        return points[pointSize-1];

    // 寻找可以满足当前时间的点位
    int i = 1;
    points[i].point_.getTime(pointTime);
    while(t > pointTime){
        i++;
        points[i].point_.getTime(pointTime);
    }

    return quintic_inter(points[i-1], points[i], t);
}

JointTrajPtFullMessage JointTrajectoryCubicDownloader::quintic_inter(JointTrajPtFullMessage start, JointTrajPtFullMessage end, shared_real time){

    
    shared_real pointTime_start,pointTime_end;
    start.point_.getTime(pointTime_start);
    end.point_.getTime(pointTime_end);

    industrial::joint_data::JointData pos_start,pos_end;
    industrial::joint_data::JointData vel_start,vel_end;
    industrial::joint_data::JointData acc_start,acc_end;

    start.point_.getPositions(pos_start);
    end.point_.getPositions(pos_end);
    start.point_.getVelocities(vel_start);
    end.point_.getVelocities(vel_end);
    start.point_.getAccelerations(acc_start);
    end.point_.getAccelerations(acc_end);

    //中间点与起始点间的时间间隔
    shared_real t = time - pointTime_start;
    //起始点与终止点间的时间间隔
    shared_real T = pointTime_end - pointTime_start;

    ROS_DEBUG("start time WAS = %f, end time = %f",pointTime_start, pointTime_end);
    ROS_DEBUG("TIME WAS = %f, durtion = %f", t, T);

    //求解五次多项式各系数
    shared_real a0[6],a1[6],a2[6],a3[6],a4[6],a5[6];

    shared_real position_start[6],velocity_start[6],acceleration_start[6];
    shared_real position_end[6],velocity_end[6],acceleration_end[6];
    shared_real position_middle[6],velocity_middle[6],acceleration_middle[6];

    for(int i;i<6;i++)
    {
        pos_start.getJoint(i,position_start[i]);
        pos_end.getJoint(i,position_end[i]);
        vel_start.getJoint(i,velocity_start[i]);
        vel_end.getJoint(i,velocity_end[i]);
        acc_start.getJoint(i,acceleration_start[i]);
        acc_end.getJoint(i,acceleration_end[i]);
    }

    ROS_DEBUG("start_vcc[0] = %f end_vcc[0]=%f", velocity_start[0], velocity_end[0]);
    ROS_DEBUG("start_pos[0] = %f end_pos[0]=%f", position_start[0], position_end[0]);

    for(int i = 0;i<6;i++)
    {
        //计算五次多项式各系数
        a0[i] = position_start[i];
        a1[i] = velocity_start[i];
        a2[i] = acceleration_start[i]/2;
        a3[i] = (20*position_end[i]-20*position_start[i]-(8*velocity_end[i]+12*velocity_start[i])*T-(3*acceleration_start[i]-acceleration_end[i])*T*T)/(2*T*T*T);
        a4[i] = (30*position_start[i]-30*position_end[i]+(14*velocity_end[i]+16*velocity_start[i])*T+(3*acceleration_start[i]-2*acceleration_end[i])*T*T)/(2*T*T*T*T);
        a5[i] = (12*position_end[i]-12*position_start[i]-(6*velocity_end[i]+6*velocity_start[i])*T-(acceleration_start[i]-acceleration_end[i])*T*T)/(2*T*T*T*T*T);

        //计算指定时间点的位置、速度、加速度
        position_middle[i] = a0[i] + a1[i]*t + a2[i]*t*t + a3[i]*t*t*t + a4[i]*t*t*t*t + a5[i]*t*t*t*t*t;
        velocity_middle[i] = a1[i] + 2*a2[i]*t + 3*a3[i]*t*t + 4*a4[i]*t*t*t + 5*a5[i]*t*t*t*t;
        acceleration_middle[i] = 2*a2[i] + 6*a3[i]*t + 12*a4[i]*t*t + 20*a5[i]*t*t*t;

        // 加上插补关节角偏移值
        //position_middle[i] += impedanceErr[i];

    }

    //创建输出对象
    JointTrajPtFullMessage middle_msg;

    industrial::joint_traj_pt_full::JointTrajPtFull middle;
    //中间点的位置、速度和加速度
    industrial::joint_data::JointData pos_middle,vel_middle,acc_middle;
    for(int i=0;i<6;++i)
    {
        pos_middle.setJoint(i,position_middle[i]);
        vel_middle.setJoint(i,velocity_middle[i]);
        acc_middle.setJoint(i,acceleration_middle[i]);
    }

    ROS_ERROR("position_middle =  %f || %f || %f || %f || %f || %f",\
             position_middle[0],position_middle[1],position_middle[2],position_middle[3],position_middle[4],position_middle[5] );

    middle.init(1,1,1,time,pos_middle,vel_middle,acc_middle);

    middle_msg.init(middle);

    return middle_msg;
}

}
}
