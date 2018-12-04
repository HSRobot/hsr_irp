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
