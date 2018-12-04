#ifndef JOINT_TRAJECTORY_FULL_INTERFACE_H
#define JOINT_TRAJECTORY_FULL_INTERFACE_H


#include <map>
#include <vector>
#include <string>


#include <industrial_robot_client/joint_trajectory_interface.h>
#include "simple_message/messages/joint_traj_pt_full_message.h"



namespace industrial_robot_client
{
namespace joint_trajectory_interface
{
	using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
	using industrial_robot_client::joint_trajectory_interface::JointTrajectoryInterface;
	namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

class JointTrajectoryFullInterface : public JointTrajectoryInterface{

public:
	void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);	// 重写接收函数
	bool send_to_robot(const std::vector<JointTrajPtMessage>& messages);			// 纯虚函数实现
	virtual bool send_to_robot(const std::vector<JointTrajPtFullMessage>& messages) = 0;		// c虚函数实现
	

private:
	static JointTrajPtFullMessage create_message(int robot_id, int seq, int valid_fields, float time, std::vector<double> joint_pos, \
		std::vector<double> velocity, std::vector<double> accelerations);

	bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<JointTrajPtFullMessage>* msgs);
};

}
}

#endif 
