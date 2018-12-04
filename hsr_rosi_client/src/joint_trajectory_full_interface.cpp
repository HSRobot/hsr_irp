#include <algorithm>
#include "joint_trajectory_full_interface.h"
#include "simple_message/joint_traj_pt.h"
#include "simple_message/joint_traj_pt_full.h"
#include "industrial_utils/param_utils.h"


namespace industrial_robot_client
{
namespace joint_trajectory_interface
{


void JointTrajectoryFullInterface::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg){
	ROS_INFO("Receiving joint trajectory message");

	// check for STOP command
	if (msg->points.empty())
	{
		ROS_INFO("Empty trajectory received, canceling current trajectory");
		//trajectoryStop();
		return;
  	}

	// convert trajectory into robot-format
	std::vector<JointTrajPtFullMessage> robot_msgs;
	if (!trajectory_to_msgs(msg, &robot_msgs))
    	return;

	// send command messages to robot
	send_to_robot(robot_msgs);
}

bool JointTrajectoryFullInterface::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<JointTrajPtFullMessage>* msgs){

	msgs->clear();
	if (!is_valid(*traj))
		return false;

	int valid_fields = industrial::joint_traj_pt_full::ValidFieldTypes::TIME;
	valid_fields |= industrial::joint_traj_pt_full::ValidFieldTypes::POSITION;
	valid_fields |= industrial::joint_traj_pt_full::ValidFieldTypes::VELOCITY;
	valid_fields |= industrial::joint_traj_pt_full::ValidFieldTypes::ACCELERATION;
	

	for (size_t i=0; i<traj->points.size(); ++i)
  	{
		trajectory_msgs::JointTrajectoryPoint rbt_pt;

		rbt_pt = traj->points[i];		// 获取第一个点位
		// rbt_pt.duration.toNSec()
    	JointTrajPtFullMessage msg = create_message(0, i, valid_fields, rbt_pt.time_from_start.toSec(), rbt_pt.positions, \
			rbt_pt.velocities, rbt_pt.accelerations);		// 构建消息

    	msgs->push_back(msg);
  	}

  	return true;	
}


JointTrajPtFullMessage JointTrajectoryFullInterface::create_message(int robot_id, int seq, int valid_fields, float time, std::vector<double> joint_pos, \
		std::vector<double> velocity, std::vector<double> accelerations){
	
  	industrial::joint_data::JointData pos, vel, acc;
  	ROS_ASSERT(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());




  	for (size_t i=0; i<joint_pos.size(); ++i){
    	pos.setJoint(i, joint_pos[i]);
		vel.setJoint(i, velocity[i]);
		acc.setJoint(i, accelerations[i]);
	}

    ROS_DEBUG("time=%f \n", time);

  	industrial::joint_traj_pt_full::JointTrajPtFull pt;
  	pt.init(robot_id, seq, valid_fields, time, pos, vel, acc);

  	JointTrajPtFullMessage msg;
  	msg.init(pt);

  	return msg;
}

bool JointTrajectoryFullInterface::send_to_robot(const std::vector<JointTrajPtMessage>& messages){

}

}
}
