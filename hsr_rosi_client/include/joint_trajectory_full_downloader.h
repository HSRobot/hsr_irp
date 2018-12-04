#ifndef JOINT_TRAJECTORY_FULL_DOWNLOADER_H
#define JOINT_TRAJECTORY_FULL_DOWNLOADER_H

#include "joint_trajectory_full_interface.h"

namespace industrial_robot_client{
namespace joint_trajectory_downloader{

using industrial_robot_client::joint_trajectory_interface::JointTrajectoryFullInterface;
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;

class JointTrajectoryFullDownloader : public JointTrajectoryFullInterface{

public:

  bool send_to_robot(const std::vector<JointTrajPtFullMessage>& messages);

};

}	//namespace joint_trajectory_downloader
}	//namespace industrial_robot_client

#endif
