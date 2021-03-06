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

#include "joint_trajectory_full_downloader.h"

namespace industrial_robot_client
{
namespace joint_trajectory_downloader
{

using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
using industrial::simple_message::SimpleMessage;

bool JointTrajectoryFullDownloader::send_to_robot(const std::vector<JointTrajPtFullMessage>& messages){

  bool rslt=true;
  std::vector<JointTrajPtFullMessage> points(messages);
  SimpleMessage msg;

  // Trajectory download requires at least two points (START/END)
  if (points.size() < 2)
    points.push_back(JointTrajPtFullMessage(points[0]));

  // The first and last points are assigned special sequence values
  points.begin()->setSequence(-1);
  points.back().setSequence(-3);

  if (!this->connection_->isConnected())
  {
    ROS_WARN("Attempting robot reconnection");
    this->connection_->makeConnect();
  }

  ROS_INFO("Sending trajectory points, size: %d", (int)points.size());

  for (int i = 0; i < (int)points.size(); ++i)
  {
    ROS_DEBUG("Sending joints trajectory point[%d]", i);

	float time;
	points[i].point_.getTime(time);

    ROS_DEBUG("Sending time = %f", time);
    points[i].toTopic(msg);
    bool ptRslt = this->connection_->sendMsg(msg);
    if (ptRslt)
      ROS_DEBUG("Point[%d] sent to controller", i);
    else
      ROS_WARN("Failed sent joint point, skipping point");

    rslt &= ptRslt;
  }

  return rslt;

}

}	// namespace industrial_robot_client
}	// namespace joint_trajectory_downloader
