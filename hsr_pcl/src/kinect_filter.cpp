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

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <sstream>
#include <hsr_pcl/LoadPCD.h>
#include <hsr_pcl/SavePCD.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ> ());
ros::Publisher pub;

bool flage = false;

/**
    save the point cloud as pcd file
 * */
int savePCD(std::string fileName){
    pcl::io::savePCDFileASCII (fileName, *cloudOut);
    ROS_INFO("save pcd success");
}

/**
    load the point clound from pcd file
 **/
int loadPCD(std::string fileName){
    pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloudOut);
    ROS_INFO("load pcd success");
}

void pubPointCloud(){
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloudOut, ros_cloud);
    ros_cloud.header.frame_id = "world";
    pub.publish(ros_cloud);
}

bool saveSrvCb(hsr_pcl::SavePCD::Request  &req,
         hsr_pcl::SavePCD::Response &res)
{
  savePCD(req.fileName);
  ROS_INFO("request: saveing point cloud");
  return true;
}

bool loadSrvCb(hsr_pcl::LoadPCD::Request  &req,
         hsr_pcl::LoadPCD::Response &res)
{
  loadPCD(req.fileName);
  pubPointCloud();
  ROS_INFO("request: loading point cloud");
  return true;
}




void kinect_data_cb(const sensor_msgs::PointCloud2::ConstPtr & msg){

    if(flage)
        return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    const sensor_msgs::PointCloud2 tmp = *msg;
    pcl::fromROSMsg(tmp, *cloud);

    cloudOut->width = cloud->width;
    cloudOut->height = cloud->height;
    cloudOut->points.resize (cloudOut->width * cloudOut->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
        if(cloud->points[i].z < 0.5)
            cloudOut->points[i] =cloud->points[i];

    pubPointCloud();
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "kinect_filter");
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filter_points", 1);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/points2", 1, kinect_data_cb);
  ros::ServiceServer saveSrv = nh.advertiseService("savePCD", saveSrvCb);
  ros::ServiceServer loadSrv = nh.advertiseService("loadPCD", loadSrvCb);
  ros::spin();
}
