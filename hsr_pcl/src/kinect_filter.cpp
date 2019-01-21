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
