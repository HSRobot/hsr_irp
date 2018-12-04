#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
  std::stringstream compressedData;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudEncoder;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudDecoder;
  pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

  PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ> (compressionProfile, true);
  PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ> ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 10000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 1; i < (cloud->points.size ()); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cout << cloud->points[0].x << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ> ());

  // compress point cloud
  PointCloudEncoder->encodePointCloud (cloud, compressedData);

  // decompress point cloud
  PointCloudDecoder->decodePointCloud (compressedData, cloudOut);

  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloudOut, ros_cloud);
  ros_cloud.header.frame_id = "world";

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    //pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (ros_cloud);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
