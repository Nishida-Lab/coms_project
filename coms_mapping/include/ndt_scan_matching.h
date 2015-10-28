#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class NDTScanMatching
{
public:
	NDTScanMatching();
	void scan_matching_callback(const sensor_msgs::PointCloud2::ConstPtr& points,
								const geometry_msgs::PoseStamped::ConstPtr& pose);

private:
    ros::NodeHandle nh_;
	ros::Rate rate_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
	message_filters::Subscriber<geometry_msgs::PoseStamped> odom_sub_;
	TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> sync_;
	

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

};
