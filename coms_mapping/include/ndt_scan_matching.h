#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
/* #include <velodyne_pointcloud/point_types.h> */
/* #include <velodyne_pointcloud/rawdata.h> */

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

struct Position {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class NDTScanMatching
{
public:
	NDTScanMatching();
	void scan_matching_callback(const sensor_msgs::PointCloud2::ConstPtr& points,
								const geometry_msgs::PoseStamped::ConstPtr& pose);
	void getRPY(const geometry_msgs::Quaternion &q,
				double &roll,double &pitch,double &yaw);

private:
    ros::NodeHandle nh_;
	ros::Rate rate_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
	message_filters::Subscriber<geometry_msgs::PoseStamped> odom_sub_;
	message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> sync_;
	
	ros::Publisher point_cloud_pub_;

	pcl::PointCloud<pcl::PointXYZI> last_scan_;

	pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

	double offset_x_;
	double offset_y_;
	double offset_z_;
	double offset_yaw_;

	double last_yaw_;

	Position current_pos_;
	Position previous_pos_;
	Position guess_pos_;

	geometry_msgs::PoseStamped last_pose_;
	tf::TransformBroadcaster br_;
	
	int initial_scan_loaded_;
	int count_;
};
