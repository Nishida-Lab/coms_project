#include <ndt_scan_matching.h>

NDTScanMatching::NDTScanMatching()
	: rate_(1), point_cloud_sub_(nh_, "/velodyne_points", 1),
	  odom_sub_(nh_, "/slam_out_pose", 1),
	  sync_(point_cloud_sub_, odom_sub_, 10)
{
	ros::NodeHandle n("~");
	// rosparam の設定
	sync_.registerCallback(boost::bind(&NDTScanMatching::scan_matching_callback, this, _1, _2));

	point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan_match_point_cloud", 1);

	//filtered_cloud_ = new pcl::PointCloud<pcl::PointXYZI>();

	offset_x_ = 0;
	offset_y_ = 0;
	offset_z_ = 0;
	offset_yaw_ = 0;

	last_yaw_ = 0;

	current_pos_.x = 0;
	current_pos_.y = 0;
	current_pos_.z = 0;
	current_pos_.roll = 0;
	current_pos_.pitch = 0;
	current_pos_.yaw = 0;

	previous_pos_.x = 0;
	previous_pos_.y = 0;
	previous_pos_.z = 0;
	previous_pos_.roll = 0;
	previous_pos_.pitch = 0;
	previous_pos_.yaw = 0;

	guess_pos_.x = 0;
	guess_pos_.y = 0;
	guess_pos_.z = 0;
	guess_pos_.roll = 0;
	guess_pos_.pitch = 0;
	guess_pos_.yaw = 0;

	initial_scan_loaded_ = 0;
	count_ = 0;
}

void NDTScanMatching::getRPY(const geometry_msgs::Quaternion &q,
							 double &roll,double &pitch,double &yaw){
	tf::Quaternion tfq(q.x, q.y, q.z, q.w);
	tf::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
}


void NDTScanMatching::scan_matching_callback(const sensor_msgs::PointCloud2::ConstPtr& points,
											 const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	ros::Time scan_time = ros::Time::now();

    pcl::PointXYZI p; 
    pcl::PointCloud<pcl::PointXYZI> scan;
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr (new pcl::PointCloud<pcl::PointXYZI>());
	tf::Quaternion q;
    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
    
    tf::Transform transform;

	pcl::fromROSMsg(*points, scan);

	pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

	if(initial_scan_loaded_ == 0)
	{
		last_scan_ = *scan_ptr;
		last_pose_ = *pose;
		initial_scan_loaded_ = 1;
		return;
	}
    // Filtering input scan to roughly 10% of original size to increase speed of registration.
	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (2.0, 2.0, 2.0);
	approximate_voxel_filter.setInputCloud (scan_ptr);
	approximate_voxel_filter.filter (*filtered_cloud_ptr);
	
	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt_.setTransformationEpsilon (0.01);
	// Setting maximum step size for More-Thuente line search.
	ndt_.setStepSize (0.1);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt_.setResolution (1.0);
	
	// Setting max number of registration iterations.
	ndt_.setMaximumIterations (30);
	
	// Setting point cloud to be aligned.
	ndt_.setInputSource (filtered_cloud_ptr);

	pcl::PointCloud<pcl::PointXYZI>::Ptr last_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(last_scan_));
	
	// Setting point cloud to be aligned to.
	ndt_.setInputTarget (last_scan_ptr);

	tf::Matrix3x3 init_rotation;

	// 一個前のposeと引き算してx, y ,zの偏差を出す
	offset_x_ = pose->pose.position.x - last_pose_.pose.position.x;
	offset_y_ = pose->pose.position.y - last_pose_.pose.position.y;
	double roll, pitch, yaw = 0;
	getRPY(pose->pose.orientation, roll, pitch, yaw);
	offset_yaw_ = yaw - last_yaw_;

    guess_pos_.x = previous_pos_.x + offset_x_;
    guess_pos_.y = previous_pos_.y + offset_y_;
    guess_pos_.z = previous_pos_.z;
    
	guess_pos_.roll = previous_pos_.roll;
    guess_pos_.pitch = previous_pos_.pitch;
    guess_pos_.yaw = previous_pos_.yaw + offset_yaw_;
    
    Eigen::AngleAxisf init_rotation_x(guess_pos_.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pos_.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pos_.yaw, Eigen::Vector3f::UnitZ());
    
    Eigen::Translation3f init_translation(guess_pos_.x, guess_pos_.y, guess_pos_.z);
    
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

	ros::Time ndt_start = ros::Time::now();
	ndt_.align (*output_cloud_ptr, init_guess);
	ros::Duration ndt_delta_t = ros::Time::now() - ndt_start;	
	
	std::cout << "Normal Distributions Transform has converged:" << ndt_.hasConverged ()
			  << " score: " << ndt_.getFitnessScore () << std::endl;

	t = ndt_.getFinalTransformation();

	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud (*scan_ptr, *output_cloud_ptr, t);
  
	tf::Matrix3x3 tf3d;
	tf3d.setValue(
		static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
		static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
		static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));
  
	current_pos_.x = t(0, 3);
	current_pos_.y = t(1, 3);
	current_pos_.z = t(2, 3);

	std::cout << "/////////////////////////////////////////////" << std::endl;
	std::cout << "count : " << count_ << std::endl;
	std::cout << "Process time : " << ndt_delta_t.toSec() << std::endl;
	std::cout << "x : " << current_pos_.x << std::endl;
	std::cout << "y : " << current_pos_.y << std::endl;
	std::cout << "z : " << current_pos_.z << std::endl;
	std::cout << "/////////////////////////////////////////////" << std::endl;

	tf3d.getRPY(current_pos_.roll, current_pos_.pitch, current_pos_.yaw, 1);
  
	transform.setOrigin(tf::Vector3(current_pos_.x, current_pos_.y, current_pos_.z));
	q.setRPY(current_pos_.roll, current_pos_.pitch, current_pos_.yaw);
	transform.setRotation(q);


	// "map"に対する"base_link"の位置を発行する
	br_.sendTransform(tf::StampedTransform(transform, scan_time, "map", "ndt_base_link"));

	sensor_msgs::PointCloud2 scan_matched;
	pcl::toROSMsg(*output_cloud_ptr, scan_matched);

	scan_matched.header.stamp = scan_time;
	scan_matched.header.frame_id = "matched_point_cloud";
	
	point_cloud_pub_.publish(scan_matched);

	// Update position and posture. current_pos -> previous_pos
	previous_pos_ = current_pos_;

	// save current scan
	last_scan_ += *output_cloud_ptr;
	last_pose_ = *pose;
	last_yaw_ = yaw;
	// geometry_msgs::TransformStamped ndt_trans;
	// ndt_trans.header.stamp = scan_time;
	// ndt_trans.header.frame_id = "map";
	// ndt_trans.child_frame_id = "base_link";
  
	// ndt_trans.transform.translation.x = curren_pos.x;
	// ndt_trans.transform.translation.y = curren_pos.y;
	// ndt_trans.transform.translation.z = curren_pos.z;
	// ndt_trans.transform.rotation = q;
  
	// ndt_broadcaster_.sendTransform(ndt_trans); 
	count_++;
}


