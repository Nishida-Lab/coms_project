#include <ndt_scan_matching.h>

NDTScanMatching::NDTScanMatching()
	: rate_(20), point_cloud_sub_(nh_, "/velodyne_points", 1),
	  odom_sub_(nh_, "/slam_out_pose", 1),
	  sync_(point_cloud_sub_, odom_sub_, 10)
{
	ros::NodeHandle n("~");
	// rosparam の設定
	sync_.registerCallback(boost::bind(&NDTScanMatching::scan_matching_callback, this, _1, _2));


	filtered_cloud_ = new pcl::PointCloud<pcl::PointXYZ>;

}

void NDTScanMatching::scan_matching_callback(const sensor_msgs::PointCloud2::ConstPtr& points,
											 const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	ros::Time scan_time;

    pcl::PointXYZI p; 
    pcl::PointCloud<pcl::PointXYZI> scan;

	tf::Quaternion q;
    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
    static tf::TransformBroadcaster br;
    tf::Transform transform;

	pcl::fromROSMsg(*points, scan);

	pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    // Filtering input scan to roughly 10% of original size to increase speed of registration.
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud (scan_ptr);
	approximate_voxel_filter.filter (*filtered_cloud);
	
	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt.setTransformationEpsilon (0.01);
	// Setting maximum step size for More-Thuente line search.
	ndt.setStepSize (0.1);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution (1.0);
	
	// Setting max number of registration iterations.
	ndt.setMaximumIterations (35);
	
	// Setting point cloud to be aligned.
	ndt.setInputSource (filtered_cloud);

	pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
	
	// Setting point cloud to be aligned to.
	ndt_.setInputTarget (map_ptr);

	tf::Matrix3x3 init_rotation;

	// 一個前のposeと引き算してx, y ,zの偏差を出す
    
    guess_pos.x = previous_pos.x + offset_x;
    guess_pos.y = previous_pos.y + offset_y;
    guess_pos.z = previous_pos.z + offset_z;
    guess_pos.roll = previous_pos.roll;
    guess_pos.pitch = previous_pos.pitch;
    guess_pos.yaw = previous_pos.yaw + offset_yaw;
    
    Eigen::AngleAxisf init_rotation_x(guess_pos.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pos.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pos.yaw, Eigen::Vector3f::UnitZ());
    
    Eigen::Translation3f init_translation(guess_pos.x, guess_pos.y, guess_pos.z);
    
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	
}


static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{

  //  if(_scanner == "velodyne"){
    ros::Time scan_time;

    pcl::PointXYZI p; 
    pcl::PointCloud<pcl::PointXYZI> scan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr (new pcl::PointCloud<pcl::PointXYZI>());
    tf::Quaternion q;
    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    
    //  scan.header = input->header;
    
    //     /*
    //       std::cout << "scan.header.stamp: " << scan.header.stamp << std::endl;
    //       std::cout << "scan_time: " << scan_time << std::endl;
    //       std::cout << "scan_time.sec: " << scan_time.sec << std::endl;
    //       std::cout << "scan_time.nsec: " << scan_time.nsec << std::endl;
    //     */
    
    //     t1_start = ros::Time::now();

    /*
      for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = input->begin(); item != input->end(); item++) {
      p.x = (double) item->x;
      p.y = (double) item->y;
      p.z = (double) item->z;
      p.intensity = (float) item->intensity;
      
      double r = p.x * p.x + p.y * p.y;
      if(r >= RADIUS){
      scan.points.push_back(p);
      }
      }
    */
    pcl::fromROSMsg(*input, scan);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    
    scan_time.sec = scan.header.stamp / 1000000.0;
    scan_time.nsec = (scan.header.stamp - scan_time.sec * 1000000.0) * 1000.0;
    
    
    // Add initial point cloud to velodyne_map
    if(initial_scan_loaded == 0){
      map += *scan_ptr;
      initial_scan_loaded = 1;
      std::cout << "initial_scan_loaded." << std::endl;
    }
    
    // Apply voxelgrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);
    
    // Matching with map
    ndt.setTransformationEpsilon(trans_eps);
    ndt.setStepSize(step_size);
    ndt.setResolution(ndt_res);
    ndt.setMaximumIterations(iter);
    
    ndt.setInputSource(filtered_scan_ptr);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
    map_ptr->header.frame_id = "map";
    
    // For future improvement
    // Apply VoxelGrid Filter if # of map points > 500,000
    /*
      std::cout << "Map points: " << map.points.size() << std::endl;
      if(map.points.size() > 500000){
      pcl::VoxelGrid<pcl::PointXYZI> map_filter;
      map_filter.setLeafSize(0.2, 0.2, 0.2);
      map_filter.setInputCloud(map_ptr);
      map_filter.filter(*map_ptr);
      map.clear();
      std::cout << "Map cleared" << std::endl;
      map += *map_ptr;
      }
    */
    
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(map_ptr);
    
    tf::Matrix3x3 init_rotation;
    
    guess_pos.x = previous_pos.x + offset_x;
    guess_pos.y = previous_pos.y + offset_y;
    guess_pos.z = previous_pos.z + offset_z;
    guess_pos.roll = previous_pos.roll;
    guess_pos.pitch = previous_pos.pitch;
    guess_pos.yaw = previous_pos.yaw + offset_yaw;
    
    Eigen::AngleAxisf init_rotation_x(guess_pos.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pos.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pos.yaw, Eigen::Vector3f::UnitZ());
    
    Eigen::Translation3f init_translation(guess_pos.x, guess_pos.y, guess_pos.z);
    
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    
    t3_end = ros::Time::now();
    d3 = t3_end - t3_start;
    
    t4_start = ros::Time::now();
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt.align(*output_cloud, init_guess);
    
    t = ndt.getFinalTransformation();
    
    //  pcl::transformPointCloud(*filtered_scan_ptr, *transformed_scan_ptr, t);
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t);
    
    tf::Matrix3x3 tf3d;
    
    tf3d.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
		  static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
		  static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));
    
    // Update current_pos.
    current_pos.x = t(0, 3);
    current_pos.y = t(1, 3);
    current_pos.z = t(2, 3);
    tf3d.getRPY(current_pos.roll, current_pos.pitch, current_pos.yaw, 1);
    
    transform.setOrigin(tf::Vector3(current_pos.x, current_pos.y, current_pos.z));
    q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
    transform.setRotation(q);
    
    br.sendTransform(tf::StampedTransform(transform, scan_time, "map", "velodyne"));
    
    // Calculate the offset (curren_pos - previous_pos)
    offset_x = current_pos.x - previous_pos.x;
    offset_y = current_pos.y - previous_pos.y;
    offset_z = current_pos.z - previous_pos.z;
    offset_yaw = current_pos.yaw - previous_pos.yaw;
    
    // Update position and posture. current_pos -> previous_pos
    previous_pos.x = current_pos.x;
    previous_pos.y = current_pos.y;
    previous_pos.z = current_pos.z;
    previous_pos.roll = current_pos.roll;
    previous_pos.pitch = current_pos.pitch;
    previous_pos.yaw = current_pos.yaw;
    
    // Calculate the offset between added_pos and current_pos
    double offset = sqrt(pow(current_pos.x-added_pos.x, 2.0) + pow(current_pos.y-added_pos.y, 2.0));
    std::cout << "offset: " << offset << std::endl;
    
    if(offset >= THRESHOLD){
      map += *transformed_scan_ptr;
      added_pos.x = current_pos.x;
      added_pos.y = current_pos.y;
      added_pos.z = current_pos.z;
      added_pos.roll = current_pos.roll;
      added_pos.pitch = current_pos.pitch;
      added_pos.yaw = current_pos.yaw;
      std::cout << "add velodyne_points to map" << std::endl;
    }
    
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    
    ndt_map_pub.publish(*map_msg_ptr);
    
    q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = scan_time;
    current_pose_msg.pose.position.x = current_pos.x;
    current_pose_msg.pose.position.y = current_pos.y;
    current_pose_msg.pose.position.z = current_pos.z;
    current_pose_msg.pose.orientation.x =  q.x();
    current_pose_msg.pose.orientation.y =  q.y();
    current_pose_msg.pose.orientation.z =  q.z();
    current_pose_msg.pose.orientation.w =  q.w();
    
    current_pose_pub.publish(current_pose_msg);
    
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "Sequence number: " << input->header.seq << std::endl;
    std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
    std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
    std::cout << "map: " << map.points.size() << " points." << std::endl;
    std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
    std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
    std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << current_pos.x << ", " << current_pos.y << ", " << current_pos.z << ", " << current_pos.roll << ", " << current_pos.pitch << ", " << current_pos.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    
    count++;
    //  }
}

int main(int argc, char **argv)
{
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "NDT_MAPPING program coded by Yuki KITSUKAWA" << std::endl;
    std::cout << "---------------------------------------------" << std::endl;

    previous_pos.x = 0.0;
    previous_pos.y = 0.0;
    previous_pos.z = 0.0;
    previous_pos.roll = 0.0;
    previous_pos.pitch = 0.0;
    previous_pos.yaw = 0.0;

    current_pos.x = 0.0;
    current_pos.y = 0.0;
    current_pos.z = 0.0;
    current_pos.roll = 0.0;
    current_pos.pitch = 0.0;
    current_pos.yaw = 0.0;

    guess_pos.x = 0.0;
    guess_pos.y = 0.0;
    guess_pos.z = 0.0;
    guess_pos.roll = 0.0;
    guess_pos.pitch = 0.0;
    guess_pos.yaw = 0.0;

    added_pos.x = 0.0;
    added_pos.y = 0.0;
    added_pos.z = 0.0;
    added_pos.roll = 0.0;
    added_pos.pitch = 0.0;
    added_pos.yaw = 0.0;

    offset_x = 0.0;
    offset_y = 0.0;
    offset_z = 0.0;
    offset_yaw = 0.0;

    ros::init(argc, argv, "ndt_mapping");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // setting parameters
    private_nh.getParam("scanner", _scanner);
    std::cout << "scanner: " << _scanner << std::endl;

    /*
    ndt.setTransformationEpsilon(trans_eps);
    ndt.setStepSize(step_size);
    ndt.setResolution(ndt_res);
    ndt.setMaximumIterations(iter);
    */

    ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

    ros::Subscriber param_sub = nh.subscribe("config/ndt_mapping", 10, param_callback);
    ros::Subscriber output_sub = nh.subscribe("config/ndt_mapping_output", 10, output_callback);
    //    ros::Subscriber hokuyo_sub = nh.subscribe("hokuyo_3d/hokuyo_cloud2", 100000, hokuyo_callback);
    ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);

    ros::spin();

    return 0;
}
