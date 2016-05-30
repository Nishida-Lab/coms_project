// #include <Outlier_Removal.h>
// #include <cloud_base.h>
// #include <Plane_segment.h>
// #include <Bounding_Box.h>
// #include <Superimposing.h>

int main(){
	return 0;
}

// void 
// cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
// {
//   cout << "[callback:" << counter<< "]" << endl;
//   //output object
//   pcl::PCLPointCloud2 output;
//   visualization_msgs::MarkerArray marker_output;
//   cout << "0)create ransac object" << endl;
//   plane_segment plane_seg(0.2, 1.5);

//   cout << "1)convert to PointCloudPtr" << endl;
//   plane_seg.convertROStoPCL(cloud);

//   cout << "2)plane segment" << endl;
//   plane_seg.applySACSegmentation();

//   cout << "3)outlier removal" << endl;
//   outlier_removal cloud_proc(50, 1.0);
//   cloud_proc.setPCLCloud( plane_seg.getPCLCloud() );
//   cloud_proc.applyOutlierRemoval();
//   if(counter == 0)
// 	{
// 	  pcl::io::savePCDFileASCII("1frame_velodyne.pcd", *cloud_proc.getPCLCloud());
// 	  cout << "first save done..." << endl;
// 	}

//   cout << "4)nearest removal" << endl;
//   cloud_proc.nearestRemove(1.8);

//   cout << "5)clustering" << endl;
//   float tolerance=1.0; 
//   int min_size=50; 
//   int max_size=2500;
//   bounding_box bou_box(tolerance, min_size, max_size);
//   bou_box.setPCLCloud( cloud_proc.getPCLCloud() );
//   bou_box.applyBoundingBox();
//   cout << " cluster:" << bou_box.getClusterNumber() << endl;
  
//   cout << "6)convert to PCLPointCloud2" << endl;
//   output = bou_box.convertPCLtoPCL2();
//   marker_output = bou_box.createBox();

//   //Publish the data
//   pub.publish (output);
//   cout << "7)publish point cloud！" << endl;
//   marker_pub.publish (marker_output);
//   marker_output.markers.clear(); 
//   cout << "8)publish box！" << endl;
//   //marker_output.markers.clear();
//   counter++;
  
// }


// int
// main (int argc, char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "my_pcl_tutorial");
//   ros::NodeHandle nh;
//   ros::NodeHandle nh_marker;
//   // Create a ROS subscriber for the input point cloud
//   ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

//   // Create a ROS publisher for the output point cloud
//   pub = nh.advertise<pcl::PCLPointCloud2> ("downsamp_noisecut", 1);
//   //marker_pub = nh_marker.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
//   marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

//   // Spin
//   ros::spin ();
// }
