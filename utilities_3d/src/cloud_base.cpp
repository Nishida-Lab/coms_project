#include <cloud_base.h>

cloud_base::cloud_base()
{}

cloud_base::cloud_base(const cloud_base &obj)
{
  ros_cloud_ = obj.ros_cloud_;
  cloud2_ = obj.cloud2_;
  cloud_ = obj.cloud_;
}

//setter
void cloud_base::setROSCloud(sensor_msgs::PointCloud2ConstPtr ros_cloud)
{ros_cloud_ = ros_cloud;}
void cloud_base::setPCLCloud2(pcl::PCLPointCloud2Ptr cloud2)
{cloud2_ = cloud2;}
void cloud_base::setPCLCloud(pcl::PointCloud<PointT>::Ptr cloud)
{cloud_ = cloud;}

//getter
sensor_msgs::PointCloud2ConstPtr cloud_base::getROSCloud()
{return ros_cloud_;}
pcl::PCLPointCloud2Ptr cloud_base::getPCLCloud2()
{return cloud2_;}
pcl::PointCloud<PointT>::Ptr cloud_base::getPCLCloud()
{return cloud_;}


//convert
//sensor_msgs::PointCloud2ConstPtr --> pcl::PointCloudT::Ptr
pcl::PointCloud<PointT>::Ptr cloud_base::convertROStoPCL(sensor_msgs::PointCloud2ConstPtr ros_source)
{
  pcl::PCLPointCloud2 pcl_cloud2;
  pcl::PointCloud<PointT> pcl_cloud;
  pcl_conversions::toPCL(*ros_source, pcl_cloud2);
  //PCLPointCloud2 --> PointCloudT
  pcl::fromPCLPointCloud2(pcl_cloud2,pcl_cloud);
  //PointCloudT --> PointCloudTPtr
  pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>(pcl_cloud));
  cloud_ = cloud_ptr;
  return cloud_ptr;
}

//pcl::PointCloudT::Ptr --> sensor_msgs::PointCloudConst2(rosへ出力するため)
pcl::PCLPointCloud2 cloud_base::convertPCLtoPCL2()
{
  pcl::PCLPointCloud2 output;
  toPCLPointCloud2(*cloud_, output);
  return output;
}

//pcl::PCLPointCloud2 --> pcl::PointCloudTPtr
pcl::PointCloud<PointT>::Ptr cloud_base::convertPCL2toPCL(pcl::PCLPointCloud2ConstPtr source)
{
  //PCLPointCloud2 --> PointCloudT
  pcl::fromPCLPointCloud2(*source, *cloud_);
  return cloud_;
}
