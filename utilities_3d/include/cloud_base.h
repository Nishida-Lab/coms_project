#ifndef CLOUDBASE_H_
#define CLOUDBASE_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLImage.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/exceptions.h>
#include <pcl/console/print.h>
#include <boost/foreach.hpp>

typedef pcl::PointXYZI PointT;

class cloud_base
{
 protected:
  sensor_msgs::PointCloud2ConstPtr ros_cloud_;
  pcl::PCLPointCloud2Ptr cloud2_;
  pcl::PointCloud<PointT>::Ptr cloud_;

 public:
  //default constructor
  cloud_base();
  //copy constructor
  cloud_base(const cloud_base &obj);
  //setter
  void setROSCloud(sensor_msgs::PointCloud2ConstPtr ros_cloud);
  void setPCLCloud2(pcl::PCLPointCloud2Ptr cloud2);
  void setPCLCloud(pcl::PointCloud<PointT>::Ptr cloud);
  //getter
  sensor_msgs::PointCloud2ConstPtr getROSCloud();
  pcl::PCLPointCloud2Ptr getPCLCloud2();
  pcl::PointCloud<PointT>::Ptr getPCLCloud();
  //convert
  pcl::PointCloud<PointT>::Ptr convertROStoPCL(sensor_msgs::PointCloud2ConstPtr);
  pcl::PointCloud<PointT>::Ptr convertPCL2toPCL(pcl::PCLPointCloud2ConstPtr source);
  pcl::PCLPointCloud2 convertPCLtoPCL2();  
};

#endif
