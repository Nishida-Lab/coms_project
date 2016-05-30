#ifndef SUPERIMPOSING_H_
#define SUPERIMPOSING_H_

#include <time.h>
#include <fstream>
#include <iostream>
 
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include <cloud_base.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>

typedef pcl::PointXYZI PointT;

class superImposing
:public cloud_base
{
protected:
  pcl::PointCloud<PointT>::Ptr pre_cloud_;
  pcl::PointCloud<PointT>::Ptr source_cloud1_;
  pcl::PointCloud<PointT>::Ptr source_cloud2_;
  pcl::PointCloud<PointT>::Ptr result_cloud_;
  int num_of_frame_;
  double attention_area_;
  double superimposing_area_;
  double attention_width_; 
  double freq_;
  std::string save_folder_pass_;
  double speed_;
  double angle_;
  double position_;
  double ndt_epsilon_;
  double ndt_step_size_;
  double ndt_resolution_;
  double ndt_leafsize_;
  int ndt_max_iterations_;

public:
  //constructor
  superImposing(pcl::PointCloud<PointT>::Ptr cloud1, pcl::PointCloud<PointT>::Ptr cloud2);
  //copy constructor
  superImposing(superImposing &obj);
  //setter
  void setFrame1(pcl::PointCloud<PointT>::Ptr source_cloud1);
  void setFrame2(pcl::PointCloud<PointT>::Ptr source_cloud2);
  void setParam_superimposing();//via txt file
  void setParam_odom();//via txt file
  void setParam_NDT();//via txt file

  //getter
  pcl::PointCloud<PointT>::Ptr getFrame1();
  pcl::PointCloud<PointT>::Ptr getFrame2();
  pcl::PointCloud<PointT>::Ptr getResult();

  /*donut filter-------
  (I)PointCloudPtr
  (I)far radius
  (I)near radius
  (O)PointCloudPtr
  ---------------------*/
  pcl::PointCloud<PointT>::Ptr nearRemoval(pcl::PointCloud<PointT>::Ptr input_cloud, double radius_f, double radius_n);

  /*side cut filter-----
  (I)PointCloudPtr
  (I)effective width
  (O)PointCloudPtr
  ---------------------*/
  pcl::PointCloud<PointT>::Ptr sideCut(pcl::PointCloud<PointT>::Ptr input_cloud, double width);

  /*cropBoxFilter------------------------------
  (I)PointCloudPtr
  (I)x_min, x_max, y_min, y_max, z_min, z_max
  (O)PointCloudPtr
  --------------------------------------------*/
  pcl::PointCloud<PointT>::Ptr
	crop_box_filter(pcl::PointCloud<PointT>::Ptr in_cloud, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);

  /*simply estimate position--------
  (I)velodyne freqency[Hz]
  (I)car speed[km/h]
  (O)position
  ---------------------------------*/
  double estimatePosition(double freq, double speed);

  /*superimposing base on estimation of movement------
  (I)member variable
  (O)PointCloudPtr(member variable)
  ---------------------------------------------------*/
  pcl::PointCloud<PointT>::Ptr applyPreTreatment();

  void applyAfterTreatment();

};

#endif
