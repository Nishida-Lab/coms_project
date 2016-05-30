#ifndef OUTLIER_H_
#define OUTLIER_H_

#include <time.h>
#include <iostream>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <cloud_base.h>

typedef pcl::PointXYZI PointT;

class outlier_removal 
:public cloud_base
{
 public:
  int number_;
  double sor_coe_;

 public:
  //constructor
  outlier_removal(int number, double sor_coe);
  //copy constructor
  outlier_removal(outlier_removal &obj);
  //setter
  void setSORNumber(int number);
  void setSORCoeffident(double sor_coe);
  //getter
  int getSORNumber();
  double getSORCoeffident();
  void applyOutlierRemoval();
  pcl::PointCloud<pcl::PointXYZ>::Ptr nearestRemove(double radius);
  void nearestRemove(double radius);
};

#endif
