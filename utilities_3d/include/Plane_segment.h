#ifndef PLANESEGMENT_H_
#define PLANESEGMENT_H_

#include <cloud_base.h>
#include <time.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

typedef pcl::PointXYZI PointT;

class plane_segment 
:public cloud_base
{
 protected:
  double sac_param_;
  float view_pos_;
 public:
  //constructor
  plane_segment(double sac_param, float view_pos);
  //copy constructor
  plane_segment(const plane_segment &obj);
  //apply sac_segmentation
  void applySACSegmentation();
};

#endif
