#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cloud_base.h>


#include <pcl/features/moment_of_inertia_estimation.h>

class bounding_box 
:public cloud_base
{
 protected:
  //Euclidean Cluster Extraction
  float tolerance_;
  int min_size_;
  int max_size_;
//visualization_msgs::MarkerArray marker_omega;

 public:
  int cluster;
  //最大座標格納用vector
  std::vector<float> max_positionX;
  std::vector<float> max_positionY;
  std::vector<float> max_positionZ;
  //最小座標格納用vector
  std::vector<float> min_positionX;
  std::vector<float> min_positionY;
  std::vector<float> min_positionZ;
  //回転したポイントクラウド格納用
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> rotatedCloud;
  std::vector<float> r_center_x; //補完する際の回転中心(x)
  std::vector<float> r_center_y; //補完する際の回転中心(y)
  std::vector<float> r_center_z; //補完する際の回転中心(z)
  std::vector<float> grad_box; //箱の傾き
  std::vector<float> length_scall; //箱の横の長さ
  std::vector<float> width_scall; //箱の奥行きの長さ
  std::vector<float> height_scall; //箱の縦の長さ
  //ナイスポイント格納用
  std::vector<std::vector<float> > nice_positionX;
  std::vector<std::vector<float> > nice_positionY;
  std::vector<std::vector<float> > nice_positionZ;

  //member function
  bounding_box(float tolerance, int min_size, int max_size);
  bounding_box(const bounding_box &obj);
  std::vector<pcl::PointIndices> applyClustering();
  void applyBoundingBox();
  void applyBoundingBox_using_inertia();
  int getClusterNumber();
  //visualization_msgs::MarkerArray createBox();
  void Show_result(pcl::PointCloud<PointT>::Ptr in_cloud);
};

#endif
