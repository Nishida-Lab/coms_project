#include <Plane_segment.h>

using namespace std;

plane_segment::plane_segment(double sac_param, float view_pos)
{
  sac_param_ = sac_param;
  view_pos_ = view_pos;
}

plane_segment::plane_segment(const plane_segment &obj)
{
	//ros_cloud_ = obj.ros_cloud_;
  cloud2_ = obj.cloud2_;
  cloud_ = obj.cloud_;
  sac_param_ = obj.sac_param_; 
}

void plane_segment::applySACSegmentation()
{
  //準備
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //外れ値番号格納 
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); //モデル格納
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //
  clock_t startTime , endTime; double margin = 0.0; //時間計測用変数
  //SACSegmentationオブジェクト
  pcl::SACSegmentation<PointT> seg;
  //オプション( true-> 平面を出す)
  seg.setOptimizeCoefficients (true);
  seg.setAxis(axis);
  //seg.setEpsAngle(20.0 * PI / 180.0);
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC); 
  seg.setDistanceThreshold (sac_param_);
  seg.setInputCloud (cloud_); 
  startTime = clock();
  seg.segment (*inliers, *coefficients);
  endTime = clock();
  //係数を表示
  // std::cerr << "Model coefficients: " 
  // 	    << coefficients->values[0] << " " 
  // 	    << coefficients->values[1] << " "
  // 	    << coefficients->values[2] << " " 
  // 	    << coefficients->values[3] << std::endl;
  cout << " plane segment done...(size: " << inliers->indices.size () << ")" << endl;
  cout << " processing time:" << (double)(endTime - startTime)/CLOCKS_PER_SEC << "秒" << endl;

  //erase
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud( cloud_ );
  extract.setIndices( inliers );
  extract.setNegative( true );
  startTime = clock();
  extract.filter( *cloud_ );
  endTime = clock();
  cout << " plane removing done...(size: " << cloud_->points.size () << ")" << endl;
  cout << " processing time:" << (double)(endTime - startTime)/CLOCKS_PER_SEC << "秒" << endl;

  //cropBoxFilter
  //最小値定義
  Eigen::Vector4f minPoint; 
  minPoint[0] = -50.0;  // define minimum point x 
  minPoint[1] = -50.0;  // define minimum point y 
  minPoint[2] = -view_pos_;  // define minimum point z 
  //最大値定義
  Eigen::Vector4f maxPoint; 
  maxPoint[0]= 50.0;  // define minimum point x 
  maxPoint[1]= 50.0;  // define max point y 
  maxPoint[2]= 1.0;  // define max point z
  //回転角度[rad]
  Eigen::Vector3f boxRotation; 
  boxRotation[0]=0;  // rotation around x-axis 
  boxRotation[1]=0;  // rotation around y-axis 
  boxRotation[2]=0;  //in radians rotation around z-axis 
  //フィルタリング開始
  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud (cloud_); //元データセット 
  cropFilter.setMin(minPoint); //最小値セット
  cropFilter.setMax(maxPoint); //最大値セット 
  cropFilter.setRotation(boxRotation);
  startTime = clock();
  cropFilter.filter (*cloud_);
  endTime = clock();
  cout << " crop box filter done...(size: " << cloud_->points.size() << ")" << endl;
  cout << " processing time:" << (double)(endTime - startTime)/CLOCKS_PER_SEC << "秒" << endl;
}
