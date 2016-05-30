#include <Outlier_Removal.h>
#include <cloud_base.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

//constructor
outlier_removal::outlier_removal(int number, double sor_coe)
{
  number_ = number;
  sor_coe_ = sor_coe;
}

//copy constructor
outlier_removal::outlier_removal(outlier_removal &obj)
{
	//ros_cloud_ = obj.ros_cloud_;
  cloud2_ = obj.cloud2_;
  cloud_ = obj.cloud_;
  number_ = obj.number_; 
  sor_coe_ = obj.sor_coe_;
}

//setter
void outlier_removal::setSORNumber(int number)
{number_ = number;}
void outlier_removal::setSORCoeffident(double sor_coe)
{sor_coe_ = sor_coe;}
//getter
int outlier_removal::getSORNumber()
{return number_;}
double outlier_removal::getSORCoeffident()
{return sor_coe_;}

//outlier removal 
void outlier_removal::applyOutlierRemoval()
{
  clock_t startTime , endTime; double margin = 0.0; //時間計測用変数
  startTime = clock();
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (cloud_);
  sor.setMeanK (number_);
  sor.setStddevMulThresh(sor_coe_);
  sor.filter (*cloud_);
  endTime = clock();
  cout << " outlier removing done...(size: " << cloud_->points.size() << ")" << endl;
  cout << " processing time:" << (double)(endTime - startTime)/CLOCKS_PER_SEC << "秒" << endl;
}

//鳥の巣削除
void outlier_removal::nearestRemove(double radius)
{
  // pcl::PointCloud<pcl::PointXYZ>::iterator Iter;
  // Iter = cloud_->points.begin();
  // for(Iter =cloud_->points.begin(); Iter < cloud_->points.end(); Iter++){
  // 	//double d=sqrt(pow(cloud_->points[Iter].x,2)+pow(cloud_->points[Iter].z,2));
  // 	double d=sqrt(pow((*Iter).x,2)+pow((*Iter).y,2));
  // 	if(d < radius){
  // 	  cloud_->points.erase(Iter);
  // 	  //cout<< d;
  // 	  //cout << "[" << "ayuge" << "]" << *Iter << endl;
  // 	  Iter--;
  // 	} 
  // }
  // cloud_->width = 1; 
  // cloud_->height = cloud_->points.size();

  pcl::PointCloud<PointT>::Ptr after_cloud(new pcl::PointCloud<PointT>);

  //kdtree
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud_);
  PointT searchPoint;
  searchPoint.x = 0.0f;
  searchPoint.y = 0.0f;
  searchPoint.z = 0.0f;
  searchPoint.intensity = 0.0f;

  //radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){}
  after_cloud->points.clear();
  sort(pointIdxRadiusSearch.begin(),pointIdxRadiusSearch.end());

  //値を入れていくお
  int pt_counter;
  pt_counter = 0;
  for(size_t i = 0; i < cloud_->points.size(); i++){ 
	if (i != pointIdxRadiusSearch[pt_counter]){
	  after_cloud->points.push_back( cloud_->points[i] );
	}
	else{
	  pt_counter++;
	  //std::cout << "pt_counter++\n";
	}
  }

  after_cloud->width = 1; 
  after_cloud->height = cloud_->points.size();
  after_cloud->points.resize(after_cloud->width * after_cloud->height);

  cloud_->points.clear();
  cloud_ = after_cloud;
  //return after_cloud;

}
