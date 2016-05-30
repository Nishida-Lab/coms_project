#include <Bounding_Box.h>

typedef pcl::PointXYZI PointT;
using namespace std;

int Pattern = 0;

//最大値を求める**********
float max(float p1, float p2, float p3)
{
  float max;
  max = p1; Pattern = 1;
  if (max < p2) {max = p2; Pattern = 2;}
  if (max < p3) {max = p3; Pattern = 3;}
  return max;
}

bounding_box::bounding_box(float tolerance, int min_size, int max_size)
{
  tolerance_ = tolerance;
  min_size_ = min_size;
  max_size_ = max_size;
}

bounding_box::bounding_box(const bounding_box &obj)
{
  ros_cloud_ = obj.ros_cloud_;
  cloud2_ = obj.cloud2_;
  cloud_ = obj.cloud_;
  tolerance_ = obj.tolerance_;
  min_size_ = obj.min_size_;
  max_size_ = obj.max_size_;  
}

vector<pcl::PointIndices> bounding_box::applyClustering()
{
  clock_t startTime, endTime;
  double margin;
  std::vector<pcl::PointIndices> cluster_indices_;
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_);
  //ユークリッドクラスタ抽出
  startTime = clock();
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (tolerance_); // 許容誤差
  ec.setMinClusterSize (min_size_); //クラスタ最小サイズ 
  ec.setMaxClusterSize (max_size_); //クラスタ最大サイズ
  ec.setSearchMethod (tree); //kdtree置いとくで
  ec.setInputCloud (cloud_); //ソースデータのセット
  ec.extract (cluster_indices_); //抽出処理
  endTime = clock();
  cout << "--->clustering done..." << endl;
  margin = (double)(endTime - startTime) / CLOCKS_PER_SEC;
  cout << " processing time:" << margin << "sec" << endl;
  return cluster_indices_;
}

void bounding_box::applyBoundingBox()
{
  //PointIndices in vector
  std::vector<pcl::PointIndices> cluster_indices;
  //clustering
  cluster_indices = applyClustering();
  //保存
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
    {    
      //抽出したポイントクラウドを格納する型
	  pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      //cloud_clusterを回転したものを入れる型
	  pcl::PointCloud<PointT>::Ptr r_cloud_cluster (new pcl::PointCloud<PointT>);
	  
      //抽出したポイントクラウドを格納
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		cloud_cluster->points.push_back (cloud_->points[*pit]); //*
 
      //リサイズとsave
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
	  //pcl::io::savePCDFileASCII("cluster.pcd" ,*cloud_cluster);

      //最大最小座標格納
      PointT min_pt , max_pt;
      //最大最小座標をを求める
      pcl::getMinMax3D (*cloud_cluster , min_pt, max_pt);
      //座標を格納
      max_positionX.push_back (max_pt.x); //x_max
      max_positionY.push_back (max_pt.y); //y_max
      max_positionZ.push_back (max_pt.z); //z_max
      min_positionX.push_back (min_pt.x); //x_min
      min_positionY.push_back (min_pt.y); //y_min
      min_positionZ.push_back (min_pt.z); //z_min

      //いい感じの直線を出す
      //その前に点を出す
      PointT p_xmax, p_xmin, p_ymax, p_ymin, r_center; //p_zmax->p_ymax
      for (int i = 0; i < cloud_cluster->points.size(); i++)
		{
		  if (cloud_cluster->points[i].x == max_pt.x)
			{
			  p_xmax.x = max_pt.x;
			  p_xmax.y = cloud_cluster->points[i].y;
			  p_xmax.z = cloud_cluster->points[i].z;
			}
		  if (cloud_cluster->points[i].x == min_pt.x)
			{
			  p_xmin.x = min_pt.x;
			  p_xmin.y = cloud_cluster->points[i].y;
			  p_xmin.z = cloud_cluster->points[i].z;
			}
		  if (cloud_cluster->points[i].y == max_pt.y)
			{
			  p_ymax.x = cloud_cluster->points[i].x;
			  p_ymax.y = max_pt.y;
			  p_ymax.z = cloud_cluster->points[i].z;
			}
		  if (cloud_cluster->points[i].y == min_pt.y)
			{
			  p_ymin.x = cloud_cluster->points[i].x;
			  p_ymin.y = min_pt.y;
			  p_ymin.z = cloud_cluster->points[i].z;
			}
		}

      //さっき求めた点同士の距離とか中心出していこうや
      float d_xmin_xmax, d_xmin_ymax, d_xmax_ymax, max_distance;
	  float grad;
      Eigen::Vector3f diff1(p_xmin.x-p_xmax.x, p_xmin.y-p_xmax.y, p_xmin.z-p_xmax.z);
      Eigen::Vector3f diff2(p_xmin.x-p_ymax.x, p_xmin.y-p_ymax.y, p_xmin.z-p_ymax.z);
      Eigen::Vector3f diff3(p_xmax.x-p_ymax.x, p_xmax.y-p_ymax.y, p_xmax.z-p_ymax.z);
      //距離を出す
      d_xmin_xmax = diff1.norm();
      d_xmin_ymax = diff2.norm();
      d_xmax_ymax = diff3.norm();
      //最大値出す
      max_distance = max(d_xmin_xmax, d_xmin_ymax, d_xmax_ymax);
      if (Pattern == 1)
		{
		  r_center.x = (p_xmin.x + p_xmax.x) /2;
		  r_center.y = (p_xmin.y + p_xmax.y) /2;
		  r_center.z = (p_xmin.z + p_xmax.z) /2;
		  grad = atan( (p_xmax.y - p_xmin.y)/(p_xmax.x - p_xmin.x) );
		}
      else if (Pattern == 2)
		{
		  r_center.x = (p_xmin.x + p_ymax.x) /2;
		  r_center.y = (p_xmin.y + p_ymax.y) /2;
		  r_center.z = (p_xmin.z + p_ymax.z) /2;
		  grad = atan( (p_ymax.y - p_xmin.y)/(p_ymax.x - p_xmin.x) );
		}
      else if (Pattern == 3)
		{
		  r_center.x = (p_xmax.x + p_ymax.x) /2;
		  r_center.y = (p_xmax.y + p_ymax.y) /2;
		  r_center.z = (p_xmax.z + p_ymax.z) /2;
		  grad = atan( (p_ymax.y - p_xmax.y)/(p_ymax.x - p_xmax.x) );
		}
      else
		{
		  std::cout << "回転中心求めるの失敗したお" << std::endl;
		  exit(1);
		}

      std::cout << " r_center:" << r_center << std::endl; 
      r_center_x.push_back( r_center.x ); //ベクタに座標を格納
      r_center_y.push_back( r_center.y ); //ベクタに座標を格納
      r_center_z.push_back( r_center.z ); //ベクタに座標を格納
	  grad_box.push_back( grad ); //方向(/2はマスト)
      //直線の中点を回転中心にして180deg回転
      //y軸を180[deg]回転する回転行列
      Eigen::Matrix4f transform;
      transform(0,0) =  -1.0; transform(0,1) =  0.0;  transform(0,2) =  0.0;  transform(0,3) =  r_center.x * 2;
      transform(1,0) =  0.0;  transform(1,1) =  1.0;  transform(1,2) =  0.0;  transform(1,3) =  0.0;
      transform(2,0) = -0.0;  transform(2,1) =  0.0;  transform(2,2) = -1.0;  transform(2,3) =  r_center.y * 2;
      transform(3,0) =  0.0;  transform(3,1) =  0.0;  transform(3,2) =  0.0;  transform(3,3) =  1.0;

      pcl::transformPointCloud(*cloud_cluster, *r_cloud_cluster, transform);

      //元のと回転したのをたすよ
      *cloud_cluster = *cloud_cluster + *r_cloud_cluster;

      //最適矩形の4点を求める
      cv::Point2f center, vtx[4],ofset(1.0f, 1.0f);
      cv::Mat buf;
      buf.create(cloud_cluster->points.size(), 2, CV_32FC1); //CV配列用意
      for (int i = 0; i < cloud_cluster->points.size(); i++)
      	{    	  
		  buf.at<float>(i, 0) = cloud_cluster->points[i].x;
      	  buf.at<float>(i, 1) = cloud_cluster->points[i].y;
      	}

      //外接矩形を計算

      cv::RotatedRect box = cv::minAreaRect(cv::Mat(buf));
      box.points(vtx);
      std::vector<float> nakami_x;
      nice_positionX.push_back(nakami_x);
      std::vector<float> nakami_y;
      nice_positionY.push_back(nakami_y);

      for (int i = 0; i < 4; ++i){ 
		nice_positionX[j].push_back(vtx[i].x);
		nice_positionY[j].push_back(vtx[i].y);
      }

	  j++;
    }

  cluster = j; 
}


void bounding_box::applyBoundingBox_using_inertia()
{
	//PointIndices in vector
	std::vector<pcl::PointIndices> cluster_indices;
	//clustering
	cluster_indices = applyClustering();
	//

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	// viewer->setBackgroundColor (0, 0, 0);
	// viewer->addCoordinateSystem (1.0);
	// viewer->initCameraParameters ();
	// viewer->addPointCloud<PointT> (cloud_, "sample cloud");

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
    {    
		//抽出したポイントクラウドを格納する型
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		//cloud_clusterを回転したものを入れる型
		pcl::PointCloud<PointT>::Ptr r_cloud_cluster (new pcl::PointCloud<PointT>);
	  
		//抽出したポイントクラウドを格納
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud_->points[*pit]); //*
 
		//リサイズとsave
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		//pcl::io::savePCDFileASCII("cluster.pcd" ,*cloud_cluster);
		
        //create object
		pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
		feature_extractor.setInputCloud (cloud_cluster);
		feature_extractor.compute ();

		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		PointT min_point_AABB;
		PointT max_point_AABB;
		PointT min_point_OBB;
		PointT max_point_OBB;
		PointT position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		float major_value, middle_value, minor_value;
		Eigen::Vector3f major_vector, middle_vector, minor_vector;
		Eigen::Vector3f mass_center;

		feature_extractor.getMomentOfInertia (moment_of_inertia);
		feature_extractor.getEccentricity (eccentricity);
		feature_extractor.getAABB (min_point_AABB, max_point_AABB);
		feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
		feature_extractor.getEigenValues (major_value, middle_value, minor_value);
		feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
		feature_extractor.getMassCenter (mass_center);

		stringstream sc;
		sc << "object" << j;
		// viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, sc.str() );

		// Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
		// Eigen::Quaternionf quat (rotational_matrix_OBB);
		// viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

		// PointT center (mass_center (0), mass_center (1), mass_center (2));
		// PointT x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
		// PointT y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
		// PointT z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
		// viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
		// viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
		// viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

		j++;

	}

	cluster = j; 
    // -----Main loop-----
	//--------------------    
	// while (!viewer->wasStopped ())   {
	// 	//viewer->removeCoordinateSystem();
	// 	viewer->spinOnce (100);
	// 	boost::this_thread::sleep (boost::posix_time::microseconds (100000));   	
	// }

	// pcl::visualization::Camera getCamera;

	// viewer->getCameraParameters( getCamera );
	// cout << "camera param *************************" << endl;
	// cout << getCamera.pos[0] << "," << getCamera.pos[1] << "," << getCamera.pos[2] << "," << endl
	// 	 << getCamera.focal[0] << "," << getCamera.focal[1] << "," << getCamera.focal[2] << "," << endl
	// 	 << getCamera.view[0] << "," << getCamera.view[1] << "," << getCamera.view[2]  << endl
	// 	 << "**************************************" << endl;
	
}


int bounding_box::getClusterNumber()
{return cluster;}

//処理した点群とBounding Boxを表示する*******
//引数:ソース点群
//戻り値:なし
//*******************************************
// void bounding_box::Show_result(pcl::PointCloud<PointT>::Ptr in_cloud)
// {

//   int angleSelect = 0;

//   pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (in_cloud, 1.0, 1.0, 1.0);

//   //表示するためのオブジェクトを生成
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
//     (new pcl::visualization::PCLVisualizer ("Nice Viewer"));
//   cout << "オブジェクト作ったで" << endl;
//   stringstream screenshot;
//   screenshot << "screenshot.png";

//   //カメラ設定
//   viewer->initCameraParameters ();

//   //view port 設定(0.0, 0.0, 0.5, 1.0, v1)
//   int v1(0);
//   viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1); //場所
//   viewer->setBackgroundColor (0.0, 1.0, 0.0, v1); //背景色

//   int fontsize = 10;
//   double text_r = 0;
//   double text_g = 0;
//   double text_b = 0;
//   viewer->addText("visualize",10,10,fontsize,text_r,text_g,text_b,"v1 text",v1); //表示文字
//   viewer->addPointCloud<PointT> (in_cloud, single_color, "source cloud", v1); //表示点群
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "source cloud");
//   //箱をかぶせるで
//   for (size_t i = 0; i < min_positionX.size() ;i++){
//     stringstream sc;
//     stringstream st;
//     stringstream sl;
//     sc << "object" << i;
//     st << "text" << i ;
//     sl << "line" << i ;

//     PointT position;
// 	position.x = max_positionX[i] + 0.01;
// 	position.y = min_positionY[i] - 0.01;
// 	position.z = min_positionZ[i] - 0.01;
// 	position.intensity = 0.0;
//     viewer->addText3D(sc.str (), position, 0.02 , 0.0, 0.0, 0.0, st.str (), 0);

//     PointT pt0, pt1, pt2, pt3, pt4, pt5, pt6, pt7;
// 	cout << nice_positionX[0][0] << endl;
// 	cout << nice_positionY[i][0] << endl;
// 	cout << min_positionZ[i] << endl;
	
//     pt0.x = nice_positionX[i][0]; pt0.y = nice_positionY[i][0]; pt0.z =min_positionZ[i];  
//     pt1.x = nice_positionX[i][1]; pt1.y = nice_positionY[i][1]; pt1.z =min_positionZ[i];  
//     pt2.x = nice_positionX[i][2]; pt2.y = nice_positionY[i][2]; pt2.z =min_positionZ[i];  
//     pt3.x = nice_positionX[i][3]; pt3.y = nice_positionY[i][3]; pt3.z =min_positionZ[i];  
//     pt4.x = nice_positionX[i][0]; pt4.y = nice_positionY[i][0]; pt4.z =max_positionZ[i];  
//     pt5.x = nice_positionX[i][1]; pt5.y = nice_positionY[i][1]; pt5.z =max_positionZ[i];  
//     pt6.x = nice_positionX[i][2]; pt6.y = nice_positionY[i][2]; pt6.z =max_positionZ[i];  
//     pt7.x = nice_positionX[i][3]; pt7.y = nice_positionY[i][3]; pt7.z =max_positionZ[i];  

//     //線書きまくるで
//     viewer->addLine(pt0, pt1, 1, 0, 0, sl.str () ,0); sl << 'a';
//     viewer->addLine(pt1, pt2, 1, 0, 0, sl.str () ,0); sl << 'b';
//     viewer->addLine(pt2, pt3, 1, 0, 0, sl.str () ,0); sl << 'c';
//     viewer->addLine(pt3, pt0, 1, 0, 0, sl.str () ,0); sl << 'd';
//     viewer->addLine(pt4, pt5, 1, 0, 0, sl.str () ,0); sl << 'e';
//     viewer->addLine(pt5, pt6, 1, 0, 0, sl.str () ,0); sl << 'f';
//     viewer->addLine(pt6, pt7, 1, 0, 0, sl.str () ,0); sl << 'g';
//     viewer->addLine(pt7, pt4, 1, 0, 0, sl.str () ,0); sl << 'h';
//     viewer->addLine(pt0, pt4, 1, 0, 0, sl.str () ,0); sl << 'i';
//     viewer->addLine(pt1, pt5, 1, 0, 0, sl.str () ,0); sl << 'j';
//     viewer->addLine(pt2, pt6, 1, 0, 0, sl.str () ,0); sl << 'k';
//     viewer->addLine(pt3, pt7, 1, 0, 0, sl.str () ,0); 
//   }

//   viewer->addCoordinateSystem(0.1);

//   // -----Main loop-----
//   //--------------------    
//   while (!viewer->wasStopped ())   {
//     //viewer->removeCoordinateSystem();
//     viewer->spinOnce (100);
//     boost::this_thread::sleep (boost::posix_time::microseconds (100000));   	
//   }

//   pcl::visualization::Camera getCamera;

//   viewer->getCameraParameters( getCamera );
//   cout << "camera param *************************" << endl;
//   cout << getCamera.pos[0] << "," << getCamera.pos[1] << "," << getCamera.pos[2] << "," << endl
//        << getCamera.focal[0] << "," << getCamera.focal[1] << "," << getCamera.focal[2] << "," << endl
//        << getCamera.view[0] << "," << getCamera.view[1] << "," << getCamera.view[2]  << endl
//        << "**************************************" << endl;

//   viewer->saveScreenshot( screenshot.str() );
//   cout << "screenshot saving" << endl; 

// }
