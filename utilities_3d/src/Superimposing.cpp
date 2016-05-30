# include <Superimposing.h>

using namespace std;
using namespace boost;

#define TORINOSU 2.0

clock_t startTime , endTime; double margin; //時間計測用変数

superImposing::superImposing(pcl::PointCloud<PointT>::Ptr cloud1, pcl::PointCloud<PointT>::Ptr cloud2){
  superImposing::setFrame1(cloud1);
  superImposing::setFrame2(cloud2);
  superImposing::setParam_superimposing();
  superImposing::setParam_odom();
  position_ = 0.0;
}

superImposing::superImposing(superImposing &obj){
  pre_cloud_ = obj.pre_cloud_;
  source_cloud1_ = obj.source_cloud1_;
  source_cloud2_ = obj.source_cloud2_;
  result_cloud_ = obj.result_cloud_;
}

//setter
void superImposing::setFrame1(pcl::PointCloud<PointT>::Ptr source_cloud1){ source_cloud1_ = source_cloud1;}
void superImposing::setFrame2(pcl::PointCloud<PointT>::Ptr source_cloud2){ source_cloud2_ = source_cloud2;}


//ファイルからパラメータを読みこむ①
void superImposing::setParam_superimposing()
{
  fstream file;
  //ファイルの読み込み
  file.open("./params/superimposing_param.txt", ios::in);
  if(! file.is_open()) {
	cout << "param.txt is not exists!" << endl;
    exit(1);
  }
  cout << "--->param.txt read OK!\n";
  string t_str;
  getline(file, t_str);
  getline(file, t_str);
  file.close();
  vector<string> ori_data;
  vector<double> data;
  //カンマ区切りするよ
  char_separator<char> seq(",");
  tokenizer< char_separator<char> > tokens( t_str, seq );
  typedef tokenizer< char_separator<char> >::iterator Iter;
  Iter init = tokens.begin();
  //カンマ区切りにした文字列をvectorに格納
  for( Iter it=tokens.begin(); it != tokens.end(); ++it){
    ori_data.push_back(*it);
  }
  if (ori_data.size() != 5){
    cout << "superimposing_param.txtのパラメータの数が違いますよ" << endl;
    exit(1);
  }
  for (int i = 0; i < ori_data.size() ; i++){
    if (i < ori_data.size()-1)
      data.push_back(lexical_cast<double>(ori_data[i]));
  }
  attention_area_ = data[0]; cout << " attention_area_:" << data[0] << endl;
  superimposing_area_ = data[1]; cout << " superimposing_area_:" << data[1] << endl;
  attention_width_ = data[2]; cout << " attention_width_:" << data[2] << endl;
  freq_ = data[3]; cout << " freq_:" << data[3] << endl;
  save_folder_pass_ = ori_data[4]; cout << " save_folder_pass_:" << ori_data[4] << endl;
}

//ファイルからパラメータを読み込む②
void superImposing::setParam_odom()
{
  fstream file;
  //ファイルの読み込み
  file.open("./params/odom.txt", ios::in);
  if(! file.is_open()) {
	cout << "odom.txt is not exists!" << endl;
    exit(1);
  }
  cout << "--->odom.txt read OK!\n";
  string t_str;
  getline(file, t_str);
  getline(file, t_str);
  file.close();
  vector<string> ori_data;
  vector<double> data;
  //カンマ区切りするよ
  char_separator<char> seq(",");
  tokenizer< char_separator<char> > tokens( t_str, seq );
  typedef tokenizer< char_separator<char> >::iterator Iter;
  Iter init = tokens.begin();
  //カンマ区切りにした文字列をvectorに格納
  for( Iter it=tokens.begin(); it != tokens.end(); ++it){
    ori_data.push_back(*it);
  }
  if (ori_data.size() != 2){
    cout << "odom.txtのパラメータの数が違いますよ" << endl;
    exit(1);
  }
  for (int i = 0; i < ori_data.size() ; i++){
	data.push_back(lexical_cast<double>(ori_data[i]));
  }
  speed_ = data[0]; cout << " speed_:" << data[0] << endl;
  angle_ = data[1]; cout << " angle_:" << data[1] << endl;
}

//ファイルからパラメータを読み込む②
void superImposing::setParam_NDT()
{
  fstream file;
  //ファイルの読み込み
  file.open("./params/ndt.txt", ios::in);
  if(! file.is_open()) {
	cout << "ndt.txt is not exists!" << endl;
    exit(1);
  }
  cout << "--->ndt.txt read OK!\n";
  string t_str;
  getline(file, t_str);
  getline(file, t_str);
  file.close();
  vector<string> ori_data;
  vector<double> data;
  //カンマ区切りするよ
  char_separator<char> seq(",");
  tokenizer< char_separator<char> > tokens( t_str, seq );
  typedef tokenizer< char_separator<char> >::iterator Iter;
  Iter init = tokens.begin();
  //カンマ区切りにした文字列をvectorに格納
  for( Iter it=tokens.begin(); it != tokens.end(); ++it){
    ori_data.push_back(*it);
  }
  if (ori_data.size() != 5){
    cout << "ndt.txtのパラメータの数が違いますよ" << endl;
    exit(1);
  }
  for (int i = 0; i < ori_data.size() ; i++){
	data.push_back(lexical_cast<double>(ori_data[i]));
  }
  ndt_epsilon_ = data[0]; cout << " ndt_epsilon_:" << data[0] << endl;
  ndt_step_size_ = data[1]; cout << " ndt_step_size_:" << data[1] << endl;
  ndt_resolution_ = data[2]; cout << " ndt_resolution_:" << data[2] << endl;
  ndt_max_iterations_ = (int)data[3]; cout << " ndt_max_iterations_:" << (int)data[3] << endl;
  ndt_leafsize_ = data[4]; cout << " ndt_leafsize_:" << data[4] << endl;
}

//getter
pcl::PointCloud<PointT>::Ptr superImposing::getFrame1(){ return source_cloud1_;}
pcl::PointCloud<PointT>::Ptr superImposing::getFrame2(){ return source_cloud2_;}
pcl::PointCloud<PointT>::Ptr superImposing::getResult(){ return result_cloud_;}


//ドーナツ点群生成関数
pcl::PointCloud<PointT>::Ptr superImposing::nearRemoval(pcl::PointCloud<PointT>::Ptr input_cloud, double radius_f, double radius_n)
{
  pcl::PointCloud<PointT>::Ptr after_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr after_cloud_neo(new pcl::PointCloud<PointT>);
  //kdtree
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(input_cloud);

  //search point setting
  PointT searchPoint;
  searchPoint.x = 0.0f;
  searchPoint.y = 0.0f;
  searchPoint.z = 0.0f;
  searchPoint.intensity = 0.0f;

  //radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  //----------------------------------------------
  //①radius_n == 0.0
  //->radius_rの範囲にある点群を生かす。
  //---------------------------------------------

  //---------------------------------------------
  //②radius_n != 0.0 && radius_n < radius_f
  //->radisu_nからradius_rの範囲にある点群を生かす。
  //---------------------------------------------

  //show parameter
  cout << "--->near Removal:\n"
	   << "(radius_f):" << radius_f << endl
	   << "(radius_n):" << radius_n << endl; 

  cout << "cloud size:" << input_cloud->points.size() << "(before)" << endl;
  startTime = clock();
  if (input_cloud->points.size() == 0){
	cout << "点群データないし終わるね、バイバーイ" << endl;
	exit(0);
  }

  //最近傍探索
  kdtree.radiusSearch (searchPoint, radius_f, pointIdxRadiusSearch, pointRadiusSquaredDistance);

  //near cut
  //radius_f以内の点群をとりあえずピックアップ
  after_cloud->points.clear();
  for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
	after_cloud->points.push_back( input_cloud->points[ pointIdxRadiusSearch[i] ]);
  
  if (radius_n == 0.0){
	//半径radius_fの範囲の点群を返す
	cout << "cloud size:" << after_cloud->points.size() << "(after)" << endl;
	endTime = clock();
	margin = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	cout << "processing time:" << margin << "sec" << endl;
	return after_cloud;
  }

  else if(radius_f > radius_n){
	kdtree.setInputCloud(after_cloud);
	after_cloud_neo->points.clear();
	pointIdxRadiusSearch.clear();
	//最近傍探索その2
	kdtree.radiusSearch (searchPoint, radius_n, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	//sort
	sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());

	//値を入れていくお
	int pt_counter;
	pt_counter = 0;
	for(int i = 0; i < after_cloud->points.size(); i++){ 
	  if (i != pointIdxRadiusSearch[pt_counter]){
		after_cloud_neo->points.push_back( after_cloud->points[i] );
	  }
	  else{
	  pt_counter++;
	  }
	}

	//半径radius_fの範囲の点群から半径radius_n分えぐった点群を返す
	cout << "cloud size:" << after_cloud_neo->points.size() << "(after)" << endl;
	endTime = clock();
	margin = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	cout << "processing time:" << margin << "sec" << endl;
	return after_cloud_neo;
  }
  else{
	cout << "設定を見なおしてみてね♡" << endl;
  }
}


//widthで指定した幅でサイドカット
pcl::PointCloud<PointT>::Ptr superImposing::sideCut(pcl::PointCloud<PointT>::Ptr input_cloud, double width)
{
  pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
  // Create the filtering object
  cout << "--->side Cut:\n";

  pcl::PassThrough<PointT> pass;
  startTime = clock();
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-width, width);
  pass.filter (*output_cloud);
  endTime = clock();
  cout << "cloud size:" << input_cloud->points.size() << "(before)" << endl;
  cout << "cloud size:" << output_cloud->points.size() << "(after)" << endl;
  margin = (double)(endTime - startTime) / CLOCKS_PER_SEC;
  cout << "processing time:" << margin << "sec" << endl;
  return output_cloud;
}


//cropBoxFilter
pcl::PointCloud<PointT>::Ptr
superImposing::crop_box_filter(pcl::PointCloud<PointT>::Ptr in_cloud, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
  pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>);
  //最小値定義
  Eigen::Vector4f minPoint; 
  minPoint[0] = xmin;  // define minimum point x 
  minPoint[1] = ymin;  // define minimum point y 
  minPoint[2] = zmin;  // define minimum point z 
  
  //最大値定義
  Eigen::Vector4f maxPoint; 
  maxPoint[0]= xmax;  // define minimum point x 
  maxPoint[1]= ymax;  // define max point y 
  maxPoint[2]= zmax;  // define max point z

  //回転角度[rad]
  Eigen::Vector3f boxRotation; 
  boxRotation[0]=0;  // rotation around x-axis 
  boxRotation[1]=0;  // rotation around y-axis 
  boxRotation[2]=0;  //in radians rotation around z-axis 

  //フィルタリング開始
  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud (in_cloud); //元データセット 
  cropFilter.setMin(minPoint); //最小値セット
  cropFilter.setMax(maxPoint); //最大値セット 
  cropFilter.setRotation(boxRotation);
  cropFilter.filter (*result); //フィルタにかけまっせ

  cout << "crop box filter 完了!(size: " << result->points.size() << ")" << endl;
  return result; 
}


//周波数と大まかな速度から進んだ距離を算出
 double superImposing::estimatePosition(double speed, double freq)
{
  double pos;
  pos = (speed/3.6)/freq;
  return pos;
}


//箱かぶせる前にfever
pcl::PointCloud<PointT>::Ptr superImposing::applyPreTreatment()
{  
  pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>);//最終結果格納用
  pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);//voxel grid filter用
  pcl::PointCloud<PointT>::Ptr target_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);

  //ほしいエリアの点群をゲットするお
  target_cloud = superImposing::nearRemoval(source_cloud1_, attention_area_, TORINOSU);
  target_cloud = superImposing::sideCut(target_cloud, attention_width_);
  //target_cloud = source_cloud1_;
  cout << "--->target cloud stand by OK!!" << endl;

  input_cloud = superImposing::nearRemoval(source_cloud2_, attention_area_, TORINOSU);
  input_cloud = superImposing::sideCut(input_cloud, attention_width_);
  //input_cloud = source_cloud2_;
  cout << "--->input cloud stand by OK!!" << endl;
  
  //nomal distributions transform
  //移動量を考慮しNDTにより変換行列を求める
  cout << "--->Normal Distibutions Transform" << endl;
  setParam_NDT();
  pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (ndt_leafsize_, ndt_leafsize_, ndt_leafsize_); //
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  cout << " Input cloud contains " << filtered_cloud->size ()
	   << " data points [filtered]" << endl;
  cout << " Target cloud contains" << target_cloud->size ()
	   << " data points [raw]" << endl;

  cout << "NDTやってるよ..." << endl;
  startTime = clock();
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<PointT, PointT> ndt;
  // Setting scale dependent NDT parameters---------------------------------
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (ndt_epsilon_);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (ndt_step_size_);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (ndt_resolution_);  
  // Setting max number of registration iterations.
  ndt.setMaximumIterations (ndt_max_iterations_);
  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Extra param
  ndt.EuclideanFitnessEpsilon(); // fitness
  ndt.setOutlierRatio(); // outlier / total
  ndt.setMaxCorrespondenceDistance(); // max correspond distance


  
  //------------------------------------------------------------------------
  
  //Set initial alignment estimate found using robot odometry.--------------
  Eigen::AngleAxisf init_rotation (angle_, Eigen::Vector3f::UnitZ ());
  //simplly position estimate ([Hz],[km/h])
  double pos = superImposing::estimatePosition(speed_, freq_); //
  //position_ += pos;
  position_ = pos;
  cout << " position_:" << position_ << endl;
  //translation
  Eigen::Translation3f init_translation (position_, 0.0, 0.0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
  //------------------------------------------------------------------------
  
  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<PointT>::Ptr output_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr sum_cloud (new pcl::PointCloud<PointT>);
  
  ndt.align (*output_cloud, init_guess);
  endTime = clock();

  //show result
  std::cout << " Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;
  margin = (double)(endTime - startTime) / CLOCKS_PER_SEC;
  cout << "processing time:" << margin << "sec" << endl;
  
  //Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
 
  //たす 
  *sum_cloud = *output_cloud + *target_cloud;
  sum_cloud->width = sum_cloud->points.size();
  sum_cloud->height = 1;
  sum_cloud->points.resize(sum_cloud->width * sum_cloud->height);

  //Saving transformed input cloud.
  stringstream ss1;
  ss1 << save_folder_pass_ << "NDT_result.pcd";
  pcl::io::savePCDFileASCII (ss1.str(), *sum_cloud);
  cout << "--->NDT result save OK!! (file:NDT_result.pcd)" << endl;

  //要る部分切り出し
  //result = superImposing::nearRemoval(sum_cloud, attention_area_, superimposing_area_);
  //result = superImposing::crop_box_filter(sum_cloud, 0.0, attention_area_, -attention_width_, attention_width_, -2.0, 1.0);
  result = sum_cloud;
  result->width = result->points.size();
  result->height = 1;
  result->points.resize(result->width * result->height);

  //Saving transformed input cloud.
  stringstream ss2;
  ss2 << save_folder_pass_ << "SI_result.pcd";
  pcl::io::savePCDFileASCII (ss2.str(), *result);
  cout << "--->Superimposing result save OK!! (file:SI_result.pcd)" << endl;

  //結果を保存
  result_cloud_ = result;
  return output_cloud;

}

void superImposing::applyAfterTreatment()
{
  //任せる
  //フレーム数
  //箱の処理
  //もっともらしい箱同士を足しあわせ
  //結果を保存

}
