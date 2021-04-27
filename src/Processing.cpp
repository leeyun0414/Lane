#include "Processing.h"
cv::Mat Processing::RotateMatrix(cv::Mat input, int degree) {
		cv::Mat output;
		cv::Point2f a(input.rows/2., input.cols/2.);
		double rotate_limit = 200;
      if (degree > rotate_limit)
		  degree = rotate_limit;
		else if (degree < -rotate_limit)
		  degree = -rotate_limit;
		cv::Mat ro_mat = cv::getRotationMatrix2D(a, -degree/20.0, 1.0);
		cv::warpAffine(input, output, ro_mat, input.size());
    //cv::normalize(imgmask, output, 0, 1, cv::NORM_MINMAX);
		/*imgmask = (output *120) + 120;
		//cout << "roattttt:" << endl << imgmask<<endl;
		outputmask = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgmask).toImageMsg();
		outputmask->header.frame_id = "/map";
		outputmask->header.stamp = ros::Time::now();*/

		//mask_pub.publish(outputmask);
	
		return input;
}

void Processing::onGaussianChange(int _x, void *_ptr) {
	const int sigmaX = 3.1;//3.1
  const int sigmaY = 10;//10
  const int gaussianSizeX = 17;//17 for lucid
  const int gaussianSizeY = 40;//40 for lucid
  float xs[gaussianSizeX], ys[gaussianSizeY];
  float sumx = 0, sumy = 0;
  for (int i = 0; i < gaussianSizeX; i++) {
    float x = 1.0 * i + 0.5 - gaussianSizeX * 0.5;
    xs[i] =(x * x / sigmaX / sigmaX / sigmaX / sigmaX - 1.0 / sigmaX / sigmaX) *exp(-1.0 * x * x / 2 / sigmaX / sigmaX);
    sumx += xs[i];
  }
    for (int i = 0; i < gaussianSizeY; i++) {
    float y = 1.0 * i + 0.5 - gaussianSizeY * 0.5;
    ys[i] = exp(-1.0 * y * y / 2 / sigmaY / sigmaY);
    sumy += ys[i];
  }
  for (int i = 0; i < gaussianSizeX; i++) {
    xs[i] /= sumx;
  }
  for (int i = 0; i < gaussianSizeY; i++) {
    ys[i] /= sumy;
  }
  gaussianKernelX = Mat(1, gaussianSizeX, CV_32F, xs).clone();
  gaussianKernelY = Mat(gaussianSizeY, 1, CV_32F, ys).clone();
  gaussianKernel =  gaussianKernelY * gaussianKernelX ;
  //gaussianKernel = Mat::ones(gaussianSizeX, gaussianSizeY, CV_32F) / gaussianSizeX /gaussianSizeY;
}

void Processing::onROIChange(int _x, void *ptr) {
  tsfIPM =(Mat_<double>(3,3) <<-3.1244505333343681e-01, -1.2511122410509723e+00,
      1.2306094695224358e+03, -1.3080549239745093e-01,
      -2.2605287144327066e+00, 2.0908415399528508e+03,
      -9.1797862905148325e-05, -1.2332182509434129e-03, 1.);//2048 lucid
  /*tsfIPM = (Mat_<double>(3, 3) <<-6.2087560281703880e-01, -1.6253555914476892e+00,
    1.1420541386342284e+03, -1.3883372635599586e-01,
    -2.4513515447365908e+00, 1.4255304261124224e+03,
    -1.6822209528863456e-04, -2.0394790779835197e-03, 1.);*////hdr
  /*tsfIPM = (Mat_<double>(3,3)<<-3.1329326340266522e-01, -1.4768639051388419e+00,
    3.5080142509585801e+02, -3.6482507243650927e-03,
    -2.4146137241911978e+00, 5.0496233428483197e+02,
    -5.4386216110840380e-06, -5.5027678457702598e-03,
    1.);*///512 lucid
    /*(Mat_<double>(3,3) <<-4.5571355360643490e-01, -1.2664342958154713e+00,
    5.3742940324105768e+02, -1.9691208835863266e-02,
    -2.2538761777495382e+00, 7.2989152350589757e+02,
    -3.1533269191852278e-05, -3.4487132345171873e-03, 1.);*///768 lucid
  tsfIPMInv = tsfIPM.inv();	
}

cv::Mat Processing::detectLanePixels(cv::Mat imgInput, double rotate) {
    
  cv::Mat imgROI = imgInput.clone();// = Mat(imgInput, roiLane);
  warpPerspective(imgROI, imgIPM32, tsfIPM, imgROI.size());
  warpPerspective(imgIPM32, imgIPMInv32, tsfIPMInv, imgROI.size());
  //cv::Mat imgtest = imgClone;
  //warpPerspective(imgtest, imgIPM35, tsfIPM, imgtest.size());
  if (show_img) {
    outputIPM = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgIPM32).toImageMsg();
    outputIPM->header.frame_id = "/map";
    outputIPM->header.stamp = ros::Time::now();
    IPM_pub.publish(outputIPM);
  }
  //cout<<imgIPM32.size()<<endl;

  cvtColor(imgIPM32, imgROIGray, CV_RGB2GRAY);
  /* const char *winipm = "ipm";
  cv::namedWindow(winipm, CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
  cv::imshow(winipm, imgROI);*/
  //imgGaussian = imgROIGray;

  //cv::Mat Rotate_matrix = RotateMatrix(gaussianKernel, rotate);
  filter2D(imgROIGray, imgGaussian, imgIPM32.depth(), gaussianKernel);
  //imgGaussian = imgROIGray.clone();
  
  //sepFilter2D(imgROIGray, imgGaussian, imgIPM32.depth(), gaussianKernelX, gaussianKernelY);
  if (show_img) {
    outputgaussian = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgGaussian).toImageMsg();
    outputgaussian->header.frame_id = "/map";
    outputgaussian->header.stamp = ros::Time::now();
    gaussian_pub.publish(outputgaussian);
  }
  //imgGaussian = imgROIGray.clone();
  cv::Mat imgHist;
  cv::equalizeHist(imgGaussian, imgHist);
  //imgHist = imgGaussian;
  cv::threshold(imgHist, imgThreshold, 255 * (thresholdingQ) / 1000, 255, THRESH_TOZERO);

  cvtColor(imgThreshold, imgBackIPM32, COLOR_GRAY2BGR);
  std::vector<cv::Mat> channels(3);
  split(imgBackIPM32, channels);
  cv::Mat ch1, ch2, ch3;
  ch2 = channels[1];
  ch1 = cv::Scalar::all(0);
  ch3 = cv::Scalar::all(0);
  cv::merge(channels, imgBackIPM32);
  cv::warpPerspective(imgBackIPM32, imgBackIPMInv32, tsfIPMInv, imgROI.size());

  cv::Mat imgModel = imgOrigin(Rect(0, 0, imgBackIPMInv32.cols, imgBackIPMInv32.rows));
  cv::Size img_size = imgInput.size();
  cv::Mat wantedMask = Mat(img_size, CV_8UC1, cv::Scalar(0));
  cv::Mat imgBackIPMInvGray;
  cv::cvtColor(imgBackIPMInv32, imgBackIPMInvGray, cv::COLOR_RGB2GRAY);
  cv::threshold(imgBackIPMInvGray, imgBackIPMInvGray, 1, 255, THRESH_BINARY);
  cv::Mat wantedMaskRoi =
      wantedMask(Rect(0, 0, imgBackIPMInvGray.cols, imgBackIPMInvGray.rows));
  cv::addWeighted(imgBackIPMInvGray, 1, wantedMaskRoi, 0, 0, wantedMaskRoi);
  cv::addWeighted(imgModel, 0, imgBackIPMInv32, 1, 0, imgModel);
  if (show_img){
    outputhistogram = cv_bridge::CvImage(std_msgs::Header(), "mono8", wantedMask).toImageMsg();
    outputhistogram->header.frame_id = "/map";
    outputhistogram->header.stamp = ros::Time::now();
    histogram_pub.publish(outputhistogram);
  }
  return wantedMask;          
}
void Processing::callback1(const sensor_msgs::CompressedImageConstPtr& msg){
  //std::cerr << "CV_mode: " << mode_flag << endl;
  if (!mode_flag)
    return;
  clock_t time_test, tmm;
  time_test = clock();
  LaneDetection ld = LaneDetection();
  msgtime = msg->header.stamp;
  //INPUT

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat cv_image = cv_ptr->image;
  if (show_img)
    std::cout << "transfer ros to cv: " << (float)(clock() - time_test)/CLOCKS_PER_SEC<< std::endl;
  time_test = clock();
  cv::Mat input=cv_image.clone();
  //cv::resize(cv_image, input, Size(roiWidth, 720), 0, 0, cv::INTER_LINEAR);
  //cv::imwrite("fdf.jpg", input);
  //cv::resize(input, input, Size(roiWidth, 720), 0, 0, cv::INTER_LINEAR);
  //std::cout << "aa" << std::endl;
  //cv::Mat imgtt = input.clone();
  //imgtt(cv::Rect(0, 0, imgWidth,imgHeight)) = 0;
  /*for (int i = 0; i < input.rows; i++) 
    for (int j = 0; j < input.cols; j++) {

      cv::Point2d temp_1;
      temp_1.x = j;
      temp_1.y = i;
      cv::Point2d temp = local.IPM(temp_1);
      if (temp.x >= imgWidth || temp.x < 0 || temp.y >= imgHeight || temp.y < 0)
        continue;
        //std::cout << "bb" << std::endl;
  
      imgtt.at<uchar>(temp.y, temp.x) = input.at<uchar>(i, j);
      //std::cout << "cc" << std::endl;
  
    }
  }
  outputmarking = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ).toImageMsg();
  outputmarking->header.frame_id = "/scan";
  outputmarking->header.stamp = ros::Time::now();
  marking_pub.publish(outputmarking);*/
  /*if (camera_name == "/lucid_cameras_x03/gige_90_fl") {
    cv::Point2f src_center(input.cols/2.0F, input.rows/2.0F);
    cv::Mat rot_mat = getRotationMatrix2D(src_center, 180, 1.0);
    cv::Mat dst;
    cv::warpAffine(input, dst, rot_mat, input.size());
    input = dst.clone();
  }*/
  ld.initialize_variable(input, roiRatio);
    //PREPROCESS
  imgOrigin = input.clone();
  input.copyTo(outputmap);

  //std::cout << outputmap << std::endl;
  cv::Mat bw1;
  cvtColor(input, bw1, CV_RGB2GRAY);
  //cv::adaptiveThreshold(bw1, bw1, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 201, -0);
  //imgOrigin.convertTo(imgOrigin, -1,  1.7, -5);
  Mat imgMask = imgOrigin.clone();
    // LANE SEGMENTATION
  Mat lanemask = detectLanePixels(imgOrigin, rotate);
  if (show_img)
    std::cout << "detectLanePixel: " << (float)(clock() - time_test)/CLOCKS_PER_SEC << std::endl;
  time_test = clock();
  Mat imgLaneSeg = imgOrigin.clone();
  
  Mat toc2[] = {lanemask, lanemask, lanemask};
  Mat img_buf2 = Mat(imgOrigin.size(), imgOrigin.depth());
  cv::merge(toc2, 3, img_buf2);
  imgLaneSeg = imgLaneSeg.mul(img_buf2 / 255);
    // LANE DETECTION
  Mat imgProcess = imgLaneSeg.clone();
  //imgProcess.convertTo(imgProcess, -1, contrast / 10, -brightness);
  if (show_img)
    std::cout << "carmask: " << (float)(clock() - time_test)/CLOCKS_PER_SEC<< std::endl;
  time_test = clock();
  ld.initialize_Img(imgProcess);
  Mat imgShow = input.clone();
      //imgProcess = input.clone();
  ld.lane_marking_detection(imgProcess, imgShow, 1);
  if (show_img)
    std::cout << "lane_mark_detection: " << (float)(clock() - time_test)/CLOCKS_PER_SEC<< std::endl;
  time_test = clock();
  outputmap = ld.CannyDetector(outputmap);
  if (show_img) {
    outputmarking = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputmap).toImageMsg();
    outputmarking->header.frame_id = "/scan";
    outputmarking->header.stamp = ros::Time::now();
    marking_pub.publish(outputmarking);
  }
  imgShow = input.clone();
  ld.seed_generation(imgProcess, imgShow, 1);
  if (show_img)
    std::cout << "seed_generation: " << (float)(clock() - time_test)/CLOCKS_PER_SEC<< std::endl;
  time_test = clock();
  if (show_img) {
    outputassociation = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgShow).toImageMsg();
    outputassociation->header.frame_id = "/map";
    outputassociation->header.stamp = ros::Time::now();
    association_pub.publish(outputassociation);
  }
  imgShow = input.clone();
  //std::vector<cv::Point2d> lanemark;
  lanemark = ld.graph_generation(imgProcess, imgShow, 50);
  if (show_img)
    std::cout << "graph: " << (float)(clock() - time_test)/CLOCKS_PER_SEC<< std::endl;
  time_test = clock();
  outputgraph = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgShow).toImageMsg();
  outputgraph->header.frame_id = "/map";
  outputgraph->header.stamp = msgtime;
  graph_pub.publish(outputgraph);
  clear_features();
  match(lanemark, msgtime, sur_cloud);
  if (show_img)
    std::cout << "2d to 3d: " << (float)(clock() - time_test)/CLOCKS_PER_SEC<< std::endl;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::get_features() {
  return sur_cloud;
}
void Processing::clear_features() {
  sur_cloud->clear();
}
ros::Time Processing::get_msg_time(){
  return msgtime;
}
void Processing::debug() {
  cerr << "----------------------------------" << endl;
  cerr << "Camera_Name:" << std::endl << camera_info->mCameraName << endl;
  cerr << "ImageWidth:" << std::endl << camera_info->mImageWidth << endl;
  cerr << "ImageHeight:" << std::endl << camera_info->mImageHeight << endl;
  cerr << "Kalib" << std::endl << camera_info->mKalib << endl;
  cerr << "InverseKalib" << std::endl << camera_info->mInverseKalib << endl;
  //cerr << camera_info->mCameraInfo << endl;
  cerr << "BaselinkToCameraMatrix" << std::endl << camera_info->mBaselinkToCameraMatrix << endl;
  cerr << "CameraToBaseLinkMatrix" << std::endl << camera_info->mCameraToBaseLinkMatrix << endl;
  cerr << "----------------------------------" << endl;
}
void Processing::match(std::vector<cv::Point2d> input,
                       ros::Time msg_time,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr lane_cloud){
                        
  if (!input.size())
    return;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointXYZ>);

  for (auto i = input.begin(); i != input.end() - 1; i++) {
    pcl::PointXYZ temp;
    if (i->x == -1 && i->y == -1){
      continue;
    }
    else {
      Eigen::Vector3d point = IPM(*i);//x forward in camera;
      Eigen::Vector3d point_camera;
      point_camera[0] = -point[1];//z forward in camera;
      point_camera[1] = point[2];
      point_camera[2] = point[0];
      temp.x = -point[1];
      temp.y = point[2];
      temp.z = point[0];
      if (temp.z > 30)
        continue;
      cloud_in_->points.push_back(temp);	  
    }
  }
	cloud_in_->width = (int)cloud_in_->points.size();
  cloud_in_->height = 1; 
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_in_);
  sor.setLeafSize (0.8f, 0.8f, 0.8f);//2.
  sor.filter (*cloud_in);
        
	if (cloud_in->size() == 0)
	  return; 
  for(size_t i=0; i<cloud_in->points.size(); i++) {//camera to base link
    Eigen::Vector4d lidarPoint(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, 1);
    lidarPoint = camera_info->mCameraToBaseLinkMatrix  * lidarPoint;
    pcl::PointXYZ temp;
    temp.x = lidarPoint[0];
    temp.y = lidarPoint[1];
    temp.z = lidarPoint[2];
    cloud_in_base->points.push_back(temp);
    cloud_in_base->height = 1;
    cloud_in_base->width = cloud_in->points.size();
  }
  lane_cloud->width += (int)cloud_in_base->points.size();//all lanes cloud
  lane_cloud->height = 1;
  pcl_conversions::toPCL(msg_time, lane_cloud->header.stamp);
  for (int i = 0; i < cloud_in_base->points.size(); i++)
    lane_cloud->points.push_back(cloud_in_base->points[i]);
        
}
Eigen::Vector3d Processing::IPM(cv::Point2d pixel) {
  Eigen::Affine3d b;
  b.matrix() = camera_info->mBaselinkToCameraMatrix;
  Eigen::Matrix3d rotation_part = b.rotation().cast<double>();
  tf::Matrix3x3 mat_local;
  tf::matrixEigenToTF(rotation_part, mat_local);
  double roll, pitch, yaw;
  mat_local.getRPY(roll, pitch, yaw, 1);
	//std::cout << pitch << std::endl;
  if (camera_pitch > 1)
    camera_pitch = pitch + 1.5708;/////////To be modified, considering pitch change
	double final_X, final_Y;
  double imgheight = camera_info->mImageHeight;
  double imgwidth = camera_info->mImageWidth;
  Eigen::Matrix3d camera_intrin = camera_info->mKalib;
	final_X = tan(camera_pitch) * (1 - 2 * (pixel.y - 1) / (imgheight - 1)) * (imgheight - 1) / 2 / camera_intrin(1, 1) - 1;
	final_X /= (tan(camera_pitch) + (1 - 2 * (pixel.y - 1) / (imgheight - 1)) * ((imgheight - 1) / 2 / (camera_intrin(1, 1))));
	final_X *= camera_height;
  final_Y = (1 - 2 * ((pixel.x - 1) / (imgwidth - 1))) * final_X * (imgwidth - 1) / 2 / camera_intrin(0, 0);
	final_Y *= cos(atan(((camera_intrin(1, 2)) - pixel.y) / camera_intrin(1,1)));
	Eigen::Vector3d output;
	output[0] = final_X;
	output[1] = final_Y;
	output[2] = 0;//camera_height;
	/*Eigen::Vector3d temp;
	temp[0] = pixel.y;
	temp[1] = pixel.x;
	temp[2] = 1;//camera_height * tan(camera_pitch + 1.5708);
	temp = camera_intrin.inverse() * temp;
	Eigen::Vector4d plane;
	plane << 0, 0, 1, 0;
	tf::Matrix3x3 rotation_tf_matrix;
	tf::Quaternion Quaternion_temp, lidar_camera_temp;
	geometry_msgs::Quaternion c2l;
	c2l.x = base2came.getRotation()[0];
	c2l.y = base2came.getRotation()[1];
	c2l.z = base2came.getRotation()[2];
	c2l.w = base2came.getRotation()[3];
	//cu_pose.pose.pose.orientation *= lidar_to_camera;
	
	quaternionMsgToTF(c2l, lidar_camera_temp);
	rotation_tf_matrix.setRotation(lidar_camera_temp); //myQuaternion
  Eigen::Matrix4d basecam = Eigen::Matrix4d::Identity();
	tf::matrixTFToEigen(rotation_tf_matrix, rotation_matrix);
	basecam(0,0) = rotation_matrix(0,0);
	basecam(0,1) = rotation_matrix(0,1);
	basecam(0,2) = rotation_matrix(0,2);
	basecam(1,0) = rotation_matrix(1,0);
	basecam(1,1) = rotation_matrix(1,1);
	basecam(1,2) = rotation_matrix(1,2);
	basecam(2,0) = rotation_matrix(2,0);
	basecam(2,1) = rotation_matrix(2,1);
	basecam(2,2) = rotation_matrix(2,2);
	basecam(0,3) = base2came.getOrigin()[0];
	basecam(1,3) = base2came.getOrigin()[1];
	basecam(2,3) = base2came.getOrigin()[2];	
	plane = basecam.transpose() * plane;
	std::cout << plane << ", " << std::endl;
	Eigen::VectorXd frac = -plane[3] / (plane.head(3).transpose() * temp).array();
	Eigen::MatrixXd out = frac.transpose().replicate<3, 1>().array() * temp.array();
	//std::cout << out <<std::endl << ", ";                                                                                                                       
	//final_X = (pixel.y - camera_intrin(0, 2)) / camera_intrin(0, 1);
	//final_Y = (pixel.x - camera_intrin(1, 2)) / camera_intrin(1, 1);
	output[0] = out(0, 0);
	output[1] = out(1, 0);
	output[2] = out(2, 0);*/

	return output;                 
}
void Processing::draw_points_on_image(pcl::PointCloud<pcl::PointXYZ>::Ptr lane_cloud,
                                      Eigen::Matrix4d vehicle_pose) {
  tf::Point imagePoint3d;
  Eigen::Affine3d temp_tr;
  pcl::PointXYZ p;
  cv::Point2d imagePoints2d(0,0);
  bool isInsideImage;
  
  cv::Mat visImg(1536, 2048, CV_8UC3, Scalar(0, 0, 0));
  cv::Mat input = visImg.clone();
  for(size_t i=0; i<lane_cloud->points.size(); i++) {
    Eigen::Vector4d lidarPoint(lane_cloud->points[i].x, lane_cloud->points[i].y,
                               lane_cloud->points[i].z, 1);                             
    //tf::transformTFToEigen(camera_info->mBaselinkToCameraMatrix , temp_tr);
    Eigen::Matrix4d camera_set_height = camera_info->mBaselinkToCameraMatrix;
    lidarPoint = camera_set_height * vehicle_pose.inverse() * lidarPoint;
    tf::Point imagePoint3d(lidarPoint[0], lidarPoint[1], lidarPoint[2]);
    if (imagePoint3d.z() >0 && imagePoint3d.z() < 30) {
      Eigen::Vector3d Points3d(imagePoint3d.x(),
                            imagePoint3d.y(),
                            imagePoint3d.z());
      
      Points3d = camera_info->mKalib * Points3d;
      cv::Point2d imagePoints2d;
      imagePoints2d.x = Points3d[0]/Points3d[2];imagePoints2d.y = Points3d[1]/Points3d[2];
      bool isInsideImage = (imagePoints2d.x > 0) && (imagePoints2d.y > 0)
          && (imagePoints2d.x < input.cols)
          && (imagePoints2d.y < input.rows);
      if (isInsideImage) {
        cv::line(visImg, imagePoints2d, imagePoints2d, cv::Scalar(0, 255, 0), 10);
      }
    }
  }
  cv::addWeighted(input, 0.6, visImg, 0.4, 0, visImg);
  
  sensor_msgs::ImagePtr test;
  test = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImg).toImageMsg();
  test->header.stamp = msgtime;
  draw_pub.publish(test); 
}
void Processing::callback2(const std_msgs::Int16MultiArrayConstPtr& msg){
  //std::cerr << msg->data.size() << " ";
  //std::cerr << "DL_mode: " << mode_flag << endl;
  clear_features();
  if (!msg->data.size()) {
    mode_flag = 1;
    return;
  }
  mode_flag = 0;
  for (int i = 0; i < msg->data.size()-1; i=i+2) {
    cv::Point2d temp(0, 0);
    temp.x = msg->data[i] / 640.0 * camera_info->mImageWidth;
    temp.y = msg->data[i+1] / 404.0 * camera_info->mImageHeight;
    //std::cerr << "x:" << temp.x << " " << "y:" << temp.y << std::endl;

    dlmark.push_back(temp);
  }
  
  match(dlmark, msgtime, sur_cloud);
  
  dlmark.clear();
}