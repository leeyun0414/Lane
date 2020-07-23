#include <stdint.h>
#include "Hungarian.h"
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <vector>
#include <LaneDetection.h>

#include <parabolicMask.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
#include <cmath>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;
#define PI 3.1415926
//using namespace boost::filesystem;

const double contrast = 29;//17.5
const int brightness = 15;//37
const int roiX = 0;//modify:y
const int roiY = 380;
const int roiWidth = 1280;
const int roiHeight = 720 - roiY;
/*const int roiX = 250;//modify:y
const int roiY = 380;
const int roiWidth = 1280 - roiX;
const int roiHeight = 720 - roiY;*/
const int srcX1 = 525;//610;//525;//modify:x
const int srcX2 = 770;//750;//770;

const int sigmaX = 3.1;//3
const int sigmaY = 10;//10
const int gaussianSizeX = 15;
const int gaussianSizeY = 15;
const int thresholdingQ = 990;//990   750

cv::Mat imgOrigin, imgIPM32,imgIPM35, imgIPMInv32, imgROIGray, imgGaussian, imgThreshold;
cv::Mat tsfIPM, tsfIPMInv, gaussianKernelX, gaussianKernelY, gaussianKernel, XX, YY;
cv::Rect roiLane;

cv::Mat imgBackIPM32, imgBackIPMInv32;
cv::Mat imgFinal,imgMix;
//cv::Mat carmask ;
class Subscribe_And_Publish
{
private:
	ros::Publisher final_pub;
	ros::Publisher IPM_pub;
	ros::Publisher marking_pub;
	ros::Publisher association_pub;
	ros::Publisher gaussian_pub;
	ros::Publisher histogram_pub;
	ros::NodeHandle n;
	int couuuuu;
	ros::Subscriber sub;
	std::string path;
	sensor_msgs::ImagePtr outputmap;
	sensor_msgs::ImagePtr outputIPM;
	sensor_msgs::ImagePtr outputmarking;
	sensor_msgs::ImagePtr outputassociation;
	sensor_msgs::ImagePtr outputgaussian;
	sensor_msgs::ImagePtr outputhistogram;
	vector<cv::String> fn1;
	vector<cv::Mat> ssrc;
	vector<cv::String> fn;
	vector<cv::Mat> DL;
	int c, rotate;
	bool fileFirst;
public:
	Subscribe_And_Publish() {

		onGaussianChange(0, NULL);
		onROIChange(0, NULL);

		sub = n.subscribe("/zed/left/image_raw_color/compressed", 50, &Subscribe_And_Publish::callback, this);
		//sub = n.subscribe("/image_front", 1, &Subscribe_And_Publish::callback, this);
		path = ros::package::getPath("lanetracking");
		c=2760;
		fileFirst = 1;
		rotate = 0;
		final_pub = n.advertise<sensor_msgs::Image> ("final", 1000);
		IPM_pub = n.advertise<sensor_msgs::Image> ("IPM", 1000);
		marking_pub = n.advertise<sensor_msgs::Image> ("marking", 1000);
		association_pub = n.advertise<sensor_msgs::Image> ("association", 1000);
		gaussian_pub = n.advertise<sensor_msgs::Image> ("gaussian", 1000);
		histogram_pub = n.advertise<sensor_msgs::Image> ("histogram", 1000);
	}
	cv::Mat RotateMatrix(cv::Mat input) {
		cv::Mat temp = (Mat_<float>(15, 15)<< 
		-0.032665849,	-0.032829586,	-0.032665849,	-0.032179516,	-0.031385001,	-0.030305529,	-0.028972009,	-0.027421577,	
		-0.025695868,	-0.03571282, -0.038992628,	-0.028125992,	-3.97E-10,	0.039129961,	0.073962547, -0.032179516,
		-0.045399889,	-0.045627456,	-0.045399889,	-0.044723973,	-0.043619733,	-0.042119455,	-0.040266089,	-0.038111258,
		-0.041611332,	-0.030014906,	-4.24E-10,	0.041757893,	0.078929797,	0.087961338, -0.031385001,	-0.044723973,
		-0.049569342,	-0.049817808,	-0.049569342,	-0.048831351,	-0.047625702,	-0.045987636,	-0.043964062,	-0.031711966,
		-4.48E-10,	0.044118907,	0.083392538,	0.09386874,	0.073962547, -0.030305529,	-0.043619733,	-0.048831351, -0.035755143,
		-0.035934366,	-0.035755143,	-0.035222817,	-0.034353163,	-0.033171602,	-4.69E-10,	0.046149608,	0.087230921,	0.099176131,
		0.078929797,	0.039129961,-0.028972009,	-0.042119455,	-0.047625702,	-0.035222817,	-5.05E-10,	-5.08E-10,	-5.05E-10,
		-4.98E-10,	-4.86E-10,	0.047793444,	0.090338051,	0.103741,	0.083392538,	0.041757893,	-3.97E-10, -0.027421577,
		-0.040266089,	-0.045987636,	-0.034353163,	-4.98E-10,	0.049743932,	0.049993273,	0.049743932,	0.04900334,	0.094024822,
		0.10743622,	0.087230921,	0.044118907,	-4.24E-10,	-0.028125992, -0.025695868,	-0.038111258,	-0.043964062,	-0.033171602,
		-4.86E-10,	0.04900334,	0.094024822,	0.094496123,	0.094024822,	0.11015598,	0.090338051,	0.046149608,	-4.48E-10,
		-0.030014906,	-0.038992628, -0.03571282,	-0.041611332,	-0.031711966,	-4.69E-10,	0.047793444,	0.09262497,	0.11182077,
		0.11238128,	0.11182077,	0.094024822, 0.047793444,	-4.69E-10, -0.031711966,	-0.041611332,	-0.03571282, -0.038992628,
		-0.030014906,	-4.48E-10,	0.046149608,	0.090338051,	0.11015598,	0.094024822,	0.094496123,	0.094024822,	0.04900334,
		-4.86E-10,	-0.033171602,	-0.043964062,	-0.038111258,	-0.025695868, -0.028125992,	-4.24E-10,	0.044118907,	0.087230921,
		0.10743622,	0.09262497, 0.04900334,	0.049743932,	0.049993273,	0.049743932,	-4.98E-10,	-0.034353163,	-0.045987636,
		-0.040266089,	-0.027421577, -3.97E-10,	0.041757893,	0.083392538,	0.103741,	0.090338051,	0.047793444,	-4.86E-10,
		-4.98E-10,	-5.05E-10,	-5.08E-10,	-5.05E-10,	-0.035222817,	-0.047625702,	-0.042119455,	-0.028972009, 0.039129961,
		0.078929797,	0.099176131,	0.087230921,	0.046149608,	-4.69E-10,	-0.033171602,	-0.034353163,	-0.035222817,	-0.035755143,
		-0.035934366,	-0.035755143,	-0.048831351,	-0.043619733,	-0.030305529, 0.073962547,	0.09386874,	0.083392538,	0.044118907,
		-4.48E-10,	-0.031711966,	-0.043964062,	-0.045987636,	-0.047625702,	-0.048831351,	-0.049569342,	-0.049817808,	-0.049569342,
		-0.044723973,	-0.031385001, 0.087961338,	0.078929797,	0.041757893,	-4.24E-10,	-0.030014906,	-0.041611332,	-0.038111258,
		-0.040266089,	-0.042119455,	-0.043619733,	-0.044723973,	-0.045399889, -0.045627456,	-0.045399889,	-0.032179516, 0.073962547,
		0.039129961,	-3.97E-10,	-0.028125992,	-0.038992628,	-0.03571282,	-0.025695868,	-0.027421577,	-0.028972009,	-0.030305529,
		-0.031385001,	-0.032179516,	-0.032665849,	-0.032829586,	-0.032665849);
		for (int i = 0; i < 15; i++) {
			for (int j = 0; j < 15; j++) {

			}
		}
		return temp;
	}
	void onGaussianChange(int _x, void *_ptr) {
		float xs[gaussianSizeX], ys[gaussianSizeY];
		float sumx = 0, sumy = 0;

		for (int i = 0; i < gaussianSizeX; i++) {
			float x = 1.0 * i + 0.5 - gaussianSizeX * 0.5;
			xs[i] =(x * x / sigmaX / sigmaX / sigmaX / sigmaX - 1.0 / sigmaX / sigmaX) *exp(-1.0 * x * x / 2 / sigmaX / sigmaX);
			sumx += xs[i];
		}
		/*for (int i = 0; i < gaussianSizeX; i++) {
			float x = 1.0 * i + 0.5 - gaussianSizeX * 0.5;
			ys[i] =(x * x / sigmaX / sigmaX / sigmaX / sigmaX - 1.0 / sigmaX / sigmaX) *exp(-1.0 * x * x / 2 / sigmaX / sigmaX);
			sumy += ys[i];
		}*/
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
        /*gaussianKernel = Mat::zeros(cv::Size(gaussianSizeX, gaussianSizeX), CV_32F);
		gaussianKernel.at<float>(10, 10) = 1;
		gaussianKernelX = Mat(1, gaussianSizeX, CV_32F, xs).clone();
		gaussianKernelY = Mat(gaussianSizeX, 1, CV_32F, xs).clone();
		filter2D(gaussianKernel, XX, gaussianKernel.depth(), gaussianKernelX);
		filter2D(gaussianKernel, YY, gaussianKernel.depth(), gaussianKernelY);
		gaussianKernel = XX + YY;
        gaussianKernelX = Mat(1, gaussianSizeX, CV_32F, xs).clone();
		gaussianKernelY = Mat(1, gaussianSizeY,  CV_32F, ys).clone();*/
		gaussianKernel = Mat::zeros(cv::Size(gaussianSizeX, gaussianSizeX), CV_32F);
		//gaussianKernel.at<float>(10, 10) = 1;
		gaussianKernelX = Mat(1, gaussianSizeX, CV_32F, xs).clone();
		gaussianKernelY = Mat(gaussianSizeY, 1, CV_32F, ys).clone();
		//filter2D(gaussianKernel, XX, gaussianKernel.depth(), gaussianKernelX);
		//filter2D(gaussianKernel, YY, gaussianKernel.depth(), gaussianKernelY);
		gaussianKernel =  gaussianKernelY * gaussianKernelX ;
		cout << setprecision(2) << gaussianKernel;
        //gaussianKernelX = Mat(1, gaussianSizeX, CV_32F, xs).clone();
		//gaussianKernelY = Mat(1, gaussianSizeY,  CV_32F, ys).clone();
  }

  void onROIChange(int _x, void *ptr) {
		roiLane = Rect(roiX, roiY, roiWidth, roiHeight);

		Point2f src[4];
		Point2f dst[4];


		src[0].x = 0;
		src[0].y = 0;
		src[1].x = srcX1;
		src[1].y = roiHeight;
		src[2].x = srcX2;
		src[2].y = roiHeight;
		src[3].x = roiWidth;
		src[3].y = 0;

		dst[0].x = 0;
		dst[0].y = 0;
		dst[1].x = 0;

		dst[1].y = roiHeight;
		dst[2].x = roiWidth;
		dst[2].y = roiHeight;
		dst[3].x = roiWidth;
		dst[3].y = 0;    	
		double alpha_ = 45.8;//17.8//22
		double beta_ = 90;
		double gamma_ = 90;//93//92
		double dist_ = 300;//70//100
		double alpha,dist,beta,gamma;
		alpha =(alpha_-90)* PI/180;	
		beta =(beta_-90)* PI/180;
		gamma =(gamma_-90)* PI/180;
		dist =dist_;

	// Projecion matrix 2D -> 3D
		Mat A1 = (Mat_<float>(4, 3)<< 
			1, 0, -646,
			0, 1, -170,//-170
			0, 0, 0,
			0, 0, 1 );


	// Rotation matrices Rx, Ry, Rz

		Mat RX = (Mat_<float>(4, 4) << 
			1, 0, 0, 0,
			0, cos(alpha), -sin(alpha), 0,
			0, sin(alpha), cos(alpha), 0,
			0, 0, 0, 1 );
		Mat RY = (Mat_<float>(4, 4) << 
			cos(beta), 0, -sin(beta), 0,
			0, 1, 0, 0,
			sin(beta), 0, cos(beta), 0,
			0, 0, 0, 1	);

		Mat RZ = (Mat_<float>(4, 4) << 
			cos(gamma), -sin(gamma), 0, 0,
			sin(gamma), cos(gamma), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1	);

	// R - rotation matrix
		Mat R = RX * RY * RZ;

	// T - translation matrix
		Mat T = (Mat_<float>(4, 4) << 
			1, 0, 0, 0,  
			0, 1, 0, 0,  
			0, 0, 1, -dist/sin(alpha),  
			0, 0, 0, 1); 
	
	// K - intrinsic matrix //387.2628173828125
		Mat K = (Mat_<float>(3, 4) << 
			673.97, 0, 646.008056640625, 0,
			0, 673.97, 170, 0,
			0, 0, 1, 0
			); 


		Mat transformationMat = K *  (T * (R * A1));
	
		tsfIPM = transformationMat.inv();
		tsfIPM = getPerspectiveTransform(dst, src);
		//cout<<tsfIPM<<endl;
		tsfIPMInv = tsfIPM.inv();
	
	}

	cv::Mat detectLanePixels(Mat imgInput, bool rotate) {

		cv::Mat imgROI = Mat(imgInput, roiLane);
		
		warpPerspective(imgROI, imgIPM32, tsfIPM, imgROI.size());
		warpPerspective(imgIPM32, imgIPMInv32, tsfIPMInv, imgROI.size());
		//cv::Mat imgtest = imgClone;
		//warpPerspective(imgtest, imgIPM35, tsfIPM, imgtest.size());
		outputIPM = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgIPM32).toImageMsg();
		outputIPM->header.frame_id = "/map";
		outputIPM->header.stamp = ros::Time::now();
		IPM_pub.publish(outputIPM);
		//cout<<imgIPM32.size()<<endl;

		cvtColor(imgIPM32, imgROIGray, CV_RGB2GRAY);
		/* const char *winipm = "ipm";
		cv::namedWindow(winipm, CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		cv::imshow(winipm, imgROI);*/
		//imgGaussian = imgROIGray;
		if (rotate == 0)
		  filter2D(imgROIGray, imgGaussian, imgIPM32.depth(), gaussianKernel);
		else if (rotate == 1){
			cv::Mat Rotate_matrix = RotateMatrix(gaussianKernel);
			filter2D(imgROIGray, imgGaussian, imgIPM32.depth(), Rotate_matrix);
		}
		
		//sepFilter2D(imgROIGray, imgGaussian, imgIPM32.depth(), gaussianKernelX, gaussianKernelY);
		outputgaussian = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgGaussian).toImageMsg();
		outputgaussian->header.frame_id = "/map";
		outputgaussian->header.stamp = ros::Time::now();
		gaussian_pub.publish(outputgaussian);
		//imgGaussian = imgROIGray.clone();
		cv::Mat imgHist;
		cv::equalizeHist(imgGaussian, imgHist);
		//imgHist = imgGaussian;
		cv::threshold(imgHist, imgThreshold, 255 * (thresholdingQ) / 1000, 255, THRESH_TOZERO);
		//const char *winSegmentation = "threshold";
		//cv::namedWindow(winSegmentation, CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		//cv::imshow(winSegmentation, imgThreshold);
		outputhistogram = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgThreshold).toImageMsg();
		outputhistogram->header.frame_id = "/map";
		outputhistogram->header.stamp = ros::Time::now();
		histogram_pub.publish(outputhistogram);
		cvtColor(imgThreshold, imgBackIPM32, COLOR_GRAY2BGR);
		std::vector<cv::Mat> channels(3);
		split(imgBackIPM32, channels);
		cv::Mat ch1, ch2, ch3;
		ch2 = channels[1];
		ch1 = cv::Scalar::all(0);
		ch3 = cv::Scalar::all(0);
		cv::merge(channels, imgBackIPM32);
		cv::warpPerspective(imgBackIPM32, imgBackIPMInv32, tsfIPMInv, imgROI.size());
		cv::Mat imgModel = imgOrigin(Rect(roiX, roiY, imgBackIPMInv32.cols, imgBackIPMInv32.rows));
		cv::Size img_size = imgInput.size();
		cv::Mat wantedMask = Mat(img_size, CV_8UC1, cv::Scalar(0));
		cv::Mat imgBackIPMInvGray;
		cv::cvtColor(imgBackIPMInv32, imgBackIPMInvGray, cv::COLOR_RGB2GRAY);
		cv::threshold(imgBackIPMInvGray, imgBackIPMInvGray, 1, 255, THRESH_BINARY);
		cv::Mat wantedMaskRoi =
				wantedMask(Rect(0, roiY, imgBackIPMInvGray.cols, imgBackIPMInvGray.rows));
		cv::addWeighted(imgBackIPMInvGray, 1, wantedMaskRoi, 0, 0, wantedMaskRoi);
		cv::addWeighted(imgModel, 0, imgBackIPMInv32, 1, 0, imgModel);
		return wantedMask;        

	}
	void callback(const sensor_msgs::CompressedImageConstPtr& msg) {
        
		LaneDetection ld = LaneDetection();
		
		Mat carmask = cv::imread(path+"/carmask-left-0723.jpg", CV_LOAD_IMAGE_GRAYSCALE);
		
		aps::parabolicMask pmask(carmask.cols, carmask.rows, 1.0 / carmask.rows);
		Mat M = pmask.mkMask();
		M.convertTo(M, CV_8UC1);
		carmask.convertTo(carmask, CV_8UC1);
		carmask = carmask.mul(M);
		
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat input = cv_ptr->image;
		//cv::Mat input = imread("frame4396.jpg");
		ld.initialize_variable(input);
	
		imgOrigin = input.clone();
	
		imgOrigin.convertTo(imgOrigin, -1, contrast / 10.0, -brightness);

		Mat imgMask = imgOrigin.clone();
		Mat img_buf = Mat(imgOrigin.size(), imgOrigin.depth());
		
		Mat toc[] = {carmask, carmask, carmask};
		cv::merge(toc, 3, img_buf);
		
		imgMask = imgMask.mul(img_buf / 255);
	
		Mat imgClone = imgMask.clone();
		
			// LANE SEGMENTATION
		Mat lanemask = detectLanePixels(imgClone, rotate);
		//const char *winSegmentation = "Lane Segmentation";
		//cv::namedWindow(winSegmentation, CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		//cv::imshow(winSegmentation, lanemask);
		Mat imgLaneSeg = imgClone.clone();

		Mat toc2[] = {lanemask, lanemask, lanemask};
		Mat img_buf2 = Mat(imgOrigin.size(), imgOrigin.depth());
		cv::merge(toc2, 3, img_buf2);
		imgLaneSeg = imgLaneSeg.mul(img_buf2 / 255);
			// LANE DETECTION
		Mat imgProcess = imgLaneSeg.clone();
		imgProcess.convertTo(imgProcess, -1, contrast / 10, -brightness);
		//if (!ld.initialize_Img(imgProcess)) ;//continue;
		ld.initialize_Img(imgProcess);
		Mat imgShow = input.clone();
		//bool verbose_lm_detction = 1;
		//cout<<"oooo"<<endl;
		ld.lane_marking_detection(imgProcess, imgShow, 1);
		outputmarking = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgShow).toImageMsg();
		outputmarking->header.frame_id = "/map";
		outputmarking->header.stamp = ros::Time::now();
		marking_pub.publish(outputmarking);
		//bool verbose_seed_gen = 1;
		//cout<<"pppp"<<endl;
		imgShow = input.clone();
		ld.seed_generation(imgProcess, imgShow, 1);
		//cout<<"iiii"<<endl;
		outputassociation = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgShow).toImageMsg();
		outputassociation->header.frame_id = "/map";
		outputassociation->header.stamp = ros::Time::now();
		association_pub.publish(outputassociation);
		ros::Time time = msg->header.stamp;
		imgShow = input.clone();
		bool verbose_validating = true;
		rotate = ld.validating_final_seeds(imgShow, imgFinal, time, 1);
		//cout<<"rrrrrr"<<endl;
			
		outputmap = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgFinal).toImageMsg();
		outputmap->header.frame_id = "/map";
		outputmap->header.stamp = ros::Time::now();
		final_pub.publish(outputmap);
	/*std::string writeFinal = "pred_data_ch1_2760_3119.json";
	std::ofstream ouch(writeFinal.c_str(), std::ofstream::app);
	if (fileFirst) {
		ouch << "{\"lanes\": [";
		fileFirst = false;
	} else
		ouch << "\n{\"lanes\": [";
	ouch.close();*/
		//bool verbose_validating = true;
		//ld.validating_final_seeds(imgShow, imgFinal,ros::Time::now(), verbose_validating);
	}
};
int main (int argc, char** argv) {
	ros::init(argc, argv, "lanetracking_node");
	ROS_INFO("my_node running...");
	Subscribe_And_Publish images;
	ros::spin();
	//destroyAllWindows();
	return 0;
}
