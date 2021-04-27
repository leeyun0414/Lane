//#pragma warning(disable: 4819)

#include "Tracking.h"
#include "Hungarian.h"
//#include "LaneDetection.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Dense>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "LaneModels.hpp"
#include <Eigen/Dense>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
#include <cmath>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>

double Tracking::valueAt(std::vector<float> &f, float x) {
	float ans = 0.f;
	for (int i = (int)f.size() - 1; i >= 0; --i) ans = ans * x + f[i];
	return ans;
}

cv::Point2f Tracking::find_vanishing_point(VALID_LINE line1, VALID_LINE line2) {
	cv::Point2f dot_p;
	double dd = 1000;
	int x;
	vector<float> co1(4), co2(4);
	co1[3] = line1.a; 
	co1[2] = line1.b;
	co1[1] = line1.c;
	co1[0] = line1.d;
	co2[3] = line2.a;
	co2[2] = line2.b;
	co2[1] = line2.c;
	co2[0] = line2.d;
	for (unsigned int xx = 800; xx >= 600; xx -= 1) {
		double x1 = valueAt(co1, xx); 	
		double x2 = valueAt(co2, xx);	
	  if (dd > abs(x1 - x2)) {
			dd = abs(x1 - x2);
			dot_p.x = (x1 + x2) / 2;
			x = xx;
		}
	}
	dot_p.y = x;
	/*double a, b;
	a = valueAt(co1, 320);
	b = valueAt(co2, 320);
	dot_p.y = 320;
	dot_p.x = (a + b) / 2;*/
  return dot_p; 
}
double Tracking::check_slope(vector<VALID_LINE>& detection) {
	double slope_sum = 0;
	for (int i = 0; i < detection.size(); i++) {
		if (detection[i].times < 3) {
			continue;
		} else {
			vector<float> co1(4);
			co1[0] = detection[i].d;
			co1[1] = detection[i].c;
			co1[2] = detection[i].b;
			co1[3] = detection[i].a;
      slope_sum += valueAt(co1, 400) - valueAt(co1, 500);
		}
	}	
	return slope_sum;
}
cv::Point2f Tracking::check_vanishing_point(vector<VALID_LINE>& detection) {
  cv::Point2f dot_p;
  dot_p.x = 1024;
  dot_p.y = 720;
	int count = 0;
	for (int i = 0; i < detection.size(); i++) {
		if (detection[i].times > 5)
		count++;
	}
  if (count == 1){
		vector<float> co1(4);
		co1[0] = detection[0].d;
		co1[1] = detection[0].c;
		co1[2] = detection[0].b;
		co1[3] = detection[0].a;
		
	  dot_p.x = valueAt(co1, dot_p.y);
	  return dot_p;
	}
  else if (count == 0){
	  return dot_p;
  }
  else {
	int second_index = -1, max_index = -1, temp = detection[0].times, temp2 = detection[0].times;
	for (int i = 0; i < detection.size(); i++) { //find max 2 detection
		
	  if (detection[i].times >= temp) {
		second_index = max_index;
		temp = detection[i].times;
		max_index = i;
	  }
	  else if (detection[i].times >= temp2 || second_index == -1) {
		second_index = i;
		temp2 = detection[i].times;
	  }
	} 
	dot_p = find_vanishing_point(detection[max_index], detection[second_index]);
	return dot_p;
  }
}
vector<cv::Point2f> Tracking::predictPoint(ros::Time& time) {
	vector<cv::Point2f> predictVector;
	
	vector<double> temp = object.tracker.predict(time);
	cv::Point2f tmp(temp[0], temp[1]);
	predictVector.push_back(tmp);
	return predictVector;
}
cv::Point2f Tracking::updatePoint(ros::Time& time, cv::Point2f detection) { 
	vector<cv::Point2f> predictVector = predictPoint(time);

	cv::Point2f vp;
	vp.x = predictVector[0].x;
	vp.y = predictVector[0].y;
	if (object.times)
    object.tracker.measIn(time, detection.x, detection.y, 0, 0);
  else {
		object.tracker.measIn(time, detection.x, detection.y, 0, 0);
		object.times++;
	}
  return vp;
}

vector<VALID_LINE> Tracking::updateState(ros::Time& time, vector<VALID_LINE>& detection, int number) { 
    // Predict the state vector
	v_pre.clear();
	if (detection.size() ==0) {
		return v_pre;
	}
	vector<vector<double>> predictMat = predictState(time);
	
	for (int num = 0; num < objectVector.size(); num++) {
		VALID_LINE llp;
		llp.a = predictMat[num][0];
		llp.b = predictMat[num][1];
		llp.c = predictMat[num][2];
		llp.d = predictMat[num][3];
		llp.times = objectVector[num].times;
		llp.cov = objectVector[num].tracker.gain.at<double>(3, 3);
		llp.head = objectVector[num].head;
		v_pre.push_back(llp);
	}
	vector<int> association = getAssociation(detection, predictMat);
	// do data associate
  //cout <<"assssssss"<<endl;
	// Update the state vector
	for (int num = 0; num < association.size(); num++) {
		int index = association[num];
		if (index != -1) {
			objectVector[index].tracker.measIn(time, detection[num].a, detection[num].b, detection[num].c, detection[num].d);
			objectVector[index].head = detection[num].head;
			//cout <<objectVector[index].tracker.gain;
		
			objectVector[index].liner = detection[num];
			if(objectVector[index].tracker.getFoundCount() < 5)
				objectVector[index].tracker.setFoundCount(number);
		}
		// First in
		else {
			ObjectTracker newObject;
			newObject.liner = detection[num];
			newObject.tracker.measIn(time, detection[num].a,detection[num].b, detection[num].c, detection[num].d);
			newObject.tracker.setFoundCount(number);
			newObject.head = detection[num].head;
			objectVector.push_back(newObject);
		}
	}
	// Delete the tracker if it doesn't be detect in several frame
	for (vector<ObjectTracker>::iterator it = objectVector.begin(); it != objectVector.end(); it++) {
		it->times = it->tracker.getFoundCount();
		it->tracker.setFoundCount(-1);
		if (it->tracker.getFoundCount() <= 0) {
			objectVector.erase(it);
			it--;
		}	
	}
	return v_pre;
}
vector<vector<double> > Tracking::predictState(ros::Time& time) {
	vector<vector<double>> predictMat;
	for (int num = 0; num < objectVector.size(); num++) {
		predictMat.push_back(objectVector[num].tracker.predict(time));
	}
	return predictMat;
}
vector<int> Tracking::getAssociation(vector<VALID_LINE>& detection, vector<vector<double> >& predictMatrix) {
	vector<vector<double> > lossMatrix(detection.size());
	vector<int> indexVector;
	if (!init) {
		for (int resultNum = 0; resultNum < detection.size(); resultNum++)
			indexVector.push_back(-1);
		init = 1;
	}
  
	else { 
		int resultIndex = 0;
		
		for (int resultNum = 0; resultNum < detection.size(); resultNum++) {
			lossMatrix[resultNum] = getLossVector(detection[resultNum], predictMatrix);
		}
		HungarianAlgorithm solver;
	
		double cost = solver.Solve(lossMatrix, indexVector);
		for (int num = 0; num < indexVector.size(); num++) {
			if (indexVector[num] != -1)
				if (lossMatrix[num][indexVector[num]] >= 1000)
						indexVector[num] = -1;
		}
	
	}
	return indexVector;
}
vector<double>Tracking::getLossVector(VALID_LINE& detection, vector<vector<double> >& predictMatrix) {
	vector<double> lossVector;
	double distance;
	vector<float> co1(4), co2(4);
	co1[3] = detection.a;
	co1[2] = detection.b;
	co1[1] = detection.c;
	co1[0] = detection.d;

	for (int num = 0; num < predictMatrix.size(); num++) {
		co2[3] = predictMatrix[num][0];
		co2[2] = predictMatrix[num][1];
	  co2[1] = predictMatrix[num][2];
	  co2[0] = predictMatrix[num][3];
		distance = 1.5*abs(valueAt(co2, 1000) -  valueAt(co1, 1000)) + 0.5*abs(valueAt(co2, 1200) -  valueAt(co1, 1200));
		distance /= 2;  
		//cout <<distance<<endl;      
		if (distance > 300)
			distance = 1000;
		lossVector.push_back(distance);
	}
	return lossVector;
}
double Tracking::output(cv::Mat img_test_val, std::vector<VALID_LINE> res, ros::Time time, double rot) {
	bool cv = true, kf = true; 
 
  v_predict = updateState(time, res, 2);
  vector<float> coeff(4);
	
	cv::Point2f VP_meas = check_vanishing_point(v_predict);
	cv::circle(img_test_val, VP_meas, 1, CV_RGB(255, 255, 255), 5, 4, 0);
	//std::string writeFileName = "pred_data.json";
  //std::ofstream out(writeFileName.c_str(), std::ofstream::app);
  //int fla = 0;
	if (VP_meas.y != 1024)
	  VP = updatePoint(time, VP_meas);
	cv::circle(img_test_val, VP, 1, CV_RGB(0, 0, 255), 5, 4, 0);
	double rotate = 0;
	//img_test_val(cv::Rect(0, 0, 1280,720)) = 0;
	//if (std::abs(rot) > 90)
	rotate = VP.x - 1024;
	//rotate = check_slope(v_predict);
	//cout <<"retate:" <<rotate<<endl;
	if (cv) {
		cout <<"CV_line:" << res.size()<<endl;
		for (auto it: res) {
			cv::Point2f dot_p;
			coeff[3] = it.a;
			coeff[2] = it.b;
			coeff[1] = it.c;
			coeff[0] = it.d;
			/*if (fla)
        out << ", [";
      else {
        out << "[";
        fla = 1;
      }
      for (int yy = 10; yy != 720; yy += 10) {
        dot_p.y = yy;
        dot_p.x = valueAt(coeff, dot_p.y);
        if (yy < 380) {
          dot_p.x = -2;
          out << (int)dot_p.x << ", ";
        } 
				else {
          if (dot_p.x > 1280 || dot_p.x < 0) dot_p.x = -2;
            out << (int)dot_p.x;
          if (yy != 710)
            out << ", ";
          else
            out << "]";
        }
      }*/
			for (unsigned int xx = it.head; xx <= 1536; ++xx) {//380
				dot_p.y = xx;
				dot_p.x = valueAt(coeff, dot_p.y); 
				//cout <<dot_p.x<<" ";
				if (dot_p.x < 0 || dot_p.x > 2048)
				  break;
				cv::circle(img_test_val, dot_p, 1,CV_RGB(255, 0, 0),  2, 4, 0);
			}
		}
	}

	if (kf) {
		cout <<"KF_line:"<< v_predict.size()<<endl;
		for (auto it: v_predict) {
			if (it.times < 3) {
				//cout <<"asdasd:"<<it.cov<<"aaa:"<<it.times<<endl;
				continue;
			}
			//cout << it.cov<<endl;
			cv::Point2f dot_p;
			coeff[3] = it.a;
			coeff[2] = it.b;
			coeff[1] = it.c;
			coeff[0] = it.d;

			/*if (fla)
        out << ", [";
      else {
        out << "[";
        fla = 1;
      }
      for (int yy = 10; yy != 720; yy += 10) {
        dot_p.y = yy;
        dot_p.x = valueAt(coeff, dot_p.y);
        if (yy < 380) {
          dot_p.x = -2;
          out << (int)dot_p.x << ", ";
        } 
				else {
          if (dot_p.x > 1280 || dot_p.x < 0) dot_p.x = -2;
            out << (int)dot_p.x;
          if (yy != 710)
            out << ", ";
          else
            out << "]";
        }
      }*/
			dot_p.y = 1000;
			dot_p.x = valueAt(coeff, dot_p.y); 
			cv::putText(img_test_val, to_string(it.times), dot_p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
			
			for (unsigned int xx = it.head; xx <= 1536; xx= xx + 1) {
				dot_p.y = xx;
				dot_p.x = valueAt(coeff, dot_p.y); 
				if (dot_p.x < 0 || dot_p.x > 2048)
				  break;
				//cv::putText(img_test_val, to_string((int)dot_p.x) + ", " + to_string((int)dot_p.y), dot_p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
			  
				cv::circle(img_test_val, dot_p, 1,CV_RGB(0, 255, 0),  2, 1, 0);
				dot_p.x += it.cov * 1000 - 730;
				//cv::circle(img_test_val, dot_p, 1,CV_RGB(255, 255, 255),  1, 1, 0);
				dot_p.x -= 2*(it.cov * 1000 - 730);
				//cv::circle(img_test_val, dot_p, 1,CV_RGB(255, 255, 255),  1, 1, 0);
			} 
		}
	}
  
	cout<<endl << "rotate:" <<rotate<<endl;
	v_predict.clear();
	return rotate;
}
