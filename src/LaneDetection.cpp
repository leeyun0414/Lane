//#pragma warning(disable: 4819)
#include "LaneDetection.h"
#include "Tracking.h"
#include "Hungarian.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Dense>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "LaneModels.hpp"
#include "RANSAC.hpp"
//#include "Hungarian.hpp"

#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
#include <cmath>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
// Lane marking definition
const int MAX_LANE_MARKING = 2000;
const int MAX_LW_N = 55;
const int MAX_LW_F = 10;
const int MAX_LW_D = 8;
const int MIN_LW_N = 10;
const int MIN_LW_F = 3;
const int gvalue = 80;

// Lane Marking Grouping
const int MAX_LANE_SEED = 2000;
const int SEED_MARKING_DIST_THRES = 200;
const int VALID_SEED_MARKING_NUMBER_THRES = 6;
const double LOW_LEVEL_ASS_THRES = 1.9;//1.9
const int RANSAC_ITERATIONS = 500;
const int RANSAC_MODEL_SIZE = 3;
const int RANSAC_ERROR_THRESHOLD = 90;
const double RANSAC_INLINERS = 0.75;//;;0.75;
Tracking trackline, trackpoint;
int app = 0, c = 2760;
bool fileFirst = 1, VP_success = 0;
std::set<VALID_LINE> v;
cv::Point2f VP(700, 340);
//bool asso = 0;
int VP_count = 0;
vector<VALID_LINE> v_predict,v_pre;
vector<vector<double>> predictMat;
std::vector<float> coeff3(3);
double getdis(pdd a, pdd b) {   
  return std::abs(a.first - b.first); 
}
double getAns(double a, double b, double c, double y) {   //format solution
	double ans;
	if (a == 0)
		ans = (y - c) / b;
  else 
		ans = (sqrt(abs(pow(b, 2) - 4 * a * (c - y))) - b) / 2 * a;
	return ans; 
}

double threshold_up, threshold_down, threshold_mid, threshold_360;

double valueAt(std::vector<float> &f, float x) {
	float ans = 0.f;
	for (int i = (int)f.size() - 1; i >= 0; --i) ans = ans * x + f[i];
	return ans;
}
cv::Point2f find_vanishing_point(VALID_LINE line1, VALID_LINE line2) {
	cv::Point2f dot_p;
	double dd = 1000;
	int x;
	vector<float> co1(3), co2(3);
	co1[2] = line1.a; 
	co1[1] = line1.b;
	co1[0] = line1.c;
	co2[2] = line2.a;
	co2[1] = line2.b;
	co2[0] = line2.c;
	for (unsigned int xx = 400; xx >= 250; xx -= 1) {
		double x1 = valueAt(co1, xx); 	
		double x2 = valueAt(co2, xx);	
		//cout << "x1:" <<x1 << " x2:"<<x2<<endl;
	  if (dd > abs(x1 - x2)) {
			//cout << "x1:" <<x1 << " x2:"<<x2<<endl;
			if (x2 == 0)
			  cout << co2[0] << " "<< co2[1]<<" " << co1[2]<<endl;
			dd = abs(x1 - x2);
			dot_p.x = (x1 + x2) / 2;
			x = xx;
		}
	}
	dot_p.y = x;
  return dot_p; 
}
vector<cv::Point2f> interpolate(vector<cv::Point2f>& Data) {
	int size = Data.size();
	double dydx = 0, final = 0;
	vector<cv::Point2f> temp;
	for (vector<cv::Point2f>::iterator it = Data.begin(); it != Data.end() - 1; it++) {
		if (it->y >= 380 && it->y <= 450) {
			dydx = ((it+1)->x - it->x) / ((it+1)->y - it->y);
			cv::Point2f tmp(it->x + dydx *  0.5, it->y + 0.5);
			temp.push_back(tmp);
		}
		if (it == Data.end() - 1) {
			final = ((it+1)->x - it->x) / ((it+1)->y - it->y);
		}
	}                                             // linear interpolation*/
	return temp;
}
cv::Point2f check_vanishing_point(vector<VALID_LINE>& detection) {
  cv::Point2f dot_p;
  dot_p.x = 700;
  dot_p.y = 340;
	int count = 0;
	for (int i = 0; i < detection.size(); i++) {
		if (detection[i].times > 5)
		count++;
	}
  if (count == 1){
		vector<float> co1(3);
		co1[0] = detection[0].c;
		co1[1] = detection[0].b;
		co1[2] = detection[0].a;
	  dot_p.x = valueAt(co1, dot_p.y);
	  return dot_p;
	}
	else if (count == 0){
		return dot_p;
	}
	else {
		std::vector<float> co(3);
		//cout << "size:"<<detection.size()<<endl;
		int second_index = -1, max_index = -1, temp = detection[0].times, temp2 = detection[0].times;
		for (int i = 0; i < detection.size(); i++) { //find max 2 detection
			//cout << detection[i].times << endl;
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
		//cout << max_index << " " << second_index << endl;
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
    object.tracker.measIn(time, detection.x, detection.y, 0);
  else {
		object.tracker.measIn(time, detection.x, detection.y, 0);
		object.times++;
	}
  return vp;

}

vector<VALID_LINE> Tracking::updateState(ros::Time& time, vector<VALID_LINE>& detection) { 
    // Predict the state vector
	if (detection.size() ==0) {
		v_pre.clear();
		return v_pre;
	}
	vector<vector<double>> predictMat = predictState(time);
	
	for (int num = 0; num < objectVector.size(); num++) {
		VALID_LINE llp;
		llp.a = predictMat[num][0];
		llp.b = predictMat[num][1];
		llp.c = predictMat[num][2];
		llp.times = objectVector[num].times;
		v_pre.push_back(llp);
	}
	//std::cout << "=================\n" ;
	vector<int> association = getAssociation(detection, predictMat);
	// do data associate

	// Update the state vector
	for (int num = 0; num < association.size(); num++) {
		int index = association[num];
		if (index != -1) {
			objectVector[index].tracker.measIn(time, detection[num].a, detection[num].b, detection[num].c);
			objectVector[index].liner = detection[num];
			if(objectVector[index].tracker.getFoundCount() < 10)
				objectVector[index].tracker.setFoundCount(2);
		}
		// First in
		else {
			ObjectTracker newObject;
			newObject.serial = number;
			newObject.liner = detection[num];
			newObject.tracker.measIn(time, detection[num].a,detection[num].b, detection[num].c);
			incNumber();
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
				if (lossMatrix[num][indexVector[num]] >= 50)
						indexVector[num] = -1;
		}
	
	}
	return indexVector;
}
vector<double>Tracking::getLossVector(VALID_LINE& detection, vector<vector<double> >& predictMatrix) {
	vector<double> lossVector;
	double distance;
	vector<float> co1(3), co2(3);
	co1[2] = detection.a;
	co1[1] = detection.b;
	co1[0] = detection.c;

	for (int num = 0; num < predictMatrix.size(); num++) {
		co2[2] = predictMatrix[num][0];
	  co2[1] = predictMatrix[num][1];
	  co2[0] = predictMatrix[num][2];
		distance = abs(valueAt(co2, 420) -  valueAt(co1, 420));        
		if (distance > 35)
			distance = 1000;
		lossVector.push_back(distance);
	}
	return lossVector;
}
void Tracking::incNumber() {
	number++;
}

void LaneDetection::initialize_variable(cv::Mat img_src) {
     
	img_size = img_src.size();
	img_height = img_src.rows;
	img_width = img_src.cols;
	img_depth = img_src.depth();
	img_roi_height = (int)(380);
	max_lw.resize(img_height);
	min_lw.resize(img_height);
	max_lw_d.resize(img_width);
			// Estimated Lane Width
	for (unsigned int hh = img_roi_height; hh != img_height; ++hh) {
		max_lw[hh] = (int)((MAX_LW_N - MAX_LW_F) * (hh - img_roi_height - 1) /
													(img_size.height - img_roi_height - 1) +
													MAX_LW_F);
		min_lw[hh] = (int)((MIN_LW_N - MIN_LW_F) * (hh - img_roi_height - 1) /
													(img_size.height - img_roi_height - 1) +
													MIN_LW_F);
	}
	unsigned int w = img_width - 1;
	while (img_width - 1 - w < w) {
		max_lw_d[w] = (int)(MAX_LW_D * (abs(w - (img_width - 1) / 2.0)) /
															((img_width - 1) / 2.0));
		max_lw_d[img_width - 1 - w] =
					(int)(MAX_LW_D * (abs(w - (img_width - 1) / 2.0)) /
									((img_width - 1) / 2.0));
		w--;
	}
			
}

void LaneDetection::initialize_Img(cv::Mat img_input) {
	img_gray = cv::Mat(img_size, img_depth);
	if (img_input.channels() == 1) {
		img_input.copyTo(img_gray);
	} 
	else {
		cv::cvtColor(img_input, img_gray, CV_BGR2GRAY);
	}
	lm.resize(0);
	marking_seed.resize(0);		
}
    
void LaneDetection::lane_marking_detection(cv::Mat img_input, cv::Mat img_show, bool verbose) {
                                             //std::cout<<"hhhhh"<<std::endl;
  
  for (unsigned int h = img_roi_height; h != img_height;) {
		int hf_size = 2 + 6 * (h - img_roi_height) / (img_height - img_roi_height);
		std::vector<int> scan_line(img_width);
    // Lane Edge Extraction
    //std::cout<<"hhhhh"<<std::endl;
  
		for (unsigned int w = hf_size + 1; w != img_width - hf_size - 1; ++w) {
			int l_val = 0, r_val = 0;
      //std::cout<<"hhhhh"<<std::endl;
			for (int i = -hf_size; i != 0; ++i) {
				l_val = l_val + img_gray.at<uchar>(h, w + i);
        //std::cout<<"a"<<std::endl;
			}
			for (int i = 1; i <= hf_size; ++i) {
				r_val = r_val + img_gray.at<uchar>(h, w + i);
        //std::cout<<"b"<<std::endl;
			}
			if (((float)(r_val - l_val) / (float)hf_size) > gvalue) 
				scan_line[w] = 1;
			if (((float)(l_val - r_val) / (float)hf_size) > gvalue) 
				scan_line[w] = -1;
    }
    //std::cout<<"hhhhh"<<std::endl;
  
    // Edge Centering
		int e_flag = 0;
		for (unsigned int w = hf_size + 1; w < img_width - hf_size - 1; w++) {
			if (scan_line[w] == 1) {
				if (e_flag >= 0) {
					e_flag++;
				}

				else {
					scan_line[w - (int)(e_flag / 2.0)] = -10;
					e_flag = 0;
				}
			} 
			else if (scan_line[w] == -1) {
				if (e_flag <= 0) {
					e_flag--;
        } 
				else {
					scan_line[w + (int)(e_flag / 2.0)] = 10;
					e_flag = 0;
				}
			} 
			else {
				if (e_flag > 0) {
					scan_line[w - (int)(e_flag / 2.0)] = 10;
					e_flag = 0;
				} 
				else if (e_flag < 0) {
					scan_line[w + (int)(e_flag / 2.0)] = -10;
					e_flag = 0;
				}
			}
		}
    // Extracting Lane Markings - marking flag
		cv::Point2i l_pt, r_pt;
		int m_flag = 0;
		for (unsigned int w = hf_size + 1; w < img_width - hf_size - 1; ++w) {
			if (scan_line[w] == 10) {
				m_flag = 1;  // Set L-edge mflag as 1
				l_pt.x = w;
				l_pt.y = h;
			}
			if (m_flag == 1) { // Set R-edge as 2, after meet L-edge flag 
			
				if (scan_line[w] == -10) {
					m_flag = 2;
					r_pt.x = w;
					r_pt.y = h;
				}
			}
			if (m_flag == 2) { // Close the L to R -edge
				
				if (((r_pt.x - l_pt.x) >= min_lw[h]) && ((r_pt.x - l_pt.x) <= (max_lw[h] + max_lw_d[w]))) { // ex, at 449h (4, 16, 1)
									// lane update
					LANE_MARKING lm_new;
					lm_new.str_p = l_pt;
					lm_new.end_p = r_pt;
					lm_new.cnt_p.x = (int)((l_pt.x + r_pt.x) / 2.0);
					lm_new.cnt_p.y = r_pt.y;
					if (lm_new.cnt_p.x > (int)(img_size.width / 2)) {
						lm_new.inn_p = l_pt;
					} 
					else {
						lm_new.inn_p = r_pt;
					}
					lm_new.size = r_pt.x - l_pt.x;
					lm.push_back(lm_new);
					w = r_pt.x + 0;
					m_flag = 0;
					if (lm.size() >= MAX_LANE_MARKING - 1) { // Error lane size
					
						break;
					}
				}
				m_flag = 0;
			}
		}
		if (lm.size() >= MAX_LANE_MARKING - 1) {
			return;
		}
		h++;
	}
	if (verbose) {
		cv::Mat img_test = img_show.clone();
		for (unsigned int n = 0; n != lm.size(); ++n) {
			int r = rand() % 200 + 10;
			int g = rand() % 200 + 10;
			int b = rand() % 200 + 10;
			cv::line(img_show, lm[n].str_p, lm[n].end_p, CV_RGB(r, g, b), 1, 8, 0);
			cv::line(img_show, lm[n].cnt_p, lm[n].cnt_p, CV_RGB(250, 250, b), 1, 8, 0);
		}
	}
}

void LaneDetection::seed_generation(cv::Mat img_input, cv::Mat img_show, bool verbose) {
  // Initialization

  // STEP 1-1. Generating Seeds
	int flag_dist = 0;
	for (unsigned int ii = 0; ii < lm.size(); ii++) {
		int flag_group = 0;  // Ns in algo. 1
		for (int jj = marking_seed.size() - 1; jj >= 0; jj--) {
			flag_dist = dist_ftn1(ii, marking_seed[jj].index[marking_seed[jj].index.size() - 1], marking_seed[jj].cnt_dir);
			if (flag_dist == 1) {
				flag_group = 1;
				marking_seed[jj].index.push_back(ii);
				if (marking_seed[jj].cnt_dir < -99)	{
					marking_seed[jj].cnt_dir = slope_ftn(lm[ii].cnt_p, marking_seed[jj].cnt_p);
				} 
				else {
					marking_seed[jj].cnt_dir = 0.8 * marking_seed[jj].cnt_dir + 0.2 * slope_ftn(lm[ii].cnt_p, marking_seed[jj].cnt_p);
				}
				marking_seed[jj].cnt_p = lm[ii].cnt_p;
				break;
			}
		}
		if (flag_group == 0) {
			MARKING_SEED seed_new;
			seed_new.flag = 0;
			seed_new.index.resize(0);
			seed_new.index.push_back(ii);
			seed_new.cnt_dir = -100;
			seed_new.cnt_p = lm[ii].cnt_p;
			marking_seed.push_back(seed_new);
		}
	}

	if (verbose) {
		cv::Mat img_test_marking_seed = img_show.clone();
		for (unsigned int ii = 0; ii < marking_seed.size(); ++ii) {
			int r = rand() % 200 + 50;
			int g = rand() % 200 + 50;
			int b = rand() % 200 + 50;
			for (unsigned int jj = 0; jj < marking_seed[ii].index.size(); ++jj) {
				int idx = marking_seed[ii].index[jj];
				cv::line(img_test_marking_seed, lm[idx].str_p, lm[idx].end_p, CV_RGB(r, g, b), 1, 8, 0);
			}
		}	

	}

// STEP 1-2. Seed Validation
	int count_i, count_j;
	for (unsigned int ii = 0; ii < marking_seed.size(); ii++) {
		count_i = marking_seed[ii].index.size();

	// if contained lane marking is less then a certain number
		if (count_i < VALID_SEED_MARKING_NUMBER_THRES) {
			marking_seed[ii].flag = -1;
			continue;
		}
	}

// STEP 1-3. Seed specification
	std::vector<int> val_seed;

	srand((unsigned)time(NULL));
	for (unsigned int ii = 0; ii < marking_seed.size(); ii++) {
		if (marking_seed[ii].flag < 0) {
			continue;
		}
		seed_specification(marking_seed[ii], 1);
		val_seed.push_back(ii);
	}

// STEP 2. Seed Growing - Dist_mat Generation
	int n_of_valid_seeds = val_seed.size();
	std::vector<int> trns_stats;
	trns_stats.resize(n_of_valid_seeds, -1);
	cv::Mat dist_mat = cv::Mat(n_of_valid_seeds, n_of_valid_seeds, CV_32FC1);

	for (int ii = 0; ii < n_of_valid_seeds; ++ii) {
		dist_mat.at<float>(ii, ii) = -1.f;
		for (int jj = ii + 1; jj < n_of_valid_seeds; ++jj) {
			dist_mat.at<float>(ii, jj) = dist_ftn2(val_seed[ii], val_seed[jj]);
			dist_mat.at<float>(jj, ii) = dist_mat.at<float>(ii, jj);
		}
	}

// STEP 2-1. Low Level Association Process #1 - Head -> Tail
	for (int ii = 0; ii < n_of_valid_seeds; ++ii) {
		int cnct_count = 0;
		int cnct_idx = -1;
		for (int jj = 0; jj < ii; ++jj) {
			if (dist_mat.at<float>(jj, ii) > LOW_LEVEL_ASS_THRES) {
				cnct_count++;
				cnct_idx = jj;
			}
		}
		int valid_flag = 0;
		float temp_max = 0;
		int max_id = -1;

		if (cnct_count == 1) {
			for (int kk = cnct_idx; kk < n_of_valid_seeds; kk++) {
				if (dist_mat.at<float>(cnct_idx, kk) > temp_max) {
					temp_max = dist_mat.at<float>(cnct_idx, kk);
					max_id = kk;
				}
			}
			if (max_id == ii) {
				valid_flag = 1;
			}
		}
		if (valid_flag == 1) {
			MARKING_SEED *seed_dst = &marking_seed[val_seed[ii]];
			MARKING_SEED *seed_connect = &marking_seed[val_seed[cnct_idx]];
			count_j = seed_connect->index.size();
			for (int kk = 0; kk < count_j; kk++) {
				seed_dst->index.push_back(seed_connect->index[kk]);
			}
			seed_connect->index.resize(0);
			seed_dst->flag = 1;
			seed_connect->flag = -1;
			seed_specification(*seed_dst, 0);
			seed_dst->str_dir = seed_connect->str_dir;
			seed_dst->str_p = seed_connect->str_p;
			seed_dst->length = seed_dst->length + seed_connect->length;
			for (int ll = cnct_idx; ll < n_of_valid_seeds; ll++) {
				dist_mat.at<float>(cnct_idx, ll) = 0;
			}
			trns_stats[cnct_idx] = ii;
		}
	}

	int temp_val = 0;
	int last_idx = 0;
// STEP 2-2. Low Level Association Process #2 - Head <- Tail
	for (int ii = n_of_valid_seeds - 1; ii >= 0; ii--) {
		int cnct_count = 0;	
		int cnct_idx = -1;
		for (int jj = ii + 1; jj < n_of_valid_seeds; jj++) {
			if (dist_mat.at<float>(ii, jj) > LOW_LEVEL_ASS_THRES) {
				cnct_count++;
				cnct_idx = jj;
			}
		}
		int valid_flag = 0;
		int temp_max = 0;
		int max_id = -1;
		if (cnct_count == 1) {
			for (int kk = 0; kk < cnct_idx; kk++) {
				if (dist_mat.at<float>(kk, cnct_idx) > temp_max) {
					temp_max = dist_mat.at<float>(kk, cnct_idx);
					max_id = kk;
				}
			}
			if (max_id == ii)	{
				valid_flag = 1;
			}
		}
		if (valid_flag == 1) {
			last_idx = cnct_idx;
			temp_val = trns_stats[last_idx];
			while (temp_val != -1) {
				last_idx = temp_val;
				temp_val = trns_stats[last_idx];
			}
			cnct_idx = last_idx;
			MARKING_SEED *seed_dst = &marking_seed[val_seed[ii]];
			MARKING_SEED *seed_connect = &marking_seed[val_seed[cnct_idx]];
			count_j = seed_connect->index.size();
			for (int kk = 0; kk < count_j; kk++) {
				seed_dst->index.push_back(seed_connect->index[kk]);
			}
			seed_connect->index.resize(0);
			seed_dst->flag = 1;
			seed_connect->flag = -1;
			seed_specification(*seed_dst, 0);
			seed_dst->end_dir = seed_connect->end_dir;
			seed_dst->end_p = seed_connect->end_p;
			seed_dst->length = seed_dst->length + seed_connect->length;
			for (int ll = 0; ll < cnct_idx; ll++) {
				dist_mat.at<float>(ll, cnct_idx) = 0;
			}
		}
	}

	if (verbose) {
		cv::Mat img_test_raw_level_assoc = img_show.clone();
		for (unsigned int ii = 0; ii < marking_seed.size(); ++ii) {
			if (marking_seed[ii].flag < 0) {
				continue;
			}
			int r = rand() % 200 + 50;
			int g = rand() % 200 + 50;
			int b = rand() % 200 + 50;

			MARKING_SEED seed = marking_seed[ii];
			for (unsigned int jj = 0; jj < seed.index.size(); ++jj) {
				int idx = seed.index[jj];
				cv::line(img_show, lm[idx].str_p, lm[idx].end_p, CV_RGB(r, g, b), 1, 8, 0);
			}

	//std::cout<<linecolor.size()<<std::endl;
		}

	} 
}

void LaneDetection::seed_specification(MARKING_SEED &marking_seed_curr, int mode) {
	float temp_x = 0;
	float temp_y = 0;

	std::vector<float> coeff2;
	std::vector<cv::Point2f> points;
	coeff2.resize(2);
	int n_of_lm = marking_seed_curr.index.size();

	for (int ii = 0; ii < n_of_lm; ii++) {
		int idx_lm = marking_seed_curr.index[ii];
		temp_x += (float)lm[idx_lm].cnt_p.x;
		temp_y += (float)lm[idx_lm].cnt_p.y;
		points.push_back(lm[idx_lm].cnt_p);
	}
	poly2(points, points.size(), coeff2);
	marking_seed_curr.cnt_dir = CV_PI / 2 - atan(coeff2[1]);
	marking_seed_curr.cnt_p.x = (int)(temp_x / n_of_lm);
	marking_seed_curr.cnt_p.y = (int)(temp_y / n_of_lm);

	if (mode == 1) {
		marking_seed_curr.str_p = lm[marking_seed_curr.index[0]].cnt_p;
		marking_seed_curr.end_p = lm[marking_seed_curr.index[n_of_lm - 1]].cnt_p;
		marking_seed_curr.length =
		length_ftn(marking_seed_curr.str_p, marking_seed_curr.end_p);
		if (n_of_lm < VALID_SEED_MARKING_NUMBER_THRES) {
			marking_seed_curr.end_dir = marking_seed_curr.cnt_dir;
			marking_seed_curr.str_dir = marking_seed_curr.cnt_dir;
		} 	
		else {
			int n_samples = std::max(5, (int)(0.3f * n_of_lm));
			poly2(points, n_samples, coeff2);
			marking_seed_curr.str_dir = (float)(CV_PI / 2 - atan(coeff2[1]));
			points.resize(0);
			for (int ii = n_of_lm - 1; ii >= n_of_lm - n_samples; ii--) {
				int idx_i = marking_seed_curr.index[ii];
				points.push_back(lm[idx_i].cnt_p);
			}
			poly2(points, n_samples, coeff2);
			marking_seed_curr.end_dir = (float)(CV_PI / 2 - atan(coeff2[1]));
		}
	}
}

int LaneDetection::validating_final_seeds(cv::Mat img_show, cv::Mat &img_result, ros::Time time, bool verbose) {
  cv::Mat img_test_val = img_show.clone();
  std::vector<float> coeff(3);
  cout << "---------------frame-----------------" << endl;
  for (unsigned int ii = 0; ii < marking_seed.size(); ++ii) {
    if ((marking_seed[ii].flag == 0) && (marking_seed[ii].index.size() > 70)) {//70
      marking_seed[ii].flag = 1;
    }
    if (marking_seed[ii].flag < 1) {
      continue;
    }
    float length = length_ftn(marking_seed[ii].end_p, marking_seed[ii].str_p);
    //60
    if (length < 60) {
      marking_seed[ii].flag = 0;
      continue;
    }
    if (marking_seed[ii].length < 60) {
      marking_seed[ii].flag = 0;
      continue;
    }
    if ((length == marking_seed[ii].length) && (length < 60)) {
      marking_seed[ii].flag = 0;
      continue;
    }
    std::vector<cv::Point2f> pts;
    for (unsigned int pp = 0; pp < marking_seed[ii].index.size(); pp++) {
      //cout << "point++";
      int idx_lm = marking_seed[ii].index[pp];
      pts.push_back(lm[idx_lm].cnt_p);
    }
    /*if (VP_success) {
      pts.push_back(VP);
      cout <<"push!!";
    }*/
		//pts.push_back(VP);
    //auto parabola = RANSAC_Parabola(RANSAC_ITERATIONS, RANSAC_MODEL_SIZE,
    //                    static_cast<int>(RANSAC_INLINERS * pts.size()),
    //                    RANSAC_ERROR_THRESHOLD, pts);
		//vector<float> temp(3);
		//temp[0] = parabola.c;
		//temp[1] = parabola.b;
		//temp[2] = parabola.a;
    //if (abs(valueAt(temp, VP.y) - VP.x) > 10 && VP_success) {
		//	marking_seed[ii].flag = 0;
		//  continue;
	  //	}
		//else { 
			//pts.push_back(VP);
			/*vector<cv::Point2f> interpolation =
			 interpolate(pts);
			for (int ii = 0; ii < interpolation.size(); ii++)
				pts.push_back(interpolation[ii]);*/
      auto parabola = RANSAC_Parabola(RANSAC_ITERATIONS, RANSAC_MODEL_SIZE,
                        static_cast<int>(RANSAC_INLINERS * pts.size()),
                        RANSAC_ERROR_THRESHOLD, pts);
			if (VP.x > 460 && VP.x < 800) {
        if (std::abs(parabola.a) > 0.0023) {//0.0013
          marking_seed[ii].flag = 0;
          continue;
        }
			}
			else {
        if (std::abs(parabola.a) > 0.013) {//0.0013
          marking_seed[ii].flag = 0;
          continue;
        }
			}
		  if (!pts.empty()) {
			  v.insert(VALID_LINE{parabola.a, parabola.b, parabola.c, 2});
	   	}
		//}
  }
  // remove error lanes
	auto prev = v.begin();
	threshold_up = 70;//70
	threshold_360 = 150;
	threshold_mid = 250;
	threshold_down = 0;
	std::vector<VALID_LINE> tmp, res; 
	if (v.size() > 1) {
		tmp.push_back(*prev);
		for (auto it = std::next(v.begin()); it != v.end(); it++) {
			pdd pre_up, pre_down, next_up, next_down, pre_mid, next_mid, pre_360, next_360;
			pre_up = prev->getUpPoint();
			pre_down = prev->getDownPoint();
			next_up = it->getUpPoint();
			next_down = it->getDownPoint();
			pre_mid = prev->getMidPoint();
			next_mid = it->getMidPoint();
			pre_360 = prev->get360Point();
			next_360 = it->get360Point();
			if (getdis(pre_up, next_up) < threshold_up && getdis(pre_down, next_down) < threshold_down) {
				tmp.push_back(*it);
			} 
			else {
				res.push_back(tmp[tmp.size() / 2]);
				tmp.clear();
				tmp.push_back(*it);
			}
			prev = it;
		}
		if (tmp.size()) res.push_back(tmp[0]);
	} 
	else if (v.size() == 1) {
		res.push_back(*v.begin());
	}

	v_predict = trackline.updateState(time,res);
	cv::circle(img_test_val, check_vanishing_point(v_predict), 1, CV_RGB(255, 255, 255), 5, 4, 0);
	cout << check_vanishing_point(v_predict) << endl;
	VP = trackpoint.updatePoint(time, check_vanishing_point(v_predict));
	cv::circle(img_test_val, VP, 1, CV_RGB(0, 0, 255), 5, 4, 0);
	int rotate = 0;
	if (VP.x > 800)
	  rotate = 1;
	/*for (auto it: res) {
		cv::Point2f dot_p;
		coeff[2] = it.a;
		coeff[1] = it.b;
		coeff[0] = it.c;
		for (unsigned int xx = 380; xx <= img_height; ++xx) {//380
			dot_p.y = xx;
			dot_p.x = valueAt(coeff, dot_p.y); 
			cv::circle(img_test_val, dot_p, 1,CV_RGB(255, 0, 0),  2, 4, 0);
		} //std::cout<<"end"<<std::endl<<std::endl;
	}*/
	for (auto it: v_predict) {
		if (it.times < 6)
		  continue;
		cv::Point2f dot_p;
		coeff[2] = it.a;
		coeff[1] = it.b;
		coeff[0] = it.c;
		for (unsigned int xx = 380; xx <= img_height; ++xx) {
			dot_p.y = xx;
			dot_p.x = valueAt(coeff, dot_p.y); 
			cv::circle(img_test_val, dot_p, 1,CV_RGB(0, 255, 0),  2, 1, 0);
		} //std::cout<<"end"<<std::endl<<std::endl;
	}

	img_result = img_test_val.clone();
	v.clear();
	v_pre.clear();
	return rotate;
}

int LaneDetection::dist_ftn1(int s_i, int s_j, double slope) {
  // For Seed Generation
	CvPoint i, j;
	i = lm[s_i].cnt_p;
	j = lm[s_j].cnt_p;
	double value = (i.x - j.x) * (i.x - j.x) + (i.y - j.y) * (i.y - j.y);  // distance between two cnt_p

	if ((lm[s_i].str_p.x > lm[s_j].end_p.x) || (lm[s_i].end_p.x < lm[s_j].str_p.x))
		return 0;
	if (value < SEED_MARKING_DIST_THRES) return 1;
		return 0;
}
float LaneDetection::dist_ftn2(int i, int j) {
  // For Low level Association
	if (marking_seed[i].end_p.y > marking_seed[j].str_p.y) {
		return 0;
	}

	std::vector<float> slp;
	slp.resize(7);
	slp[0] = marking_seed[i].end_dir;
	slp[1] = marking_seed[j].str_dir;
	if ((abs(slp[0] - slp[1]) > 0.5) && (abs(abs(slp[0] - slp[1]) - 3.141592) < 2.641592))
		return 0;

	slp[2] = slope_ftn(marking_seed[i].cnt_p, marking_seed[j].cnt_p);
	slp[3] = slope_ftn(marking_seed[i].str_p, marking_seed[j].str_p);
	slp[4] = slope_ftn(marking_seed[i].str_p, marking_seed[j].end_p);
	slp[5] = slope_ftn(marking_seed[i].end_p, marking_seed[j].str_p);
	slp[6] = slope_ftn(marking_seed[i].end_p, marking_seed[j].end_p);

	float slp_mean = (slp[0] + slp[1] + slp[2] + slp[3] + slp[4] + slp[5] + slp[6]) / 7;
	float temp = 0;
	for (int i = 0; i < 7; i++) {
		temp += (slp[i] - slp_mean) * (slp[i] - slp_mean);
	}
	float slp_var = temp / 7;
	if (slp_var > 0.5) {
		return 0;
	}
	float sig = 0.25;
	float diff1, diff2;
	diff1 = slp[0] - slp[6];
	diff2 = slp[1] - slp[3];
	if (((abs(diff1) + abs(diff2)) > 0.6) && (diff1 * diff2 > 0))	{
		return 0;
	}
	if (abs(diff1) > 1.570796) {
		diff1 = abs(diff1 - 3.141592);
	}
	if (abs(diff2) > 1.570796) {
		diff2 = abs(diff2 - 3.141592);
	}

	return (float)(exp(-(diff1) * (diff1) / sig * sig) + exp(-(diff2) * (diff2) / sig * sig));

}

float LaneDetection::slope_ftn(cv::Point2f pos1, cv::Point2f pos2) {
	if (pos1.y > pos2.y) std::swap(pos1, pos2);
		return (float)(acos((float)((pos2.x - pos1.x) /
						sqrt((float)((pos1.x - pos2.x) * (pos1.x - pos2.x) +
													(pos1.y - pos2.y) * (pos1.y - pos2.y))))));
}
float LaneDetection::length_ftn(cv::Point2f str_p, cv::Point2f end_p) {
	return sqrt((float)(str_p.x - end_p.x) * (str_p.x - end_p.x) +
              (float)(str_p.y - end_p.y) * (str_p.y - end_p.y));
}

void LaneDetection::poly2(std::vector<cv::Point2f> points, int n, std::vector<float> &coeff) {
	double x, y;
	Eigen::MatrixXd aMat, bMat, xMat;
	aMat.resize(n, 2);
	bMat.resize(n, 1);
	xMat.resize(2, 1);
	for (int rr = 0; rr != n; ++rr) {
		x = points[rr].y;  // swap x and y to better model line
		y = points[rr].x;
		aMat(rr, 0) = 1;
		aMat(rr, 1) = x;
		bMat(rr, 0) = y;
	}
	xMat = aMat.colPivHouseholderQr().solve(bMat);
	coeff[0] = xMat(0, 0);
	coeff[1] = xMat(1, 0);
}

