#ifndef YOLOTRACKER_H
#define YOLYTRACKER_H

#include "KalmanTracker.h"
#include "Hungarian.h"
#include "LaneDetection.h"

#include <iostream>
#include <sstream>
#include <vector>
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/progressive_morphological_filter.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <jsk_recognition_msgs/BoundingBoxArray.h>
//#include <jsk_recognition_msgs/BoundingBox.h>
//#include <visualization_msgs/MarkerArray.h>
// For opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

struct ObjectTracker {
    KalmanTracker tracker;
    int id;
    VALID_LINE liner;
    int times = 0;
    float head;
};

class Tracking {
    private:
        vector<ObjectTracker> objectVector;
        ObjectTracker object;
        //vector<vector<double> > predictMatrix;
        double currentTime, previousTime;
        int seq, number;
        bool init;
        //vector<cv::Scalar> randomColor;
        //std_msgs::Header _velodyne_header;
        std::vector<VALID_LINE> line_pre, line_now;
	    ros::Publisher pub_predict, cluster_pub, prediction_pub, marker_pub;
        //std::ofstream outfile;
        vector<VALID_LINE> v_pre, v_predict;
        cv::Point2f VP;
        vector<vector<double>> predictMat;
        double valueAt(std::vector<float> &f, float x);
    public:
        Tracking(){}
        ~Tracking(){}
        vector<int> getAssociation(vector<VALID_LINE>&, vector<vector<double> >&);
        vector<vector<double> > predictState(ros::Time&);
        vector<cv::Point2f> predictPoint(ros::Time&);
        cv::Point2f updatePoint(ros::Time&, cv::Point2f );
        vector<VALID_LINE> updateState(ros::Time&, vector<VALID_LINE>&, int);
        vector<double> getLossVector(VALID_LINE&, vector<vector<double> >&);
        cv::Point2f check_vanishing_point(vector<VALID_LINE>& detection);
        double check_slope(vector<VALID_LINE>& detection);
        cv::Point2f find_vanishing_point(VALID_LINE line1, VALID_LINE line2);        //void clusterCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        double output(cv::Mat img_test_val, std::vector<VALID_LINE> res, ros::Time time, double eot);
        void incNumber();
};


#endif
