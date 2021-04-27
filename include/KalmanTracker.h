
// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>

// Output
#include <iostream>

// Vector
#include <vector>

#include "ros/ros.h"


using namespace std;


class KalmanTracker {

private:
	int stateSize, measSize, contrSize;
	unsigned int type;
	int foundCount, number;
    bool found;
	double ticks, preTick;

	cv::KalmanFilter kf;
	cv::Mat state;
	cv::Mat meas;
    cv::Mat processNoise;// = (3, 1, CV_32F);
	ros::Time t_now, t_prev;

    cv::Mat temp;

public:
    cv::Mat gain;
	KalmanTracker() {

		found = false;
		foundCount = 0;

		// >>>> Kalman Filter
	    int stateSize = 8;
		int measSize = 8;
		int contrSize = 0;

	    type = CV_64F;
	    kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);

	    state = cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	    meas = cv::Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
	    processNoise = cv::Mat(stateSize, 1, type);
	    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

	    // Transition State Matrix A
	    // Note: set dT at each processing step!
	    // [ 1 0 dT 0  0 0 ]
	    // [ 0 1 0  dT 0 0 ]
	    // [ 0 0 1  0  0 0 ]
	    // [ 0 0 0  1  0 0 ]
	    // [ 0 0 0  0  1 0 ]
	    // [ 0 0 0  0  0 1 ]
	    cv::setIdentity(kf.transitionMatrix);
        kf.transitionMatrix = (cv::Mat_<double>(8, 8) <<
            1,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,
            0,0,1,0,0,0,0,0,
            0,0,0,1,0,0,0,0,
            0,0,0,0,1,0,0,0,
            0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,1);
	    // Measure Matrix H
	    // [ 1 0 0 0 0 0 ]
	    // [ 0 1 0 0 0 0 ]
	    // [ 0 0 1 0 0 0 ]
	    // [ 0 0 0 1 0 0 ]
	    // [ 0 0 0 0 1 0 ]
	    // [ 0 0 0 0 0 1 ]
	    cv::setIdentity(kf.measurementMatrix);

	    // Process Noise Covariance Matrix Q
	    // [ Ex   0   0     0     0    0  ]
	    // [ 0    Ey  0     0     0    0  ]
	    // [ 0    0   Ev_x  0     0    0  ]
	    // [ 0    0   0     Ev_y  0    0  ]
	    // [ 0    0   0     0     Ew   0  ]
	    // [ 0    0   0     0     0    Eh ]
	    cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-6));
        /*
	    kf.processNoiseCov.at<double>(0) = 1e-1;
	    kf.processNoiseCov.at<double>(7) = 1e-1;
	    kf.processNoiseCov.at<double>(14) = 1e-1;
	    kf.processNoiseCov.at<double>(21) = 1e-1;
	    kf.processNoiseCov.at<double>(28) = 1e-1;
	    kf.processNoiseCov.at<double>(35) = 1e-1;
        */

	    // Measures Noise Covariance Matrix R
	    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-6));
	}

	void measIn(const ros::Time t_now, double a, double b, double c, double d) {
		if (!found) // First detection!
        {
            // >>>> Initialization
            /*
            kf.errorCovPre.at<double>(0) = 1e-1;
            kf.errorCovPre.at<double>(7) = 1e-1;
            kf.errorCovPre.at<double>(14) = 1e-1;
            kf.errorCovPre.at<double>(21) = 1e-1;
            kf.errorCovPre.at<double>(28) = 1e-1;
            kf.errorCovPre.at<double>(35) = 1e-1;
            */
	        cv::setIdentity(kf.errorCovPre, cv::Scalar(1e-6));


            // <<<< Initialize the state vector
            state.at<double>(0) = a;
            state.at<double>(1) = b;
            //state.at<double>(2) = 0;
            //state.at<double>(3) = 0;
            state.at<double>(2) = c;
            state.at<double>(3) = d;
            state.at<double>(4) = 0;
            state.at<double>(5) = 0;
            //state.at<double>(2) = 0;
            //state.at<double>(3) = 0;
            state.at<double>(6) = 0;
            state.at<double>(7) = 0;
            kf.statePost = state;
            gain = kf.gain;
            found = true;
            foundCount = 2;
            this->t_now = t_now;
        }


        else {
		    t_prev = this->t_now;
            ros::Duration diff = t_now - t_prev;
            this->t_now = t_now;

            double dT = diff.toSec();

            // >>>>> Kalman Update

            // Calculate velocity of x and y
            double v_a = 0;
            double v_b = 0;
            double v_c = 0;
            double v_d = 0;
            
            if (dT != 0) {
                v_a = (a - state.at<double>(0)) / dT;
                v_b = (b - state.at<double>(1)) / dT;
                v_c = (c - state.at<double>(2)) / dT;
                v_d = (d - state.at<double>(3)) / dT;
            //cout << "vx: " << v_a << " vy: " << v_b << endl;
            }
            //cout << "dT: " << dT << endl;

            // Setup meas vector and update
            meas.at<double>(0) = a;
            meas.at<double>(1) = b;
            //meas.at<double>(2) = vx;
            //meas.at<double>(3) = vy;
            meas.at<double>(2) = c;
            meas.at<double>(3) = d;
            meas.at<double>(4) = v_a;
            meas.at<double>(5) = v_b;
            meas.at<double>(6) = v_c;
            meas.at<double>(7) = v_d;
                        //temp = kf.measurementNoiseCov * meas;
            //meas += temp;
            kf.correct(meas); // Kalman Correction
		    state = kf.predict();
            gain = kf.errorCovPost;
            //cout << "NoiseCov:\n" << kf.measurementNoiseCov << endl << "porcessCov\n" << kf.processNoiseCov << endl;
        }
	}

	vector<double> predict(const ros::Time t_now) {
        vector<double> stateVector;

		t_prev = this->t_now;
        ros::Duration diff = t_now - t_prev;
        //this->t_now = t_now;

        double dT = diff.toSec();
        // Put dT into transitionMatrix
        /*
        if (dT > 0) {
            kf.transitionMatrix.at<double>(2) = dT;
            kf.transitionMatrix.at<double>(9) = dT;
        }
        else {
            kf.transitionMatrix.at<double>(2) = 0;
            kf.transitionMatrix.at<double>(9) = 0;
        }
        */

        // Setup stateVector to return
		if(found) {
			state = kf.predict();
            //state += kf.processNoiseCov * state;
            //state += cv::randn(processNoise, cv::Scalar(0), cv::Scalar::all(sqrt(kf.processNoiseCov.at<float>(0, 0))));
            stateVector.push_back(state.at<double>(0));
            stateVector.push_back(state.at<double>(1));
            stateVector.push_back(state.at<double>(2));
            stateVector.push_back(state.at<double>(3));

		}

		else {
            stateVector.push_back(0);
            stateVector.push_back(0);
            stateVector.push_back(0);
            stateVector.push_back(0);

	    }
        return stateVector;
	}

    int getFoundCount() { return foundCount;}
    void setFoundCount(int num) { foundCount += num;}
    /*
    int getNumber() { return number;}
    void setNumber(int num) { number = num;}
    */
};

