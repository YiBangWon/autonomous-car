#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "ackermann_msgs/AckermannDriveStamped.h"

#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "opencv/cv.hpp"
#include <iostream>
#include "preprocess.h"
#include "laneDetector.h"
#include "controller.h"
#include "laneInfo.h"
#include "patternCheck.h"

// global variables
LaneDetector laneDetector; // LaneDetector 클래스 변수 => 차선 인식을 위해 필요한 필드변수와 메소드를 가지고 있음
LaneDetector laneDetector_future;
PreProcess preProcess; // PreProcess 클래스 변수 = > 전처리 과정을 위해 필요한 필드변수와 메소드를 가지고 있음
PreProcess preProcess_future;
Controller controller;
PatternCheck patternCheck;
PatternCheck patternCheck_future;
//ImageConverter ic;



// erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) + speed_to_erpm_offset
// servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset

/*
float car_direction = 1; // 1: front & -1: back 
float car_run_speed = 0.45;
//float car_angle = 0; // 1: left & -1: right
//float car_angle_speed = 2.0;
float Max = 0.29;
float Min = -0.34;
float car_angle = -0.34; // 1: left & -1: right
float car_angle_speed = 1.0;
float interval = 0.01;
bool isClassConstructed = false;
*/

float speed_Max = 1.3;
float speed_Min = 0.45;
float car_direction = 1; // 1: front & -1: back 
float car_run_speed = 0.45;//speed_Min;
float angle_Max = 0.29;
float angle_Min = -0.34;
float car_angle = 0.03; // 1: left & -1: right
float car_angle_speed = 1.0;
int patternCount = 0;
bool isClassConstructed = false;

float interval = 0.01;
bool isAlert = false;


//pattern_check
//int frameNum = 0;
//int countStartLine = 0;
//


using namespace cv;
using namespace std;


// Works as a main function
cv::Mat imageProcess(cv::Mat frame){
	// Variables 
	cv::Mat resultFrame;
	cv::Mat originalFrame = frame; 
	
	//if(!isClassConstructed){
	//	laneInfo = LaneInfo();
	//	patternCheck = PatternCheck(originalFrame, laneInfo);
	//}
	//else patternCheck.updatePatternCheckFrame(originalFrame);
	//patternCheck.checkPattern();

	// first constructing objects = preProcess, laneDetector, controller
	if(!isClassConstructed) {
		// PreProcess(cv::Mat& frame, int upperLeftX, int upperLeftY, int lowerRightX, int lowerRightY) => initializing - Road Area ROI
	//	preProcess = PreProcess(originalFrame, 150, (int)(frame.rows * 2.5 / 3), frame.cols - 20, frame.rows -10); // original roi	
		preProcess = PreProcess(originalFrame, 150, (int)(frame.rows * 2.5 / 3), frame.cols - 20, frame.rows -10);	// upper roi
	//	preProcess_future =  PreProcess(originalFrame, 150, (int)(frame.rows * 2.5 / 3) - 50, frame.cols - 20, frame.rows -10 - 50);
		
		// LaneDetector(offsetX, offsetY, frameHight, frameWidth, topLeftSrcP,topRightSrcP, lowRightSrcP, lowLeftSrcP)
		laneDetector = LaneDetector(preProcess.getOffsetX(), preProcess.getOffsetY(), preProcess.getFrameHight(), preProcess.getFrameWidth(), cv::Point(85, 0), cv::Point(350, 0), cv::Point(preProcess.getROIWidth() -55,  preProcess.getROIHight()), cv::Point(35,  preProcess.getROIHight()));
		//original Roi
	//	laneDetector_future = LaneDetector(preProcess_future.getOffsetX(), preProcess_future.getOffsetY(), preProcess_future.getFrameHight(), preProcess_future.getFrameWidth(), cv::Point(110, 0), cv::Point(275, 0), cv::Point(preProcess_future.getROIWidth() -135,  preProcess_future.getROIHight()), cv::Point(60,  preProcess_future.getROIHight()));
		
		controller = Controller(laneDetector.getLaneInfo());
	}


	controller.updateController(laneDetector.getLaneInfo());
	preProcess.updateFrame(originalFrame);
	//preProcess_future.updateFrame(originalFrame);
	
	//pattern_check
	//+laneDetector.PATTERN_CHECK(preProcess.getNormalViewROIFrame());

	cv::Mat normalViewFrame = preProcess.getNormalViewROIFrame();	
//	cv::Mat normalViewFrame_future = preProcess_future.getNormalViewROIFrame();
	if(!isClassConstructed){
		patternCheck = PatternCheck(normalViewFrame);
	//	patternCheck_future = PatternCheck(normalViewFrame_future);
		isClassConstructed = true;
	}
	else {
		patternCheck.updatePatternCheckFrame(normalViewFrame);
	//	patternCheck_future.updatePatternCheckFrame(normalViewFrame_future);
	}
	patternCount++;
	if(patternCount <  100) patternCheck.checkPattern(false);
	else patternCheck.checkPattern(true);
//	patternCheck_future.checkPattern();
	//laneDetector.getLaneInfo().setcountStartLine(patternCheck.getCountStartLine());
	laneDetector.setStartLineCount(patternCheck.getCountStartLine());

	/*
	cv::Mat patternProcessedFrame = patternCheck.getPatternProcessedFrame();
	cv::Mat patternProcessedFrame_future = patternCheck_future.getPatternProcessedFrame();

	preProcess.preprocessElec(patternProcessedFrame);
	preProcess_future.preprocessElec(patternProcessedFrame_future);
	*/

	preProcess.preprocessElec(patternCheck.getRectPoints());
//	preProcess_future.preprocessElec(patternCheck_future.getRectPoints());

	laneDetector.setOriginalFrame(originalFrame); 
	//laneDetector_future.setOriginalFrame(originalFrame);
 
	cv::Mat preprocessedThresholdFrame = preProcess.getPreprocessedFrame_E();
	cv::Mat preprocessedGrayFrame = preProcess.getGrayFrame();
//	cv::Mat preprocessedThresholdFrame_future = preProcess_future.getPreprocessedFrame_E();
	//cv::Mat preprocessedGrayFrame_future = preProcess_future.getGrayFrame();
	
	// warping the temporal processed and thresholded image
	laneDetector.warpImage(preprocessedThresholdFrame, TEMPORAL_THRESHOLD_OPTION); 
//	laneDetector_future.warpImage(preprocessedThresholdFrame_future, TEMPORAL_THRESHOLD_OPTION); 
	//imshow("Warp Image - Thresholded", laneDetector.getIpImage_T());


	// warping the temporal processed image
	laneDetector.warpImage(preprocessedGrayFrame, TEMPORAL_OPTION); 
//	laneDetector_future.warpImage(preprocessedGrayFrame_future, TEMPORAL_OPTION);
	//imshow("Warp Image - Gray", laneDetector.getIpImage());


	laneDetector.setLaneMarker(); // search window point들을 만듦.
//	laneDetector_future.setLaneMarker();
//	float curveture =  abs(laneDetector.getHoughlineCluterSlope() - laneDetector_future.getHoughlineCluterSlope());
	// houghline left => laneDetector.cpp	
//	if(curveture > 16.5){
//		laneDetector.setLaneInfoFutureScarp(true);
//		isAlert = true;
//	} else{
//		laneDetector.setLaneInfoFutureScarp(false);
//		isAlert = false;
//	}

	// houghline right
//	if(curveture > 2.3){
//		laneDetector.setLaneInfoFutureScarp(true);
//		isAlert = true;
//	} else{
//		laneDetector.setLaneInfoFutureScarp(false);
//		isAlert = false;
//	}

 	laneDetector.startDetectLaneLogic();
	controller.updateLaneInfo(laneDetector.getLaneInfo(), car_angle, interval, speed_Min, speed_Max);
	
	//if (controller.getControlStatus() == 2) cntFrame++;
	//cout << " ~~~~~~~~~~~ Frame count " << cntFrame << endl;

	

	//////////////////////////// ROS - parameter modification code ////////////////////////////

	// Testing Frame
	// <<<<<<<<<<<< Result and originalFrame >>>>>>>>>>>>
	resultFrame = laneDetector.getResultFrame(); // Result Frame including original Frame	
	//resultFrame = laneDetector_future.getResultFrame();
	//resultFrame = preProcess.getNormalViewROIFrame();
	
	//<<<<<<<<<<<<<<<< test check pattern >>>>>>>>>>>>
//	cvtColor(preprocessedThresholdFrame, resultFrame, CV_GRAY2BGR);

	// <<<<<<<<<<<< Road area ROI Region >>>>>>>>>>>>
	// Coordinate test - Road roi
	///*
	//cv::Point upperLeft = cv::Point(150, (int)(frame.rows * 2.5 / 3) );
	//cv::Point lowerRight = cv::Point(frame.cols - 20, frame.rows - 10);
//	cv::circle(resultFrame, upperLeft, 5, Scalar(255,0,0),2, 8);
//	cv::circle(resultFrame, lowerRight, 5, Scalar(255,0,0),2, 8);	
	//cvtColor(preprocessedGrayFrame, resultFrame, CV_GRAY2BGR); // ROI Region frame - gray
	//*/
	//

	

	// <<<<<<<<<<<<<<<<<<<Pattern check ROI >>>>>>>>>>>>>>>>>>>
	//resultFrame = preProcess.getNormalViewROIFrame();
	//resultFrame = preProcess_future.getNormalViewROIFrame();
	//cvtColor(preprocessedThresholdFrame, resultFrame, CV_GRAY2BGR); // ROI Region frame - thresholded
	//cvtColor(patternCheck.getRosPatternFrame(), resultFrame, CV_GRAY2BGR);
	

	// <<<<<<<<<<<< Inverse perspective mapping parameter >>>>>>>>>>>>
	//cvtColor(laneDetector.getWarpCoordinateTestFrame(), resultFrame, CV_GRAY2BGR); // checking warp parameterpoints => laneDetector.h Constructor
	//cvtColor(laneDetector.getIpImage(), resultFrame, CV_GRAY2BGR); // confirming the inverse perspective frame
	
	//cvtColor(laneDetector_future.getWarpCoordinateTestFrame(), resultFrame, CV_GRAY2BGR); // checking warp parameterpoints => laneDetector.h Constructor
	//cvtColor(laneDetector_future.getIpImage(), resultFrame, CV_GRAY2BGR); // confirming the inverse perspective frame
	// laneDetector.cpp => warpImage() function circle => change to comment



	// <<<<<<<<<<<< left, right road roi, lane interval parameter, lane width >>>>>>>>>>>>
//	cvtColor(laneDetector.getIpImage(), resultFrame, CV_GRAY2BGR); // confirming the points => goto laneDetector.cpp - getIpImage()
//	cvtColor(laneDetector.getIPLeftImage(), resultFrame, CV_GRAY2BGR);
	//cvtColor(laneDetector.getIPRightImage(), resultFrame, CV_GRAY2BGR);	
	
	// <<<<<<<<<<< check houghline clsuter
	//cvtColor(laneDetector.getHoughClusterFrame(), resultFrame, CV_GRAY2BGR);
	//cvtColor(laneDetector_future.getHoughClusterFrame(), resultFrame, CV_GRAY2BGR);
	
	// ***** laneDetector.cpp => matchGaussianFilter => change convolution value(for sesitivity) 
	// ***** laneDetector.h => LEFT_ROI_X, MID_ROI_X, LEFT_MID_OFFSET, RIGHT_MID_OFFSET, RIGHT_ROI_X, LANE_SPACE


	
	//<<<<<<<<<<<< checking the lane marker Frame >>>>>>>>>>>>
//	resultFrame = laneDetector.getLaneMarkerFrameL();
//	resultFrame = laneDetector.getLaneMarkerFrameR();

	///////////////////////////////////////////////////////////////////////////////////////////////////
	///*
	rectangle(resultFrame, cv::Point(0,10), cv::Point(640, 120), Scalar(0,0,255), CV_FILLED);
	if(controller.getControlStatus() == 0){
		putText(resultFrame, format("Pid: %f,  %s", controller.getPidValue(), "Left"), Point(15,30), FONT_ITALIC, 0.7, Scalar(0,255,0),2, false);
	} else if(controller.getControlStatus() == 1){
		putText(resultFrame, format("Pid: %f,  %s", controller.getPidValue(), "Right"), Point(15,30), FONT_ITALIC, 0.7, Scalar(0,255,0),2, false);
	} else{
		putText(resultFrame, format("Pid: %f", controller.getPidValue()), Point(10,30), FONT_ITALIC, 0.7, Scalar(0,255,0),2, false);
	}
	
	putText(resultFrame, format("Car_Run_Speed: %f",controller.getCarRunSpeed()), Point(15,65), FONT_ITALIC, 0.7, Scalar(0,255,0),2, true);
	putText(resultFrame, format("Distance: %f",(controller.getCenterOfBottomLane()-320)), Point(15, 100), FONT_ITALIC, 0.7, Scalar(0,255,0),2, true);
	putText(resultFrame, format("Pattern Count: %d",(controller.getCountStartLine())), Point(335,65), FONT_ITALIC, 0.7, Scalar(0,255,0),2, true);
	putText(resultFrame, format("Straight Frame Count: %d", controller.getStraightLaneCount()), Point(335,100), FONT_ITALIC, 0.7, Scalar(0,255,0), 2,true);
	
//	if(isAlert){
//		putText(resultFrame, format("Alert!!!: %f",curveture), Point(335,30), FONT_ITALIC, 0.7, Scalar(255,0,0),2, false);
//	} else{
//		putText(resultFrame, format("Straight: %f",curveture), Point(335,30), FONT_ITALIC, 0.7, Scalar(0,255,0),2, false);
//	}
//*/
	circle(resultFrame, Point((int)controller.getCenterOfBottomLane(), resultFrame.rows - 10), 5, cv::Scalar(0, 255, 255), 4, 4);

	
	return resultFrame;
	

}







class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //image_transport::Publisher image_pub_2;
  
  ros::Publisher ack_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux/input/teleop", 1);
  ackermann_msgs::AckermannDriveStamped ack_msg;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    //image_pub_2 = it_.advertise("/image_converter/output_video2", 1);

  }

  ~ImageConverter()
  {
    //destroyWindow(OPENCV_WINDOW);
    //destroyWindow(OPENCV_GRAYIMAGE);
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    clock_t begin = clock();	
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::mono8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    	
    cv::Mat img = cv_ptr->image;
    cv_ptr->image = imageProcess(img);
    


	
	///////////////////////////////

	//car_angle = car_angle + 0.1; // 1: left & -1: right
	//car_angle_speed = car_angle_speed + 1.0;

//	car_angle += interval;
// 	if( car_angle >= Max ) interval *= -1;
//	else if( car_angle <= Min ) interval *= -1; // 1: left & -1: right
// 	car_angle = Max;

	
	///////////////////////////////
	

    // Update GUI Window
    
    
//    ack_msg.drive.steering_angle = car_angle * car_angle_speed + 0.03;
    ack_msg.drive.steering_angle = controller.getCarAngle() * car_angle_speed; 

    if(controller.getCarRunSpeed() == 0){
	return;
    }
    ack_msg.drive.speed = car_direction * controller.getCarRunSpeed();
	

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    ack_pub.publish(ack_msg);
    //image_pub_.publish(cv_ptr_gray->toImageMsg());

    clock_t end = clock();
    double time = double(end - begin) / CLOCKS_PER_SEC;
 //   cout << "time: " << time << endl;    
  }
};



int main(int argc, char** argv) {
	ros::init(argc, argv, "hiscar_v1");
	cout << "//  ~~~~~~~~~~~ hello ~~~~~~~~~~~  //" << endl;	
	ImageConverter ic;
        ros::spin();
        return 0;	
}



