
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
#include <ctime>

/*
���󸶴� �Ķ����
=====================================================================================================
<��ӵ��� ����>
char videoName[100] = "C:\\Users\\user\\Videos\\smartCarVideo\\test\\test1.mpg"; // ���� ���� �̸�!!!!

****************** ROI ���� ���� - preprocess.h *******************
upperLeft = cv::Point(0, roiStartY = (int)(frame.rows * 3.2 / 5));
lowerRight = cv::Point(frame.cols, frame.rows * 9 / 10);

****************** Warp Image Parameter - laneDetector.h => LaneDetector() ******************
topLeftSrcP = cv::Point(630, 10);
topRightSrcP = cv::Point(750, 10);
lowLeftSrcP = cv::Point(430, 180);
lowRightSrcP = cv::Point(970, 180);

******************** preprocess.h => stack frame �� *********************
#define STACK_FRAME_NUM 25

********************* preprocess.cpp => temporalBlur() *****************
int frameStepSize = 3;
int averageNum = 6;

********************* laneDetector.h *************************
#define LEFT_ROI_X 590
#define MID_ROI_X 690
#define RIGHT_ROI_X 790

********************* laneDetector.cpp setLaneMarker() *************************
cv::HoughLines(ipLeftImage_T, leftLanes, 1, CV_PI / 180 * 1.5, threshold, 0, 0, -CV_PI / 5, CV_PI / 5);
cv::HoughLines(ipRightImage_T, rightLanes, 1, CV_PI / 180 * 1.5, threshold, 0, 0, -CV_PI / 5, CV_PI / 5);





=====================================================================================================
<1/10 ������ 412ȣ Ʈ�� ����>
****************** ROI ���� ���� - preprocess.h ******************
upperLeft = cv::Point(0, roiStartY = (int)(frame.rows * 1.2 / 3));
lowerRight = cv::Point(frame.cols, frame.rows * 2 / 3);

****************** Warp Image Parameter - laneDetector.h => LaneDetector() ******************
topLeftSrcP = cv::Point(515, 10);
topRightSrcP = cv::Point(765, 10);
lowRightSrcP = cv::Point(950, 180);
lowLeftSrcP = cv::Point(330, 180);


******************** preprocess.h => stack frame �� *********************
#define STACK_FRAME_NUM 3

********************* preprocess.cpp => temporalBlur() *****************
int frameStepSize = 1;
int averageNum = 3;

********************* laneDetector.h *************************
#define LEFT_ROI_X 200 
#define MID_ROI_X 425
#define RIGHT_ROI_X 650 

********************* laneDetector.cpp setLaneMarker() *************************
cv::HoughLines(ipLeftCannyImage, leftLanes, 1, CV_PI / 180 * 1.5, threshold, 0, 0, -CV_PI / 3, CV_PI / 3);
cv::HoughLines(ipRightCannyImage, rightLanes, 1, CV_PI / 180 * 1.5, threshold, 0, 0, -CV_PI / 3, CV_PI / 3);

=====================================================================================================
*/


// global variables
int mouseWarpOption = 0; // warp point�� ���� �� �ʿ��� ���� ����
LaneDetector laneDetector; // LaneDetector Ŭ���� ���� => ���� �ν��� ���� �ʿ��� �ʵ庯���� �޼ҵ带 ������ ����
PreProcess preProcess; // PreProcess Ŭ���� ���� = > ��ó�� ������ ���� �ʿ��� �ʵ庯���� �޼ҵ带 ������ ����

// function declaration
//void videoEvent(int controlKey, int &frameRate, int originalFrameRate, cv::VideoCapture video); // ���� ����, ��, �ڷ� ���� ���� ���
//void setWarpPointsCallBackFunc(int event, int x, int y, int flags, void* userdata); // �Ϲ� �信�� warping�� ����Ʈ ����
//void showROICoordCallBackFunc(int event, int x, int y, int flags, void* userdata); // ž�信�� left right roi point�� ���� ������ ��� ����


// erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) + speed_to_erpm_offset
// servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset


float car_direction = 1; // 1: front & -1: back 
float car_run_speed = 0.5;
//float car_angle = 0; // 1: left & -1: right
//float car_angle_speed = 2.0;
float Max = 0.3;
float Min = -0.35;
float car_angle = 0; // 1: left & -1: right
float car_angle_speed = 1.0;
float interval = 0.01;


using namespace cv;
using namespace std;



cv::Mat imageProcess(cv::Mat frame){
	cv::Mat resultFrame;


	// Variables 
	int controlKey; // ���� ��Ʈ��Ű(�̺�Ʈ) 
	int frameRate; // ���� ��Ʈ�ѿ� ���� ���ϴ� FPS
	char videoName[100] = "/test.mp4"; // ���� ���� �̸�!!!!
	cv::Mat originalFrame = frame; // ���󿡼� �޴� ���� ������
	//cv::VideoCapture video(ros::package::getPath("ros_opencv_example") + videoName); // object for capturing video
	//frameRate = video.get(cv::CAP_PROP_FPS); //������ fps(frame per second)
	frameRate = 30;	
	static int originalFrameRate = frameRate;// ���� ������ FPS
	//int frameNum = 0;


	// ���� �о���� - �� ������ ���� ó�� 
	//if (video.read(originalFrame)) {
		/* 
		������ ������ PrePrcesst ������ lanedDetector ������ �ʱ�ȭ ��Ŵ => ������ ���� ������ ���ο� object�� �ݺ� �������� �ʰ� 
		method�� ���� �ʵ庯���� update�Ǳ⸸ �ϹǷ� ȿ�����̴�.
		 */
		preProcess = PreProcess(originalFrame); 
		//preProcess.preprocess(); => ��ӵ��� ���󿡼� ���
		preProcess.preprocessElec();
		laneDetector = LaneDetector(preProcess.getOffsetX(), preProcess.getOffsetY(), preProcess.getROIHight());
	//}
	/*
	else {
		cout << "Fail to read file!!! Check the file Description";
		return 0;
	}
	*/

	// processing the video file 
	//while (true) {
		// ������ ������ ���� ���
		/*		
		if (!video.read(originalFrame)) {
			cout << "End of file";
			return 0;
		}
		*/
		//imshow("original Frame", originalFrame);
		preProcess.updateFrame(originalFrame);
		
		//preProcess.preprocess(); //preprocess Frame
		preProcess.preprocessElec();
		laneDetector.setOriginalFrame(originalFrame); 

		cv::Mat preprocessedThresholdFrame = preProcess.getPreprocessedFrame_E();
		cv::Mat preprocessedGrayFrame = preProcess.getGrayFrame();
		//imshow("test!!!!", preprocessedThresholdFrame);
		//cvNamedWindow("Click 4 points from left top in clockwise order, to warp the image", CV_WINDOW_AUTOSIZE);
		//imshow("Click 4 points from left top in clockwise order, to warp the image", preprocessedGrayFrame);
	//	cvSetMouseCallback("Click 4 points from left top in clockwise order, to warp the image", setWarpPointsCallBackFunc, NULL);
		mouseWarpOption = 0;

		laneDetector.warpImage(preprocessedThresholdFrame, TEMPORAL_THRESHOLD_OPTION); // warping the temporal processed and thresholded image
		//imshow("Warp Image - Thresholded", laneDetector.getIpImage_T());

		laneDetector.warpImage(preprocessedGrayFrame, TEMPORAL_OPTION); // warping the temporal processed image
		//imshow("Warp Image - Gray", laneDetector.getIpImage());

		laneDetector.setLaneMarker(); // search window point���� ����.

		//cvNamedWindow("Show coordinate for left and right ROI", CV_WINDOW_AUTOSIZE);
		//imshow("Show coordinate for left and right ROI", laneDetector.getIpImage());
		//cvSetMouseCallback("Show coordinate for left and right ROI", showROICoordCallBackFunc, NULL);


		//////////////////////////////// Otsu & warping image ////////////////////////////
		//cv::Mat preprocessedOtsuFrame = preProcess.getOtsuProcessedFrame();
		//cvNamedWindow("Pre-processed Frame", CV_WINDOW_AUTOSIZE);
		//imshow("Pre-processed Frame", preprocessedOtsuFrame);
		//cvSetMouseCallback("Pre-processed Frame", callBackFunc, NULL);
		//laneDetector.warpImage(preprocessedOtsuFrame, TEMPORAL_THRESHOLD_OPTION);
		//imshow("Warp Image", laneDetector.getIpImage());
		//=============================================== Temproal blurring�� �̿��� ��� =======================================================
		//if (preProcess.isTemporalProcessComplete()) {
		//	cv::Mat preprocessedTemporalThresholdFrame = preProcess.getTemporalThresholdProcessedFrame();
		//	cv::Mat preprocessedTemporalFrame = preProcess.getTemporalProcessedFrame();
		//	imshow("test!!!!", preprocessedTemporalFrame);
		//	//cvNamedWindow("Pre - processed Frame - click the points!!!", CV_WINDOW_AUTOSIZE);
		//	//imshow("Pre-processed Frame - click the points!!!", preprocessedTemporalThresholdFrame);
		//	//cvSetMouseCallback("Pre-processed Frame - click the points!!!", callBackFunc, NULL);
		//	laneDetector.warpImage(preprocessedTemporalThresholdFrame, TEMPORAL_THRESHOLD_OPTION); // warping the temporal processed and thresholded image
		//	imshow("Thresholded Warp Image", laneDetector.getIpImage_T());
		//	laneDetector.warpImage(preprocessedTemporalFrame, TEMPORAL_OPTION); // warping the temporal processed image
		//	imshow("Warp Image", laneDetector.getIpImage());
		//	laneDetector.setLaneMarker(); // search window point���� ����.
		//	cvNamedWindow("Show coordinate for left and right ROI", CV_WINDOW_AUTOSIZE);
		//	imshow("Show coordinate for left and right ROI", laneDetector.getIpImage());
		//	cvSetMouseCallback("Show coordinate for left and right ROI", callBackFunc2, NULL);
		//}
		//=====================================================================================================================================
		// ================================================= OTSU method�� �̿��� ��� ===========================================================
		//cv::Mat otsuThresholdedFrame = preProcess.getOtsuProcessedFrame();
		//laneDetector.warpImage(otsuThresholdedFrame, TEMPORAL_THRESHOLD_OPTION); // warping the temporal processed and thresholded image
		//imshow("Thresholded Warp Image2", laneDetector.getIpImage_T());
		//laneDetector.warpImage(roiGrayFrame, TEMPORAL_OPTION); // warping the temporal processed image
		//imshow("Warp Image2", laneDetector.getIpImage());
		//laneDetector.setLaneMarker(); // search window point���� ����.
		// ======================================================================================================================================
		//controlKey = cv::waitKey(150);
		resultFrame = laneDetector.getResultFrame();
			cv::Point upperLeft = cv::Point(150, (int)(resultFrame.rows * 2.3 / 3));
			cv::Point lowerRight = cv::Point(resultFrame.cols-100, resultFrame.rows -10);
			cv::circle(resultFrame, upperLeft, 3, Scalar(255,0,0),2, 8);
			cv::circle(resultFrame, lowerRight, 3, Scalar(255,0,0),2, 8);
			cv::circle(resultFrame, cv::Point(98, lowerRight.y -10), 3, Scalar(255,10,255),2, 8);
			cv::circle(resultFrame, cv::Point(190, lowerRight.y -10), 3, Scalar(255,10,255),2, 8);
			cv::circle(resultFrame, cv::Point(280, lowerRight.y -10), 3, Scalar(255,10,255),2, 8);
		controlKey = cv::waitKey(frameRate);
		//videoEvent(controlKey, frameRate, originalFrameRate, video);
		//frameNum++;
		return resultFrame;
	//}

}





class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  ros::Publisher ack_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux/input/teleop", 1);
  ackermann_msgs::AckermannDriveStamped ack_msg;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
    //ros::Publisher ack_pub = nh_.advertise("ackermann_cmd_mux/input/teleop", &AckermannDriveStamped, 1);

    //namedWindow(OPENCV_WINDOW);
    //namedWindow(OPENCV_GRAYIMAGE);
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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      //cv_ptr_gray = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::mono8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    	
    cv::Mat img = cv_ptr->image;
   // cv_ptr->image.copyTo(cv_ptr_gray->image);
    
    cv_ptr->image = imageProcess(img);
    clock_t end = clock();
    double time = double(end - begin) / CLOCKS_PER_SEC;
    cout << "time: " << time << endl;
    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //circle(img, Point(200, 200), 100, CV_RGB(255,0,0), 3);
      //cvtColor(cv_ptr_gray->image, cv_ptr_gray->image, CV_BGR2GRAY);
	
	///////////////////////////////

	//car_angle = car_angle + 0.1; // 1: left & -1: right
	//car_angle_speed = car_angle_speed + 1.0;

	car_angle += interval;
 	if( car_angle >= Max ) interval *= -1;
	else if( car_angle <= Min ) interval *= -1; // 1: left & -1: right
 	

	
	///////////////////////////////
	

    // Update GUI Window
    
    //imshow(OPENCV_GRAYIMAGE, cv_ptr_gray->image);
    cv::waitKey(30);
    
    //ack_msg = AckermannDriveStamped();
    ack_msg.drive.steering_angle = car_angle * car_angle_speed;
    ack_msg.drive.speed = car_direction * car_run_speed;


    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    ack_pub.publish(ack_msg);
    //image_pub_.publish(cv_ptr_gray->toImageMsg());    
  }
};





int main(int argc, char** argv) {
	ros::init(argc, argv, "ros_opencv_example");
	ImageConverter ic;
        ros::spin();
        return 0;
}


/*
void videoEvent(int controlKey, int &frameRate, int originalFrameRate, cv::VideoCapture video) {
	if (controlKey == 32) {
		if (frameRate != 0)
			frameRate = 0;
		else
			frameRate = originalFrameRate;
	}
	else if (controlKey == ']') {
		video.set(CV_CAP_PROP_POS_FRAMES, video.get(cv::CAP_PROP_POS_FRAMES) + 90);
	}
	else if (controlKey == '[') {
		video.set(CV_CAP_PROP_POS_FRAMES, video.get(cv::CAP_PROP_POS_FRAMES) - 90);
	}
	else if (controlKey == 27) {

	}

}

void  setWarpPointsCallBackFunc(int event, int x, int y, int flags, void* userdata) {
	// ���콺 ���� Ŭ�� => �����ϰ��� �ϴ� ������ 4������ ����(left Top ���� �ð� �������� ����)
	if (event == CV_EVENT_LBUTTONDBLCLK) {
		cv::Point warpPoint = cv::Point(x, y);
		laneDetector.setWarpSrcPoint(mouseWarpOption, warpPoint);
		mouseWarpOption = (mouseWarpOption + 1) % 4;
		cout << "next: " << mouseWarpOption << endl;
	}
}

void showROICoordCallBackFunc(int event, int x, int y, int flags, void* userdata) {
	if (event == CV_EVENT_LBUTTONDOWN) {
		cout << "Coordinate: (" << x << ", " << y  << endl;
	}
}*/
