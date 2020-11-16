#pragma once
#include "laneInfo.h"
#ifndef _CONTROLLER_H__
#define _CONTROLLER_H__

/*
class LaneInfo {
	cv::Point interceptionPoint;
	bool isCurvedRoad;
	int cameraCenterX; 
	int cameraCenterY;
	int frameHight;
	int frameWidth;
	cv::Point leftBottomPoint; // intersection point of left lane and bottom of the frame
	cv::Point rightBottomPoint; // intersection point of right lane and bottom of the frame
	cv::Point scarpBottomPoint; //intersection point of scarp lane and bottom of the frame

public:
	//setter
	void setInterceptionPoint(cv::Point);
	void setIsCurvedRoad(bool truth);
	void setCameraCenterX(int x);
	void setCameraCenterY(int y);
	void setFrameHight(int h);
	void setFrameWidth(int w);	
	void setleftBottomPoint(cv::Point);
	void setRightBottomPoint(cv::Point);
	void setScarpBottomPoint(cv::Point);
	

	// getter
	cv::Point getInterceptionPoint(void);
	bool getIsCurvedRoad(void);
	int getCameraCenterX(void);
	int getCameraCenterY(void);
	int getFrameHight(void);
	int getFrameWidth(void);
	cv::Point getLeftBottomPoint(void);
	cv::Point getRightBottomPoint(void);
	cv::Point getScarpBottomPoint(void);
};
*/

#define LANE_SPACE_PERSPECTIVE 350 // lanespace when normal view
#define GO_SPEED 1.4
#define CURVE_SPEED 0.9

class Controller {
private:
	LaneInfo laneInfo;
	// erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) + speed_to_erpm_offset
	// servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset

	float speed_Max = 1.3;
	float speed_Min = 0.45;
	//float angle_Max = 0.29;
	float angle_Max = 0.34;
	float angle_Min = -0.34;
	float interval = 0.01;
	float centerOfBottomLane = 0.0;
	int nonScarpFrame = 10;
	int controlStatus = -1; // 0: Left, 1: right
//	int countStartLine = 0;

	/*
	float car_direction = 1; // 1: front & -1: back 
	float car_run_speed = speed_Min;
	//float car_angle = 0; // 1: left & -1: right
	//float car_angle_speed = 2.0;
	//float car_angle = -0.34; // 1: left & -1: right
	float car_angle = 0.03; // 1: left & -1: right
	float car_angle_speed = 1.0;
	bool isClassConstructed = false;
	*/
        float error = 0;
	float error_sum = 0;
	float error_old = 0;
	float p1 = 0.0;
	float i1 = 0.0;
	float d1 = 0.0;
	float p[3] = {0.0035, 0.000005, 0.005};  // -- original
	//float p[3] = {0.0035, 0.000005, 0.01}; // -- Good
	//float p[3] = {0.0035, 0.000005*0.75, 0.015};
	
	float dp[3] = {p[0]/10, p[1]/10, p[2]/10};
	float pid = 0.0f; //!
	float car_run_speed = 0.0f; //!
	
	// for checking lane Departure
	bool isSafe = true;
	long cntFrame = 0;
	long cntCurveFrame = 0;
	//pattern_check
	int pattern_frameNum = 0;
	int countStartLine = 0;

	// Check Drive Start
	bool driveStart = false;
	

public:
	Controller() {}
	Controller(LaneInfo laneInfo) :laneInfo(laneInfo) {
	}

	void updateLaneInfo(LaneInfo info, float car_angle, float interval, float speed_Min, float speed_Max);
	float cal_error(int x, int center_x);
	void twiddle(int x, int center_x);
	float calPid(int target, int center);
	float getCarAngle(void);
	//float getCarSpeed(void);
	float getPidValue(void);
	float getCarRunSpeed(void);
	float getCenterOfBottomLane(void);
	int getControlStatus(void);
	//pattern_check
	int getCountStartLine(void);
	void updateController(LaneInfo);
	int getStraightLaneCount(void);
	void setCountStartLine(int);
	//int getCountStartLine(void);
};

#endif 
