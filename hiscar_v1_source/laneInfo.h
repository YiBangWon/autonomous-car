
#pragma once
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "opencv/cv.hpp"

#ifndef _LANE_INFO__
#define _LANE_INFO__
#define CURVED_LEFT 0
#define CURVED_RIGHT 1

class LaneInfo {
private:
	cv::Point interceptionPoint; // interception point of left and right lane
	bool isCurvedRoad; // is the road is curved road
	int isCurvedDirection; // 0: left, 1: right, -1: notDefined : use previous value
	bool isOnlyOneLaneDetectedL; // road is not scarp but only one left lane is detected due to road condition
	bool isOnlyOneLaneDetectedR; // road is not scarp but only one right lane is detected due to road condition
	bool isFutureScarp = false; // if road is scarp in the future
	int cameraCenterX; // center of car - x axis
	int cameraCenterY; // center of car - y axis
	int frameHight; // frame hight: bottom of the frame Y coordinate
	int frameWidth;
	cv::Point leftBottomPoint; // intersection point of left lane and bottom of the frame
	cv::Point rightBottomPoint; // intersection point of right lane and bottom of the frame
	cv::Point scarpBottomPoint; //intersection point of scarp lane and bottom of the frame
	cv::Point leftBottomPointOneLane; // intersection point of left lane and bottom of the frame(when only one non-scarp left lane is detected)
	cv::Point rightBottomPointOneLane; // intersection point of right lane and bottom of the frame(when only one non-scarp right lane is detected)



	//pattern_check
	int countStartLine = 0;
	//int isStartLine;
	//int frameNum;
	//int countSquare;
	//int frame_startLine;
	//

public:
	//setter
	void setInterceptionPoint(cv::Point);
	void setIsCurvedRoad(bool truth, int option = -1);
	void setCameraCenterX(int x);
	void setCameraCenterY(int y);
	void setFrameHight(int h);
	void setFrameWidth(int w);
	void setleftBottomPoint(cv::Point);
	void setRightBottomPoint(cv::Point);
	void setScarpBottomPoint(cv::Point);
	void setIsOnlyOneLaneDetectedR(bool value);
	void setIsOnlyOneLaneDetectedL(bool, bool);
	void setLeftBottomPointOneLane(cv::Point);
	void setRightBottomPointOneLane(cv::Point);
	void setIsFutureScarp(bool);
	void setIsCurvedDirection(int);

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
	bool getIsOnlyOneLaneDetectedR(void);
	bool getIsOnlyOneLaneDetectedL(void);
	cv::Point getLeftBottomPointOneLane(void);
	cv::Point getRightBottomPointOneLane(void);
	int getIsCurvedDirection(void);
	bool getIsFutureScarp(void);

	//pattern_check
	void setcountStartLine(int count_StartLine);
	int getcountStartLine(void);
	/*
	void setisStartLine(int is_StartLine);
	void setcountStartLine(int count_StartLine);
	void setcountSquare(int count_Square);
	void setframe_startLine(int frame_start_Line);
	void setframeNum(int frame_Num);
	*/	
	//
};
#endif
