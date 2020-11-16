#pragma once
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "opencv/cv.hpp"
#include "laneInfo.h"

#ifndef _PATTERN_CHECK__
#define _PATTERN_CHECK__

using namespace std;
using namespace cv;
class PatternCheck{
private:
	cv::Mat frame;
	cv::Mat rosPatternFrame;
	cv::Mat patternProcessedFrame;
	cv::Mat adaptiveFrame;
	//pattern_check
	bool isStartLine = true;
	int frame_startLine = 0;
	int frameNum = 0;
	int countStartLine = 0;
	vector<cv::Point> rectPoints;

public:
	PatternCheck(){}
	PatternCheck(cv::Mat frameInfo){
		frame = frameInfo;
	}

	void updatePatternCheckFrame(cv::Mat);
	void checkPattern(bool);
	cv::Mat getRosPatternFrame(void);
	cv::Mat getPatternProcessedFrame(void);
	int getCountStartLine(void);
	vector<cv::Point> getRectPoints(void);
};


#endif
