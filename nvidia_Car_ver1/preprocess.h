
#pragma once

#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "opencv/cv.hpp"

/*
======================================= B-Snake 알고리즘 적용을 위해 필요한 정보 - 주석처리함 =======================================================
// sub ROI에 대한 정보
#define ROI_LEVEL1 0
#define ROI_LEVEL2 1
#define ROI_LEVEL3 2
#define ROI_LEVEL4 3
//#define ROI_LEVEL_F 4
#define ROI_NUM 4
// 영상의 sub ROI 설정에 관한 좌표: 위쪽에서 아래쪽으로 ROI1 ~4 => B-snake 알고리즘을 사용하기 위해서 필요한 변수이다(현 프로그램의 논리에서는 사용되지 않음).
#define UPPER_LEFT_1  cv::Point(upperLeft.x, upperLeft.y - roiStartY)
#define LOWER_RIGHT_1  cv::Point(lowerRight.x, upperLeft.y + height * 1/6 - roiStartY) // blue
#define UPPER_LEFT_2  cv::Point(upperLeft.x, upperLeft.y + height * 1/6 - roiStartY)
#define LOWER_RIGHT_2  cv::Point(lowerRight.x, upperLeft.y + height * 2/6 - roiStartY) // green
#define UPPER_LEFT_3 cv::Point(upperLeft.x,  upperLeft.y + height * 2/6 - roiStartY)
#define LOWER_RIGHT_3  cv::Point(lowerRight.x, upperLeft.y + height * 3/6 - roiStartY) // red
#define UPPER_LEFT_4 cv::Point(upperLeft.x, upperLeft.y + height * 3/6 - roiStartY)
#define LOWER_RIGHT_4  cv::Point(lowerRight.x, lowerRight.y - roiStartY) // red
==================================================================================================================================================
*///nvidia@tegra-ubuntu:~

#define STACK_FRAME_NUM 2 // stack에 저장할 이전 프레임의 갯수 => Temporal blurring에서 사용한다

#ifndef _PRE_PROCESS__
#define _PRE_PROCESS__
using namespace std;

class PreProcess {

private:
	cv::Mat originalFrame; // 원본 영상 frame
	cv::Mat processFrame; // ROI gray scale Frame - grayscale로 변환한 도로 영역에 대해서만 전처리를 함으로 연산량을 줄일 수 있다.
	cv::Mat otsuProcessedFrame; // preprocessed frame
	cv::Mat temporalProcessedThresholdedFrame; // binary frame
	cv::Mat temporalProcessedFrame; // gray scale frame
	cv::Mat warpFrame; // inverse perspecive mapping을 한 frame
	cv::Mat preprocessedFrame_E;
	cv::Mat normalViewROIFrame;
	//cv::Mat roiFrame[ROI_NUM]; // B-Snake
	vector<cv::Mat> frameStack; // 이전 프레임들을 저장하는 변수 - temporalBlur에서 사용
	//cv::Mat leftOriginalFrame; // 좌측(정확히 반)
	//cv::Mat rightOriginalFrame; // 우측(정확히 반)
	int frameStackCount; // frameStack에 들어있는 frame의 갯수
	int frameNum; // 현재 프레임 번호
	int roiStartY; // 원본 프레임에서 roi Frame이 시작하는 y좌표(높이) => roi 처리후 실제 영상에서의 위치를 찾기 위한 offset 값
	int hight; // roi 높이
	int width; 
	int frameHight; // originalFrame Hight
	int frameWidth;

	//ROI를 설정하기 위한 Point에 대한 정보
	cv::Point upperLeft;
	cv::Point  lowerRight;
	cv::Point lowerRigntOfLeftFrame;
	cv::Point upperLeftOfRightFrame;

	bool isTemporalFrameReady = false; // temporal blur를 할 수 있는지의 여부(stack에 이전 프레임들에 대한 정보가 축적 되었는가의 여부)


public:
	/* Constructors */
	PreProcess() {}

	PreProcess(cv::Mat& frame, int upperLeftX, int upperLeftY, int lowerRightX, int lowerRightY) {
		/* subRoi frame 초기화 */
		originalFrame = frame;
		frameHight = frame.rows;
		frameWidth = frame.cols;

		// 처리할 ROI 설정
		upperLeft = cv::Point(upperLeftX, upperLeftY);
		lowerRight = cv::Point(lowerRightX, lowerRightY);

		hight = lowerRight.y -upperLeft.y;
		width = lowerRight.x - upperLeft.x;
		
		/* frameStack의 count정보와 전체 프레임 갯수에 대한 정보 초기화 */
		frameStackCount = 0;
		frameNum = 1;
		
		/* 전체 ROI(Region Of Interest) , 오른쪽 ROI, 왼쪽 ROI 생성 */
		processFrame = frame(cv::Rect(upperLeft, lowerRight));
		normalViewROIFrame = processFrame.clone();
		//cout << "=======> column 길이: " << processFrame.cols << endl;
		cv::cvtColor(processFrame, processFrame, CV_BGR2GRAY);
		//leftOriginalFrame = frame(cv::Rect(upperLeft, lowerRigntOfLeftFrame));
		//rightOriginalFrame = frame(cv::Rect(upperLeftOfRightFrame, lowerRight));
		/* 
		===================== 전체 ROI에 대한 다중 ROI - B-snake 알고리즘에서 사용 ====================== 
		//height = abs(lowerRight.y - upperLeft.y);
		//roiFrame[ROI_LEVEL1] = processFrame(cv::Rect(UPPER_LEFT_1, LOWER_RIGHT_1));
		//roiFrame[ROI_LEVEL2] = processFrame(cv::Rect(UPPER_LEFT_2, LOWER_RIGHT_2));
		//roiFrame[ROI_LEVEL3] = processFrame(cv::Rect(UPPER_LEFT_3, LOWER_RIGHT_3));
		//roiFrame[ROI_LEVEL4] = processFrame(cv::Rect(UPPER_LEFT_4, LOWER_RIGHT_4));
		=============================================================================================
		*/
	}

	/* method declaration */
	void preprocessElec(vector<cv::Point>); // preproecess for 1/10 electric car 
	void preprocess(void); // grayscale로 변환 & subRoi로 frame 분할
	int getOtsuThreshold(cv::Mat &srcFrame, cv::Mat &grayFrame);
	int getBasicGlobalThresholdValue(cv::Mat &srcFrame); // basic method로 영상의 (int type)threshold 값을 리턴하는 함수
	void updateFrame(const cv::Mat &frame); // video frame update
	cv::Mat temporalBlur(cv::Mat frame); // temporal Blurring 
	bool addToFrameStack(cv::Mat frame); // add to frame stack(이전 프레임들 저장)
	cv::Mat getTemporalThresholdProcessedFrame(void);
	cv::Mat getOtsuProcessedFrame(void);
	bool isTemporalProcessComplete(void);
	cv::Mat getTemporalProcessedFrame(void);
	cv::Mat getPreprocessedFrame_E(void);
	cv::Mat getGrayFrame(void);
	int getOffsetX(void);
	int getOffsetY(void);
	int getROIHight(void);
	int getROIWidth(void);
	int getFrameHight(void);
	int getFrameWidth(void);
	cv::Mat getNormalViewROIFrame(void);
};

#endif
