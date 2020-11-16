
#pragma once

#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "opencv/cv.hpp"

/*
======================================= B-Snake �˰��� ������ ���� �ʿ��� ���� - �ּ�ó���� =======================================================
// sub ROI�� ���� ����
#define ROI_LEVEL1 0
#define ROI_LEVEL2 1
#define ROI_LEVEL3 2
#define ROI_LEVEL4 3
//#define ROI_LEVEL_F 4
#define ROI_NUM 4
// ������ sub ROI ������ ���� ��ǥ: ���ʿ��� �Ʒ������� ROI1 ~4 => B-snake �˰����� ����ϱ� ���ؼ� �ʿ��� �����̴�(�� ���α׷��� �������� ������ ����).
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

#define STACK_FRAME_NUM 2 // stack�� ������ ���� �������� ���� => Temporal blurring���� ����Ѵ�

#ifndef _PRE_PROCESS__
#define _PRE_PROCESS__
using namespace std;

class PreProcess {

private:
	cv::Mat originalFrame; // ���� ���� frame
	cv::Mat processFrame; // ROI gray scale Frame - grayscale�� ��ȯ�� ���� ������ ���ؼ��� ��ó���� ������ ���귮�� ���� �� �ִ�.
	cv::Mat otsuProcessedFrame; // preprocessed frame
	cv::Mat temporalProcessedThresholdedFrame; // binary frame
	cv::Mat temporalProcessedFrame; // gray scale frame
	cv::Mat warpFrame; // inverse perspecive mapping�� �� frame
	cv::Mat preprocessedFrame_E;
	cv::Mat normalViewROIFrame;
	//cv::Mat roiFrame[ROI_NUM]; // B-Snake
	vector<cv::Mat> frameStack; // ���� �����ӵ��� �����ϴ� ���� - temporalBlur���� ���
	//cv::Mat leftOriginalFrame; // ����(��Ȯ�� ��)
	//cv::Mat rightOriginalFrame; // ����(��Ȯ�� ��)
	int frameStackCount; // frameStack�� ����ִ� frame�� ����
	int frameNum; // ���� ������ ��ȣ
	int roiStartY; // ���� �����ӿ��� roi Frame�� �����ϴ� y��ǥ(����) => roi ó���� ���� ���󿡼��� ��ġ�� ã�� ���� offset ��
	int hight; // roi ����
	int width; 
	int frameHight; // originalFrame Hight
	int frameWidth;

	//ROI�� �����ϱ� ���� Point�� ���� ����
	cv::Point upperLeft;
	cv::Point  lowerRight;
	cv::Point lowerRigntOfLeftFrame;
	cv::Point upperLeftOfRightFrame;

	bool isTemporalFrameReady = false; // temporal blur�� �� �� �ִ����� ����(stack�� ���� �����ӵ鿡 ���� ������ ���� �Ǿ��°��� ����)


public:
	/* Constructors */
	PreProcess() {}

	PreProcess(cv::Mat& frame, int upperLeftX, int upperLeftY, int lowerRightX, int lowerRightY) {
		/* subRoi frame �ʱ�ȭ */
		originalFrame = frame;
		frameHight = frame.rows;
		frameWidth = frame.cols;

		// ó���� ROI ����
		upperLeft = cv::Point(upperLeftX, upperLeftY);
		lowerRight = cv::Point(lowerRightX, lowerRightY);

		hight = lowerRight.y -upperLeft.y;
		width = lowerRight.x - upperLeft.x;
		
		/* frameStack�� count������ ��ü ������ ������ ���� ���� �ʱ�ȭ */
		frameStackCount = 0;
		frameNum = 1;
		
		/* ��ü ROI(Region Of Interest) , ������ ROI, ���� ROI ���� */
		processFrame = frame(cv::Rect(upperLeft, lowerRight));
		normalViewROIFrame = processFrame.clone();
		//cout << "=======> column ����: " << processFrame.cols << endl;
		cv::cvtColor(processFrame, processFrame, CV_BGR2GRAY);
		//leftOriginalFrame = frame(cv::Rect(upperLeft, lowerRigntOfLeftFrame));
		//rightOriginalFrame = frame(cv::Rect(upperLeftOfRightFrame, lowerRight));
		/* 
		===================== ��ü ROI�� ���� ���� ROI - B-snake �˰��򿡼� ��� ====================== 
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
	void preprocess(void); // grayscale�� ��ȯ & subRoi�� frame ����
	int getOtsuThreshold(cv::Mat &srcFrame, cv::Mat &grayFrame);
	int getBasicGlobalThresholdValue(cv::Mat &srcFrame); // basic method�� ������ (int type)threshold ���� �����ϴ� �Լ�
	void updateFrame(const cv::Mat &frame); // video frame update
	cv::Mat temporalBlur(cv::Mat frame); // temporal Blurring 
	bool addToFrameStack(cv::Mat frame); // add to frame stack(���� �����ӵ� ����)
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
