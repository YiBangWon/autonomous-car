

#pragma once


#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "opencv/cv.hpp"
#include "laneInfo.h"

#define _USE_MATH_DEFINES
#include "math.h"
#include <cmath>

#ifndef _LANE_DETECTOR__
#define _LANE_DETECTOR__

// warp image
#define TEMPORAL_OPTION 0 
#define TEMPORAL_THRESHOLD_OPTION 1
// #define SEARCH_WINDOW_INTERVAL 5
#define SEARCH_WINDOW_INTERVAL 2
#define LEFT_OPTION 0
#define RIGHT_OPTION 1
#define TOTAL_OPTION 2
#define SPLIT_ROI_OPTION 0
#define NON_SPLIT_ROI_OPTION 1
#define SPLIT_LEFT_ROI_OPTION 2
#define SPLIT_RIGHT_ROI_OPTION 3
///* clock wise parameter 
//#define LEFT_ROI_X 40 // 250` original
#define LEFT_ROI_X 40 // upper ROI
#define MID_ROI_X 230
#define LEFT_MID_OFFSET 90
#define RIGHT_MID_OFFSET 20
//#define RIGHT_ROI_X 360 // 600 original
#define RIGHT_ROI_X 410
#define LANE_SPACE 280 // ���� ���� - ž��� ���� �� ���� ���� ������ ����

// latest Update Lane Info
#define BOTH_LANE_UPDATE 0
#define SCARP_LANE_UPDATE 1
#define LEFT_LANE_UPDATE 2
#define RIGHT_LANE_UPDATE 3

//*/

/*
// counter-clockwise parameter
#define LEFT_ROI_X 60 // 250`
#define MID_ROI_X 240
#define LEFT_MID_OFFSET 40
#define RIGHT_MID_OFFSET 40
#define RIGHT_ROI_X 340 // 600
#define LANE_SPACE 215 // ���� ���� - ž��� ���� �� ���� ���� ������ ����
*/

//controller parameter
#define INTERSECT_DENOM 50.0
using namespace std;

struct Convolution {
	cv::Point point;
	double convolutionValue;
	//bool predicate(const Convolution& c1, const Convolution& c2) {
	//   return (c1.convolutionValue < c2.convolutionValue);
	//}
};

// ��ü ROI���� ������ hougline������ ���� 
struct HoughLinesInfo {
	vector<cv::Vec2f> leftLines, rightLines;
	double leftLineSlope; // �������� ����ȭ�� �������ε��� ��� ����
	double rightLineSlope; // ���������� ����ȭ�� �������ε��� ��� ����
	double leftLineInterceptX; // �������� ����ȭ�� �������ε��� ��� x ����
	double rightLineInterceptX; // ���������� ����ȭ�� �������ε��� ��� x ����
	bool isScarp; // �ް�� ���������� ���� ���� => true: �ް��
};

/*
struct leftLineInfo{
	float
}
*/
class LaneDetector {

private:
	cv::Mat normalViewOriginalFrame; // ���ĺ� ���� ����
	int normalViewOffsetX; // ���ĺ� ���� ���󿡼� x offset
	int normalViewOffsetY; // ���ĺ� ���� ���󿡼� y offset
	cv::Mat ipSrcImage; // ��ǥ Ȯ�ο� image => ip image�ε� roi cut���� ���� �̹���
	

	HoughLinesInfo houghLinesInfo; // houghline�� ���� ���� 
	cv::Point leftPointRecord[2]; // ���� ���� ����(����, �Ʒ��� Point 2��)
	cv::Point rightPointRecord[2]; // ������ ���� ����(����, �Ʒ��� Point 2��)
	cv::Point totalPointRecord[2]; // ��ü ROI���� ���� ����(����, �Ʒ��� Point 2��)
	cv::Point leftLanePoints[2]; // ���� ���� ���� ��, �Ʒ��� ��
	cv::Point rightLanePoints[2]; // ������ ���� ���� ��, �Ʒ��� ��
	cv::Point totalLanePoints[2]; // ��ü ROI���� ���� ���� ��, �Ʒ��� ��
	
	cv::Mat transformMatrix; // IPM ��ȯ��� => high cohesion���� Ŭ���� ���� ���� �ű�°� ������????
	cv::Mat invertTransformMatrix; // IPM ����ȯ ���
	cv::Mat ipImage_T; // thresholded Inverse perspective Image 
	cv::Mat ipCannyImage; // ipImage_T�� canny Edge detection�� �̹���
	cv::Mat ipLeftImage_T; // left thresholded Inverse perspective Image 
	cv::Mat ipRightImage_T; // right thresholded Inverse perspective Image 
	cv::Mat ipImage; // Inverse perspective Image & ROI cut complete 
	cv::Mat ipLeftImage; // left Inverse perspective Image 
	cv::Mat ipRightImage; // right  Inverse perspective Image 
	cv::Mat ROS_Invers_Matrix_Frame;
	cv::Mat laneMarkerFrameL;
	cv::Mat laneMarkerFrameR;

	
	int searchWindowLength = 10; // search window�� ������ ���� => ���� search window�� ���̴� 20�̴�.
	int matchFilterWidth;
	int samplePointNum;
	int ipFrameWidth;

	vector<cv::Point> leftSearchWindowPoints; // ����þ� Ŀ���� convolution��ų search window�� ������ - left ROI
	vector<cv::Point> rightSearchWindowPoints; // ����þ� Ŀ���� convolution��ų search window�� ������ - right ROI
	vector<cv::Point> totalSearchWindowPoints; // ����þ� Ŀ���� convolution��ų search window�� ������ - total ROI
	double * gaussianKernel; // ���ο� convolution�� ��ų ����þ� Ŀ��(match filter�μ� Ȱ���Ѵ�)

	bool isLaneDetected; // ���� ������ �ùٷ� ����Ǿ��°�? ����, �����Ÿ� ����
	float ipLeftLaneSlope; // top view���� ����� ���� ���� ����
	float ipRightLaneSlope; // top view���� ����� ������ ���� ����
	cv::Point topLeftPoint1, topLeftPoint2; // topview������ left ���� point
	cv::Point topRightPoint1, topRightPoint2; // topview������ right ���� point
	cv::Point topTotalPoint1, topTotalPoint2; // topview������ total ���� point

	// ���� ���� �� lane markers�� laneModeling�� ���ȴ�.
	vector<cv::Point> leftLaneMarkerPoints;
	vector<cv::Point> rightLaneMarkerPoints;
	vector<cv::Point> totalLaneMarkerPoints;

	// warping source points
	cv::Point topLeftSrcP;
	cv::Point topRightSrcP;
	cv::Point lowLeftSrcP;
	cv::Point lowRightSrcP;

	// warping dstination points
	cv::Point topLeftDstP;
	cv::Point topRightDstP;
	cv::Point lowLeftDstP;
	cv::Point lowRightDstP;

	LaneInfo laneInfo;
	int vanishingLineHight;
	int cameraCenterX;
	
	bool isLeftLaneUpdated;
	bool isRightLaneUpdated;

	int currentLaneUpdateInfo; //  BOTH_LANE_UPDATE 0 , SCARP_LANE_UPDATE , LEFT_LANE_UPDATE 2, RIGHT_LANE_UPDATE 3
	

	
	//
 

public:

	LaneDetector() {
	}
	// constructor => initialize default warping source points
	LaneDetector(int offsetX, int offsetY, int frameHight, int frameWidth, cv::Point topLeft, cv::Point topRight, cv::Point lowRight, cv::Point lowLeft) {
		//topLeftSrcP = cv::Point(75, 0);
		//topRightSrcP = cv::Point(310, 0);
		//lowRightSrcP = cv::Point(width -95, hight);
		//lowLeftSrcP = cv::Point(25, hight);
		
		topLeftSrcP = topLeft;
		topRightSrcP = topRight;
		lowRightSrcP = lowRight;
		lowLeftSrcP = lowLeft;

	
		gaussianKernel = setGaussianKernel(8, 1.5); // ������ �� 6 pixel
		houghLinesInfo.isScarp = false;
		isLaneDetected = false;
		normalViewOffsetX = offsetX;
		normalViewOffsetY = offsetY;

		vanishingLineHight = frameHight * 1.8 / 3;
		//cameraCenterX = frameWidth / 2 + 25;
		cameraCenterX = frameWidth / 2 + 60;
			
		// initialize laneInfo
		laneInfo.setCameraCenterX(cameraCenterX);
		laneInfo.setCameraCenterY(vanishingLineHight);
		laneInfo.setInterceptionPoint(cv::Point(cameraCenterX, vanishingLineHight)); // initial laneInfo intersection Point => middle of the road
		laneInfo.setIsCurvedRoad(false);
		laneInfo.setFrameHight(frameHight);
		laneInfo.setFrameWidth(frameWidth);


	}

	// warping�ϰ��� �ϴ� ������ �װ��� ���� �ʱ�ȭ�ϴ� �Լ�
	void setWarpSrcPoint(int option, cv::Point srcPoint); 
	
	// Persepective transformation(���ٹ� ��ȯ)
	void warpImage(cv::Mat src, int option);
	
	// ipImage_T�� return��F
	cv::Mat getIpImage_T(void);

	// ipSrcImage�� return��
	cv::Mat getIpImage(void);

	// laneInfo
	void setLaneMarker(void);
	void setSamplePoints(vector<cv::Vec2f> &lanes, cv::Mat frame, vector<cv::Point> &searchWindowPoint);
	double* setGaussianKernel(int width, double sigma);
	void matchGaussianFilter(vector<cv::Point> &searchWindowPoint, vector<cv::Point> &LaneMarkerPoints, cv::Mat frame);  // match gaussian filter along search window to obtain lane markers
	void fitsLine(cv::Mat, int option); // fits line from private variable LaneMarkers(left, right) => ���߿� frame �Ķ���� �� ���ְ� �ܼ��� Point ���� �ٲ۴� �۾����� �ٲ� ��!!!!
										//   bool predicate(const Convolution& c1, const Convolution& c2);
	void setOriginalFrame(cv::Mat);
	void drawLaneDetectResult(int option);
	void clusterHouglines(vector<cv::Vec2f>, vector<vector<cv::Vec2f> > &cluster);
	cv::Vec2f getInterceptAndSlope(cv::Vec2f);
	cv::Vec2f twoPoints2Polar(const cv::Vec4i & line);
	bool isLeftRightLaneValid(void);
	bool isScarp(double slope1, double slope2);
	void detectLane(int option); // detect line from houghline info and canny Frame. If lane is detected return true
	void startDetectLaneLogic(void);
	float getHoughlineCluterSlope(void);

	// draw houghlines to frame when rho and theta is given
	void drawHoughlines(float rho, float theta, cv::Mat frame);
	void setLatestUpdateInfo(int);
	void laneInfoUpdate(int option);
	LaneInfo getLaneInfo(void);
	void setLaneInfoFutureScarp(bool);
	void clusterTotalLaneMarkerPoints(void);

	// ROS functions	
	cv::Mat getResultFrame(void);
	cv::Mat getHoughClusterFrame(void);
	cv::Mat getIPLeftImage(void);
	cv::Mat getIPRightImage(void);
	cv::Mat getWarpCoordinateTestFrame(void);
	cv::Mat getLaneMarkerFrameL(void);
	cv::Mat getLaneMarkerFrameR(void);	
	
	void setStartLineCount(int);


};

#endif
