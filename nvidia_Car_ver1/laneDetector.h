

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
#define LANE_SPACE 280 // 차선 간격 - 탑뷰로 봤을 때 양쪽 차선 사이의 간격

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
#define LANE_SPACE 215 // 차선 간격 - 탑뷰로 봤을 때 양쪽 차선 사이의 간격
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

// 전체 ROI에서 추출한 hougline에관한 정부 
struct HoughLinesInfo {
	vector<cv::Vec2f> leftLines, rightLines;
	double leftLineSlope; // 왼쪽으로 군집화된 허프라인들의 평균 기울기
	double rightLineSlope; // 오른쪽으로 군집화된 허프라인들의 평균 기울기
	double leftLineInterceptX; // 왼쪽으로 군집화된 허프라인들의 평균 x 절편
	double rightLineInterceptX; // 오른쪽으로 군집화된 허프라인들의 평균 x 절편
	bool isScarp; // 급경사 도로인지에 대한 정보 => true: 급경사
};

/*
struct leftLineInfo{
	float
}
*/
class LaneDetector {

private:
	cv::Mat normalViewOriginalFrame; // 정식뷰 원본 영상
	int normalViewOffsetX; // 정식뷰 원본 영상에서 x offset
	int normalViewOffsetY; // 정식뷰 원본 영상에서 y offset
	cv::Mat ipSrcImage; // 좌표 확인용 image => ip image인데 roi cut되지 않은 이미지
	

	HoughLinesInfo houghLinesInfo; // houghline에 대한 정보 
	cv::Point leftPointRecord[2]; // 왼쪽 차선 정보(윗쪽, 아랫쪽 Point 2개)
	cv::Point rightPointRecord[2]; // 오른쪽 차선 정보(윗쪽, 아랫쪽 Point 2개)
	cv::Point totalPointRecord[2]; // 전체 ROI에서 차선 정보(윗쪽, 아랫쪽 Point 2개)
	cv::Point leftLanePoints[2]; // 왼쪽 차선 위쪽 점, 아래쪽 점
	cv::Point rightLanePoints[2]; // 오른쪽 차선 위쪽 점, 아래쪽 점
	cv::Point totalLanePoints[2]; // 전체 ROI에서 차선 위쪽 점, 아래쪽 점
	
	cv::Mat transformMatrix; // IPM 변환행렬 => high cohesion으로 클래스 내로 변수 옮기는게 낫겠지????
	cv::Mat invertTransformMatrix; // IPM 역변환 행렬
	cv::Mat ipImage_T; // thresholded Inverse perspective Image 
	cv::Mat ipCannyImage; // ipImage_T의 canny Edge detection한 이미지
	cv::Mat ipLeftImage_T; // left thresholded Inverse perspective Image 
	cv::Mat ipRightImage_T; // right thresholded Inverse perspective Image 
	cv::Mat ipImage; // Inverse perspective Image & ROI cut complete 
	cv::Mat ipLeftImage; // left Inverse perspective Image 
	cv::Mat ipRightImage; // right  Inverse perspective Image 
	cv::Mat ROS_Invers_Matrix_Frame;
	cv::Mat laneMarkerFrameL;
	cv::Mat laneMarkerFrameR;

	
	int searchWindowLength = 10; // search window의 길이의 절반 => 실제 search window의 길이는 20이다.
	int matchFilterWidth;
	int samplePointNum;
	int ipFrameWidth;

	vector<cv::Point> leftSearchWindowPoints; // 가우시안 커널을 convolution시킬 search window의 시작점 - left ROI
	vector<cv::Point> rightSearchWindowPoints; // 가우시안 커널을 convolution시킬 search window의 시작점 - right ROI
	vector<cv::Point> totalSearchWindowPoints; // 가우시안 커널을 convolution시킬 search window의 시작점 - total ROI
	double * gaussianKernel; // 도로와 convolution을 시킬 가우시안 커널(match filter로서 활용한다)

	bool isLaneDetected; // 양쪽 차선이 올바로 검출되었는가? 평행, 일정거리 유지
	float ipLeftLaneSlope; // top view에서 검출된 왼쪽 차선 기울기
	float ipRightLaneSlope; // top view에서 검출된 오른쪽 차선 기울기
	cv::Point topLeftPoint1, topLeftPoint2; // topview에서의 left 차선 point
	cv::Point topRightPoint1, topRightPoint2; // topview에서의 right 차선 point
	cv::Point topTotalPoint1, topTotalPoint2; // topview에서의 total 차선 point

	// 최종 검출 된 lane markers로 laneModeling에 사용된다.
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

	
		gaussianKernel = setGaussianKernel(8, 1.5); // 도로의 폭 6 pixel
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

	// warping하고자 하는 영역의 네개의 점을 초기화하는 함수
	void setWarpSrcPoint(int option, cv::Point srcPoint); 
	
	// Persepective transformation(원근법 변환)
	void warpImage(cv::Mat src, int option);
	
	// ipImage_T를 return함F
	cv::Mat getIpImage_T(void);

	// ipSrcImage를 return함
	cv::Mat getIpImage(void);

	// laneInfo
	void setLaneMarker(void);
	void setSamplePoints(vector<cv::Vec2f> &lanes, cv::Mat frame, vector<cv::Point> &searchWindowPoint);
	double* setGaussianKernel(int width, double sigma);
	void matchGaussianFilter(vector<cv::Point> &searchWindowPoint, vector<cv::Point> &LaneMarkerPoints, cv::Mat frame);  // match gaussian filter along search window to obtain lane markers
	void fitsLine(cv::Mat, int option); // fits line from private variable LaneMarkers(left, right) => 나중에 frame 파라미터 값 없애고 단순히 Point 값만 바꾼는 작업으로 바꿀 것!!!!
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
