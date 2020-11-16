
#include "laneDetector.h"
#include <algorithm>
#include <ctime>

void LaneDetector::setWarpSrcPoint(int option, cv::Point srcPoint) {
	switch (option) {
	case 0:
		topLeftSrcP = srcPoint;
		cout << "Set top left source point: " << topLeftSrcP << endl;
		break;
	case 1:
		topRightSrcP = srcPoint;
		cout << "Set top right source point: " << topRightSrcP << endl;
		break;
	case 2:
		lowRightSrcP = srcPoint;
		cout << "Set low right source point: " << lowRightSrcP << endl;
		break;
	case 3:
		lowLeftSrcP = srcPoint;
		cout << "Set low left source point: " << lowLeftSrcP << endl;
		break;
	default:
		cout << "Error in setWarpSrcPoint()" << endl;
	}
}


void LaneDetector::warpImage(cv::Mat src, int option) {
	ROS_Invers_Matrix_Frame = src;
	
	//imshow("test Source", src);
	cv::Point2f srcPoint[4]; // 오른쪽 위 => 오른쪽 아래 => 왼쪽 위 => 왼쪽 아래 순
	cv::Point2f dstPoint[4]; // 오른쪽 위 => 오른쪽 아래 => 왼쪽 위 => 왼쪽 아래 순
	float width = topRightSrcP.x - topLeftSrcP.x;
	float hight = lowRightSrcP.y - topRightSrcP.y;
	cv::Mat dst = cv::Mat(src.size(), src.type());; // warping한 이미지

	// warping source points
	srcPoint[0] = topLeftSrcP;
	srcPoint[1] = lowLeftSrcP;
	srcPoint[2] = lowRightSrcP;
	srcPoint[3] = topRightSrcP;

	//	cv::circle(ROS_Invers_Matrix_Frame, cv::Point(60, 10) , 5, cv::Scalar(0,255,0),2, 8);
	//	cv::circle(ROS_Invers_Matrix_Frame, cv::Point(200, 10) , 5, cv::Scalar(0,255,0),2, 8);
	//	cv::circle(ROS_Invers_Matrix_Frame, cv::Point(250, 10) , 5, cv::Scalar(0,255,0),2, 8);
	//	cv::circle(ROS_Invers_Matrix_Frame, cv::Point(270, 10) , 5, cv::Scalar(0,255,0),2, 8);
	//	cv::circle(ROS_Invers_Matrix_Frame, cv::Point(340, 10) , 5, cv::Scalar(0,255,0),2, 8);
	
	/*
	//ROS warping matrix - coordinate testing code
	for(int i = 0 ; i < 4; i++){
		cv::circle(ROS_Invers_Matrix_Frame, srcPoint[i], 5, cv::Scalar(0,255,0),2, 8);
	}
	*/




	// warping destination points
	dstPoint[0] = cv::Point2f(topLeftSrcP.x, topLeftSrcP.y);
	dstPoint[1] = cv::Point2f(topLeftSrcP.x, topLeftSrcP.y + hight);
	dstPoint[2] = cv::Point2f(topLeftSrcP.x + width, topLeftSrcP.y + hight);
	dstPoint[3] = cv::Point2f(topLeftSrcP.x + width, topLeftSrcP.y);

	// Warping image and getting transformation matrix
	transformMatrix = cv::getPerspectiveTransform(srcPoint, dstPoint); // 변환 행렬
	cv::warpPerspective(src, dst, transformMatrix, dst.size(), CV_INTER_LINEAR); // top view로 변환
	cv::invert(transformMatrix, invertTransformMatrix); // calculating the inverse of inverse perspective transformation matrix(역변환 행렬)  
	ipFrameWidth = dst.cols;

	// getting ROI Inverse Perspective images
	cv::Rect roi = cv::Rect(cv::Point(LEFT_ROI_X, 0), cv::Point(MID_ROI_X - LEFT_MID_OFFSET, dst.rows));
	cv::Mat leftIPImage = dst(roi);
	roi = cv::Rect(cv::Point(MID_ROI_X + RIGHT_MID_OFFSET, 0), cv::Point(RIGHT_ROI_X, dst.rows));
	cv::Mat rightIPImage = dst(roi);
	roi = cv::Rect(cv::Point(LEFT_ROI_X, 0), cv::Point(RIGHT_ROI_X, dst.rows));
	cv::Mat wholeIPImage = dst(roi);
	ipSrcImage = dst;

	if (option == TEMPORAL_THRESHOLD_OPTION) {
		ipImage_T = wholeIPImage;
		ipLeftImage_T = leftIPImage;
		ipRightImage_T = rightIPImage;
		samplePointNum = ipImage_T.rows / SEARCH_WINDOW_INTERVAL;
		//cout << "test samplePointIngerval: " << samplePointNum;
		//imshow("Left side", ipLeftImage_T);
		//imshow("Left side", ipLeftImage_T);
		//imshow("Right side", ipRightImage_T);
	}
	else if (option == TEMPORAL_OPTION) {
		ipImage = wholeIPImage;
		ipLeftImage = leftIPImage;
		ipRightImage = rightIPImage;
		//imshow("Left side", ipLeftImage);
		//imshow("Right side", ipRightImage);
	}
}

cv::Mat LaneDetector::getIpImage_T(void) {
	return ipImage_T;
}

cv::Mat LaneDetector::getIpImage(void) {
	// ROS code
	/*	
	// left, mid, right road roi in bird view frame
	int leftOffset =  40;
	int rightOffset = 40;
	circle(ipSrcImage, cvPoint(60, 10), 7, cv::Scalar(255), 2, 8);
	circle(ipSrcImage, cvPoint(200 -leftOffset, 10), 5, cv::Scalar(255), 2, 8);
	circle(ipSrcImage, cvPoint(200, 10), 7, cv::Scalar(255), 2, 8);
	circle(ipSrcImage, cvPoint(200 + rightOffset, 10), 5, cv::Scalar(255), 2, 8);
	circle(ipSrcImage, cvPoint(360, 10), 7, cv::Scalar(255), 2, 8);
	*/
	
	// check the lane interval
	/*
	circle(ipSrcImage, cvPoint(92, 10), 7, cv::Scalar(255), 2, 8);
	circle(ipSrcImage, cvPoint(100, 10), 7, cv::Scalar(255), 2, 8);
	*/	
	return ipSrcImage;
	
}

void LaneDetector::setLaneMarker(void) {
	vector<cv::Vec2f> totalLanes;
	vector<vector<cv::Vec2f> > houghLineClusters;
	cv::Mat temp;

	int threshold = 15;
	
	cv::Canny(ipImage_T, ipCannyImage, 100, 150, 3);
	cv::HoughLines(ipCannyImage, totalLanes, 1, CV_PI / 180 * 1.5, threshold, 0, 0, -CV_PI / 2, CV_PI / 2);

	if (totalLanes.size() != 0) {
		// 최대 25개의 houghline만을 사용
		while (totalLanes.size() > 25) {
			totalLanes.erase(totalLanes.begin() + 25);
		}
		clusterHouglines(totalLanes, houghLineClusters);
	}

	/*
	급커브 판단 논리 => 군집화된 왼쪽과 오른쪽 허프라인들의 평균 x 절편의 간격이 20 미만이고(탑뷰에서 도로의 폭 120), 왼쪽 오른쪽 군집화된 허프라인의 평균기울기가 모두
	tan 50에서 tan 0 사이의 기울기를 유지할 때 급커브로 판단.
	*/
	//cout << "testtttttttttttttt???????? " << abs(houghLinesInfo.leftLineInterceptX - houghLinesInfo.rightLineInterceptX) << endl;
}






void LaneDetector::startDetectLaneLogic(void){

	/////////////////// Detect lane Algorithm/////////////////////////////////
	detectLane(SPLIT_ROI_OPTION);
	if(isLeftLaneUpdated == true && isRightLaneUpdated == true && isLeftRightLaneValid()){
		setLatestUpdateInfo(BOTH_LANE_UPDATE);
		cout << "case1111111111111111111111111111111111111111 - left right valid " << endl;
		// If detected left and right lane are valid
		leftPointRecord[0] = leftLanePoints[0];
		leftPointRecord[1] = leftLanePoints[1];
		rightPointRecord[0] = rightLanePoints[0];
		rightPointRecord[1] = rightLanePoints[1];

		drawLaneDetectResult(SPLIT_ROI_OPTION);
		houghLinesInfo.isScarp = false;
	} else{
		cout << "houghLinesInfo.leftLineSlope: " << houghLinesInfo.leftLineSlope << endl;
		cout << "houghLinesInfo.rightLineSlope: " <<houghLinesInfo.rightLineSlope << endl;
//abs(houghLinesInfo.leftLineInterceptX - houghLinesInfo.rightLineInterceptX) < 50&& 
/*
if((((-1.4 <= houghLinesInfo.leftLineSlope && houghLinesInfo.leftLineSlope <= 0) && (-1.4 <= houghLinesInfo.rightLineSlope && houghLinesInfo.rightLineSlope <= 0))
			|| (((0 <= houghLinesInfo.leftLineSlope && houghLinesInfo.leftLineSlope <= 1.25) && (0 <= houghLinesInfo.rightLineSlope && houghLinesInfo.rightLineSlope <= 1.25))))
			)
*/


	     	if(
	           ( //x
		    ( //a	
		       ( (-1.4 <= houghLinesInfo.leftLineSlope) && (houghLinesInfo.leftLineSlope <=-0.3) )
			&&
		       ( (-1.4 <= houghLinesInfo.rightLineSlope) && (houghLinesInfo.rightLineSlope <=-0.3) )
                    ) //a'
                    ||
		    ( //b

		       ( (0.3 <= houghLinesInfo.leftLineSlope) && (houghLinesInfo.leftLineSlope <= 1.6) )
                        &&
		       ( (0.3 <= houghLinesInfo.rightLineSlope) && (houghLinesInfo.rightLineSlope <= 1.6) )
                    ) //b'
                  ) //x
                  &&
                  ( //c
			( (-1 * normalViewOriginalFrame.cols)  < totalLanePoints[1].x )
			&&
			( (2 * normalViewOriginalFrame.cols) > totalLanePoints[1].x ) 
		  ) //c'
         	) //if'
	{
			// if the road is scarp road
			setLatestUpdateInfo(SCARP_LANE_UPDATE);
			cout << "case2222222222222222222222222 scarp" << endl;
			//totalPointRecord[0] = totalLanePoints[0];
			//totalPointRecord[1] = totalLanePoints[1];
			detectLane(NON_SPLIT_ROI_OPTION);
			totalPointRecord[0] = totalLanePoints[0];
			totalPointRecord[1] = totalLanePoints[1];
			drawLaneDetectResult(NON_SPLIT_ROI_OPTION);
			houghLinesInfo.isScarp = true;				
		} else if((isLeftLaneUpdated == true && isRightLaneUpdated == false) || (isRightLaneUpdated == false || isLeftLaneUpdated == true)){
			// detect with only one lane(but not scarp lane);
			houghLinesInfo.isScarp = false;
			if(isLeftLaneUpdated == true){
				cout << "case3333333333333333333333333 left only" << endl;
				setLatestUpdateInfo(LEFT_LANE_UPDATE);
				leftPointRecord[0] = leftLanePoints[0];
				leftPointRecord[1] = leftLanePoints[1];
				drawLaneDetectResult(SPLIT_LEFT_ROI_OPTION);
			} else if(isRightLaneUpdated == true){
				cout << "case44444444444444444444444444 right only"<< endl;
				setLatestUpdateInfo(RIGHT_LANE_UPDATE);
				rightPointRecord[0] = rightLanePoints[0];
				rightPointRecord[1] = rightLanePoints[1];
				drawLaneDetectResult(SPLIT_RIGHT_ROI_OPTION);
			}
			
		} else{
			// if there is no lane detected and can't tell it's scarp road => resuse lane info
			if(currentLaneUpdateInfo == BOTH_LANE_UPDATE){
				cout << "caseRRRRRRR111111111111111 both lane" << endl;
				drawLaneDetectResult(SPLIT_ROI_OPTION);
			} else if(currentLaneUpdateInfo == SCARP_LANE_UPDATE){
				cout << "caseRRRRRRR222222222222222 scarp lane"  << endl;
				drawLaneDetectResult(NON_SPLIT_ROI_OPTION);
			} else if(currentLaneUpdateInfo == LEFT_LANE_UPDATE){
				drawLaneDetectResult(SPLIT_LEFT_ROI_OPTION);
				cout << "caseRRRRRRRR333333333333333 left lane"  << endl;
			} else if(currentLaneUpdateInfo == RIGHT_LANE_UPDATE){
				drawLaneDetectResult(SPLIT_RIGHT_ROI_OPTION);
			} else{
				cout<< "caseRRRRRRRRRRRRRR444444444 right lane " << endl;
			}

					
		}		
	}









	/*if ((abs(houghLinesInfo.leftLineInterceptX - houghLinesInfo.rightLineInterceptX) < 50
		&& (((-0.85 <= houghLinesInfo.leftLineSlope && houghLinesInfo.leftLineSlope <= 0) && (-0.85 <= houghLinesInfo.rightLineSlope && houghLinesInfo.rightLineSlope <= 0))
			|| (((0 <= houghLinesInfo.leftLineSlope && houghLinesInfo.leftLineSlope <= 1.2) && (0 <= houghLinesInfo.rightLineSlope && houghLinesInfo.rightLineSlope <= 1.2))))
		|| houghLinesInfo.isScarp)
		)
*/
/*
if ((abs(houghLinesInfo.leftLineInterceptX - houghLinesInfo.rightLineInterceptX) < 50
		)
		|| houghLinesInfo.isScarp
		) {
		cout << "Scarp\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\" << endl;
		detectLane(SPLIT_ROI_OPTION);
		
		if (isLeftRightLaneValid()) { // 급커브 진입후 break조건(양쪽 valid 차선 나오면 그걸로 사용)
			drawLaneDetectResult(SPLIT_ROI_OPTION);
			houghLinesInfo.isScarp = false;
		} else { //전체 ROI에서 차선 검출
			detectLane(NON_SPLIT_ROI_OPTION);
			drawLaneDetectResult(NON_SPLIT_ROI_OPTION);
			houghLinesInfo.isScarp = true;
		}
	} else { // 급커브가 아닌 경우
		detectLane(SPLIT_ROI_OPTION);
		
		if (isLeftRightLaneValid()) { // valid 차선이 detect된 경우
			// valid한 차선일 경우 이전 차선 정보 update
			cout << "in11111111111111111111111 "<< endl;
			leftPointRecord[0] = leftLanePoints[0];
			leftPointRecord[1] = leftLanePoints[1];
			rightPointRecord[0] = rightLanePoints[0];
			rightPointRecord[1] = rightLanePoints[1];
			drawLaneDetectResult(SPLIT_ROI_OPTION);
		} else { // valid하지 않은 차선이 detect된 경우 => 이전 차선 유지
			cout << "in222222222222222222222 " << endl;
			drawLaneDetectResult(SPLIT_ROI_OPTION);
		}
	}
*/

}

//ipImage의 left, right ROI분리해서 lanemarker setting하고 차선을 검출 => 올바른 차선 검출일 경우 private 변수 isLaneDetected를 true로 바꿔줌
void LaneDetector::detectLane(int option) {
	cv::Mat rgbFrame;
	int threshold = 15;

	// left, right ROI분리해서 lanemarker setting하는 논리
	if (option == SPLIT_ROI_OPTION) {
		cv::Mat ipLeftCannyImage;
		cv::Mat ipRightCannyImage;
		vector<cv::Vec2f> leftHoughLines;
		vector<cv::Vec2f> rightHoughLines;

		cv::Canny(ipLeftImage_T, ipLeftCannyImage, 100, 150, 3);
		cv::Canny(ipRightImage_T, ipRightCannyImage, 100, 150, 3);
		//imshow("canny test", ipRightCannyImage);
		//cv::HoughLines(ipLeftImage_T, leftLanes, 1, CV_PI / 180 * 1.5, threshold, 0, 0, -CV_PI / 5, CV_PI / 5);
		//cv::HoughLines(ipRightImage_T, rightLanes, 1, CV_PI / 180 * 1.5, threshold, 0, 0, -CV_PI / 5, CV_PI / 5);
		cv::HoughLines(ipLeftCannyImage, leftHoughLines, 1, CV_PI / 180, threshold, 0, 0, -CV_PI / 3.5, CV_PI / 3.5);
		cv::HoughLines(ipRightCannyImage, rightHoughLines, 1, CV_PI / 180, threshold, 0, 0, -CV_PI / 3.5, CV_PI / 3.5);

		

		// 오른쪽 ROI영역 
		if (rightHoughLines.size() != 0) {
			while (rightHoughLines.size() > 20) {
				rightHoughLines.erase(rightHoughLines.begin() + 20);
			}
			//cout << "right in !!!!!!!!!!!!!!!!! " << endl;
			cv::cvtColor(ipRightImage_T, rgbFrame, CV_GRAY2BGR);
			setSamplePoints(rightHoughLines, rgbFrame, rightSearchWindowPoints);
			//cout << "rightSearchPoint number: " << rightSearchWindowPoints.size() << endl;
			//cout << "test ipRightImage.cols: " << ipRightImage.cols << endl; // 100
			if (rightSearchWindowPoints.size() != 0) matchGaussianFilter(rightSearchWindowPoints, rightLaneMarkerPoints, ipRightImage); 
																																				
			cout << "rigthLaneMarker size: " << rightLaneMarkerPoints.size() << endl;
			if (rightLaneMarkerPoints.size() <= 2) {
				//cv::line(normalViewOriginalFrame, rightPointRecord[0], rightPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
				//cout << "warning RRRRRRRRRRRRRRRR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
			}

			for (int i = 0; i < rightLaneMarkerPoints.size(); i++) {
				circle(rgbFrame, rightLaneMarkerPoints.at(i), 7, cv::Scalar(0, 255, 0), 2, 8);
			}
			rgbFrame.copyTo(laneMarkerFrameR);
			//imshow("test lanemMakerPoints on right lane image", rgbFrame);
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (rightLaneMarkerPoints.size() > 2) fitsLine(rgbFrame, RIGHT_OPTION);
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			rightSearchWindowPoints.clear();
			rightLaneMarkerPoints.clear();
			//cout << "after right clear: " << rightLaneMarkerPoints.size() << endl;
		}


if (leftHoughLines.size() != 0) {
			while (leftHoughLines.size() > 20) {
				leftHoughLines.erase(leftHoughLines.begin() + 20);
			}

			//cout << "left in " << endl;
			cv::cvtColor(ipLeftImage_T, rgbFrame, CV_GRAY2BGR);
			setSamplePoints(leftHoughLines, rgbFrame, leftSearchWindowPoints);
			if (leftSearchWindowPoints.size() != 0) matchGaussianFilter(leftSearchWindowPoints, leftLaneMarkerPoints, ipImage); // match gaussian filter along search window to obtain lane markers

			for (int i = 0; i < totalLaneMarkerPoints.size(); i++) {
				circle(rgbFrame, totalLaneMarkerPoints.at(i), 7, cv::Scalar(0, 255, 0), 2, 8);
			}
			

			//imshow("test lanemMakerPoints on Left lane image", rgbFrame);
			rgbFrame.copyTo(laneMarkerFrameL);
			
			if (leftLaneMarkerPoints.size() <= 2) {
				//cv::line(normalViewOriginalFrame, leftPointRecord[0], leftPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
				//cout << "waring LLLLLLLLLLLLLLLLLLL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
			}

			if (leftLaneMarkerPoints.size() > 2) fitsLine(rgbFrame, LEFT_OPTION);
			leftSearchWindowPoints.clear();
			leftLaneMarkerPoints.clear();
		}




	}
	// total ROI 사용
	else if (option == NON_SPLIT_ROI_OPTION) {
		vector<cv::Vec2f> houghLines;
		cv::HoughLines(ipCannyImage, houghLines, 1, CV_PI / 180 * 1.5, threshold, 0, 0, -CV_PI / 3, CV_PI / 3);
		//cout <<"Test right hougline number: " << rightLanes.size()<< endl;
		//cout << "test ipimage.cols: " << ipImage.cols << endl; // 400

		if (houghLines.size() != 0) {
			while (houghLines.size() > 20) {
				houghLines.erase(houghLines.begin() + 20);
			}
			//cout << "left in " << endl;
			cv::cvtColor(ipImage_T, rgbFrame, CV_GRAY2BGR);
			setSamplePoints(houghLines, rgbFrame, totalSearchWindowPoints);
			if (totalSearchWindowPoints.size() != 0) matchGaussianFilter(totalSearchWindowPoints, totalLaneMarkerPoints, ipImage); // match gaussian filter along search window to obtain lane markers
			
			/*
			for (int i = 0; i < totalLaneMarkerPoints.size(); i++) {
				circle(rgbFrame, totalLaneMarkerPoints.at(i), 3, cv::Scalar(0, 255, 0), 2, 8);
			}
			*/
			//rgbFrame.copyTo(laneMarkerFrameL);
			if (totalLaneMarkerPoints.size() <= 2) {
				//cv::line(normalViewOriginalFrame, leftPointRecord[0], leftPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
				//cout << "waring LLLLLLLLLLLLLLLLLLL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
			}
			if (totalLaneMarkerPoints.size() > 2){
				clusterTotalLaneMarkerPoints();
				// added code => check the roi and use only one part of the roi
				/*
					vector<cv::Point> leftPointVector;
					vector<cv::Point> rightPointVector;
					for(int i = 0 ; i < totalLaneMarkerPoints.size(); i++){
						if(totalLaneMarkerPoints.at(i).x < MID_ROI_X){
							leftPointVector.push_back(totalLaneMarkerPoints.at(i));						
						}else if(totalLaneMarkerPoints.at(i).x > MID_ROI_X){
							rightPointVector.push_back(totalLaneMarkerPoints.at(i));						
						}
					}
				if(leftPointVector.size() > rightPointVector.size()){
					totalLaneMarkerPoints.clear();
					totalLaneMarkerPoints = leftPointVector;
				}else{
					totalLaneMarkerPoints.clear();
					totalLaneMarkerPoints = rightPointVector;
				}
				*/
				
				fitsLine(rgbFrame, TOTAL_OPTION);
			}
			totalSearchWindowPoints.clear();
			totalLaneMarkerPoints.clear();
		}
	}
}




bool LaneDetector::isLeftRightLaneValid(void) {
	// 검출된 양쪽 차선이 valid한 차선인지 확인 => 서로 평행, 차선 간격이 일정 범위(영상마다 다름 - laneDetector.h의 ) 안에 들어올 것
/*
	if (((abs(ipLeftLaneSlope) - abs(ipRightLaneSlope)) < 0.4) && ((abs(topLeftPoint1.x - topRightPoint1.x) + abs(topLeftPoint2.x - topRightPoint2.x)) / 2 < LANE_SPACE + 25)
		&& ((abs(topLeftPoint1.x - topRightPoint1.x) + abs(topLeftPoint2.x - topRightPoint2.x)) / 2 > LANE_SPACE - 25)) */
	
	
	//ROS - Test Code
	cout << "aaaaaabbbbbbbbbbssssssssssss: " << topLeftPoint1.x << endl;
	cout << "aaaaaabbbbbbbbbbssssssssssss2222222: " << leftLanePoints[0] << endl;
	cout << "aaaaaabbbbbbbbbbssssssssssss: " << topRightPoint1.x << endl;	

	//cout << "aaaaaabbbbbbbbbbssssssssssss: " << abs(topLeftPoint1.x - topRightPoint1.x) << endl;
	//cout << "aaaaaabbbbbbbbbbssssssssssss: " << abs(topRightPoint2.x - topRightPoint2.x) << endl; 
	cout << "test abs: " <<abs(topLeftPoint1.x - topRightPoint1.x);

	
	if(
      	   (   (abs(topLeftPoint1.x - topRightPoint1.x) < LANE_SPACE + 30)
		&& (abs(topLeftPoint1.x - topRightPoint1.x) > LANE_SPACE - 30))
		&&
	       ( (abs(topLeftPoint2.x - topRightPoint2.x) < LANE_SPACE + 30)
		&& (abs(topLeftPoint2.x - topRightPoint2.x) > LANE_SPACE - 30))
		&&
		(topLeftPoint1.x != 0 && topRightPoint1.x != 0)
		
	) {
		cout << "Valid Lane ~~~" << endl;
		return true;
	}
	else { // valid하지 않은 차선일 경우 => 이전 차선 정보 update하지 않음(이전 차선 유지)
		//cout << "Invalid Lane ~~~" << endl;
		//cout << "Valid Lane !!!!!!!!!!!!!!!!!!!! NOt" << endl;
		return false;
	}
}




void LaneDetector::clusterHouglines(vector<cv::Vec2f> houghlines, vector<vector<cv::Vec2f> > &houghlineCluster) {

	int clusterNum = 0;
	float averageXintercept0 = 0; // 왼쪽 군집화된 houghline들의 평균 x 절편
	float averageXintercept1 = 0; // 오른쪽 군집화된 houghline들의 평균 x 절편
	float averageSlope0 = 0; // 왼쪽 군집화된 houghline들의 평균 기울기
	float averageSlope1 = 0; // 오른쪽 군집화된 houghline들의 평균 기울기
	double xInterceptSum = 0;
	double slopeSum = 0;
	cv::Vec2f interceptAndSlopeInfo; // info[0]: x intercept , info[1] = slope

	// houghlines를 rho값을 기준으로 오름차순으로 정렬
	sort(houghlines.begin(), houghlines.end(), [](const cv::Vec2f & lhs, const cv::Vec2f & rhs) {
		return lhs[0] < rhs[0];
	});

	//cout << "houghline size: " << houghlines.size() << endl;

	// houghline clustering 알고리즘
	while (houghlines.size() >= 4) {
		int houghlineSize = houghlines.size();
		int midIndex = houghlines.size() / 2;
		float midRho = houghlines.at(midIndex)[0];
		vector<cv::Vec2f> tempCluster;

		for (int i = 0; i < houghlines.size(); ++i) {
			if ((houghlines.at(i)[0] - midRho) < 10 && (houghlines.at(i)[0] - midRho) > -10) {
				tempCluster.push_back(houghlines.at(i));
				houghlines.erase(houghlines.begin() + i);
				i--;
			}
		}

		if (!tempCluster.empty()) {
			houghlineCluster.push_back(tempCluster);
			clusterNum++;
		}
		else {
			break;
		}
	}

	//////////////////////////////////////////// cluster 분류 후 왼쪽 오른쪽 houghline cluter 검출 //////////////////////////////////////
	if (clusterNum == 0) {
		// 이전 차선 유지
	}
	else if (clusterNum == 1) {
		// 왼쪽 선인지 오른쪽 선인지 판단하는 논리만 집어 넣으면 됨

	}
	else if (clusterNum >= 2) {
		sort(houghlineCluster.begin(), houghlineCluster.end(), [](const vector<cv::Vec2f> & lhs, vector<cv::Vec2f> & rhs) {
			return lhs.size() > rhs.size();
		});

		// 왼쪽 선과 오른쪽 선 구분
		for (int i = 0; i < houghlineCluster.at(0).size(); i++) {
			interceptAndSlopeInfo = getInterceptAndSlope(houghlineCluster.at(0).at(i));
			xInterceptSum += interceptAndSlopeInfo[0];
			slopeSum += interceptAndSlopeInfo[1];
		}

		averageXintercept0 = xInterceptSum / houghlineCluster.at(0).size();
		averageSlope0 = slopeSum / houghlineCluster.at(0).size();

		xInterceptSum = 0;
		slopeSum = 0;

		for (int i = 0; i < houghlineCluster.at(1).size(); i++) {
			interceptAndSlopeInfo = getInterceptAndSlope(houghlineCluster.at(1).at(i));
			xInterceptSum += interceptAndSlopeInfo[0];
			slopeSum += interceptAndSlopeInfo[1];

		}
		averageXintercept1 = xInterceptSum / houghlineCluster.at(1).size();
		averageSlope1 = slopeSum / houghlineCluster.at(1).size();

		if (averageXintercept0 > averageXintercept1) {
			// houghlineCluster.at(0) 이 오른쪽 차선 집합
			// 절편 정보 입력
			houghLinesInfo.rightLineInterceptX = averageXintercept0;
			houghLinesInfo.leftLineInterceptX = averageXintercept1;
			houghLinesInfo.rightLineSlope = averageSlope0;
			houghLinesInfo.leftLineSlope = averageSlope1;

			houghLinesInfo.rightLines.resize(houghlineCluster.at(0).size());
			copy(houghlineCluster.at(0).begin(), houghlineCluster.at(0).end(), houghLinesInfo.rightLines.begin());

			houghLinesInfo.leftLines.resize(houghlineCluster.at(1).size());
			copy(houghlineCluster.at(1).begin(), houghlineCluster.at(1).end(), houghLinesInfo.leftLines.begin());

		}
		else {
			// houglineintercept.at(1)이 오른쪽 차선 집합
			// 절편 정보 입력
			houghLinesInfo.rightLineInterceptX = averageXintercept1;
			houghLinesInfo.leftLineInterceptX = averageXintercept0;
			houghLinesInfo.rightLineSlope = averageSlope1;
			houghLinesInfo.leftLineSlope = averageSlope0;

			houghLinesInfo.rightLines.resize(houghlineCluster.at(1).size());
			copy(houghlineCluster.at(1).begin(), houghlineCluster.at(1).end(), houghLinesInfo.rightLines.begin());

			houghLinesInfo.leftLines.resize(houghlineCluster.at(0).size());
			copy(houghlineCluster.at(0).begin(), houghlineCluster.at(0).end(), houghLinesInfo.leftLines.begin());
		}

	}

	//cout << "left size: " << leftLanes.size() << endl;
	//cout << "right size: " << rightLanes.size() << endl;

	//////////////////////////  draw left lines ////////////////////////////
	for (int i = 0; i < houghLinesInfo.leftLines.size(); i++) {
		float rho = houghLinesInfo.leftLines.at(i)[0];
		float theta = houghLinesInfo.leftLines.at(i)[1];
		//drawHoughlines(rho, theta, ipImage);
		cv::Point pt1;
		cv::Point pt2;

		// Point 구하기
		if (theta < CV_PI / 4. || theta > 3. * CV_PI / 4.) {
			pt1 = cv::Point(rho / cos(theta), 0);
			pt2 = cv::Point((rho - ipImage.rows * sin(theta)) / cos(theta), ipImage.rows);
			//cv::line(ipImage, pt1, pt2, cv::Scalar(255), 1);
		}
		else {
			pt1 = cv::Point(0, rho / sin(theta));
			pt2 = cv::Point(ipImage.cols, (rho - ipImage.cols * cos(theta)) / sin(theta));
			//cv::line(ipImage, pt1, pt2, cv::Scalar(255), 1);
		}
	}


	//////////////////////////  draw right lines ////////////////////////////
	for (int i = 0; i < houghLinesInfo.rightLines.size(); i++) {
		float rho = houghLinesInfo.rightLines.at(i)[0];
		float theta = houghLinesInfo.rightLines.at(i)[1];
		cv::Point pt1;
		cv::Point pt2;

		// Point 구하기
		if (theta < CV_PI / 4. || theta > 3. * CV_PI / 4.) {
			pt1 = cv::Point(rho / cos(theta), 0);
			pt2 = cv::Point((rho - ipImage.rows * sin(theta)) / cos(theta), ipImage.rows);
			//cv::line(ipImage, pt1, pt2, cv::Scalar(0), 1);
		}
		else {
			pt1 = cv::Point(0, rho / sin(theta));
			pt2 = cv::Point(ipImage.cols, (rho - ipImage.cols * cos(theta)) / sin(theta));
			//cv::line(ipImage, pt1, pt2, cv::Scalar(0), 1);
		}
	}

	//imshow("Clustered houghlines", ipImage);

}

void LaneDetector::drawHoughlines(float rhoValue, float thetaValue, cv::Mat frame) {
	float rho = rhoValue;
	float theta = thetaValue;
	cv::Point pt1;
	cv::Point pt2;
	double slope = 0;
	float yIntercept = 0;
	float xIntercept = 0;
	//vector<cv::Point> samplePoints;

	// Point 구하기
	if (theta < CV_PI / 4. || theta > 3. * CV_PI / 4.) {
		pt1 = cv::Point(rho / cos(theta), 0);
		pt2 = cv::Point((rho - frame.rows * sin(theta)) / cos(theta), frame.rows);
		slope = (double)(pt1.y - pt2.y) / (double)(pt1.x - pt2.x);
		houghLinesInfo.leftLineSlope = slope;
		//cv::line(frame, pt1, pt2, cv::Scalar(0), 1);
	}
	else {
		pt1 = cv::Point(0, rho / sin(theta));
		pt2 = cv::Point(frame.cols, (rho - frame.cols * cos(theta)) / sin(theta));
		slope = (double)(pt1.y - pt2.y) / (double)(pt1.x - pt2.x);
		houghLinesInfo.leftLineSlope = slope;
		//cv::line(frame, pt1, pt2, cv::Scalar(0), 1);
	}
}

cv::Vec2f LaneDetector::getInterceptAndSlope(cv::Vec2f houghLine) {
	float rho = houghLine[0];
	float theta = houghLine[1];
	cv::Point pt1;
	cv::Point pt2;
	cv::Vec2f info;
	info[0] = 0.0; //xIntercept
	info[1] = 0.0; // slope;
	//vector<cv::Point> samplePoints;

	// Point 구하기
	if (theta < CV_PI / 4. || theta > 3. * CV_PI / 4.) {
		pt1 = cv::Point(rho / cos(theta), 0);
		pt2 = cv::Point((rho - ipImage.rows * sin(theta)) / cos(theta), ipImage.rows);
	}
	else {
		pt1 = cv::Point(0, rho / sin(theta));
		pt2 = cv::Point(ipImage.cols, (rho - ipImage.cols * cos(theta)) / sin(theta));
	}

	if ((pt1.x - pt2.x) != 0) {
		info[1] = (float)(pt1.y - pt2.y) / (pt1.x - pt2.x);
		float yIntercept = pt1.y - (info[1] * pt1.x);
		info[0] = (ipImage.rows - yIntercept) / info[1];
	}
	else {
		info[0] = 0;
	}

	return info;
}

void LaneDetector::setSamplePoints(vector<cv::Vec2f>& lanes, cv::Mat frame, vector<cv::Point>& searchWindowPoint) {
	vector<cv::Vec2f>::const_iterator it = lanes.begin();
	int laneCount = 0;

	while (it != lanes.end()) {
		laneCount++;
		float rho = (*it)[0];
		float theta = (*it)[1];
		//vector<cv::Point> samplePoints;

		// hough line 그리기 => 실전에서는 주석 처리 할 것
		if (theta < CV_PI / 4. || theta > 3. * CV_PI / 4.) {
			cv::Point pt1(rho / cos(theta), 0);
			cv::Point pt2((rho - frame.rows * sin(theta)) / cos(theta), frame.rows);
			//cv::line(frame, pt1, pt2, cv::Scalar(0, 0, 255), 1);
		}
		else {
			cv::Point pt1(0, rho / sin(theta));
			cv::Point pt2(frame.cols, (rho - frame.cols * cos(theta)) / sin(theta));
			//cv::line(frame, pt1, pt2, cv::Scalar(0, 0, 255), 4);
		}
		//imshow("left hough", frame);

		//cv::cvtColor(frame, frame, CV_BGR2GRAY);
		//cout << "test SearchWindowNum: " << searchWindowNum << endl;
		if (sin(theta) != 0 && cos(theta) != 0) {
			float slope = -1 * cos(theta) / sin(theta);
			float yIntercept = rho / sin(theta);
			for (int i = 0; i < samplePointNum; i++) {
				int y = i * SEARCH_WINDOW_INTERVAL;
				int x = (int)((rho / sin(theta) - y) * (sin(theta) / cos(theta))) - searchWindowLength;
				searchWindowPoint.push_back(cv::Point(x, y));
			}
		}
		it++;
	}
}


// https://gist.github.com/kchapelier/b1fd7e71f5378b871e3d6daa5ae193dc 참고
double* LaneDetector::setGaussianKernel(int w, double sigma) {
	matchFilterWidth = w;
	double sqr2pi = sqrt(2 * M_PI);
	int width = w / 2;
	double * gaussianArray;
	gaussianArray = (double *)malloc(sizeof(double) * (width * 2) + 1);
	double norm = 1.0 / (sqr2pi * sigma);
	double coefficient = 2 * sigma * sigma;
	double total = 0;

	for (int x = -1 * width; x <= width; x++) {
		gaussianArray[width + x] = norm * exp(-x * x / coefficient);
		total += gaussianArray[width + x];
	}

	for (int i = 0; i < width * 2; i++) {
		gaussianArray[i] = gaussianArray[i] / total;
	}
	return gaussianArray;
}

void LaneDetector::matchGaussianFilter(vector<cv::Point> &searchWindowPoint, vector<cv::Point>& laneMarkerPoints, cv::Mat frame) {

	vector<Convolution> convolutionResultVec;
	int samplePoitHomogeneousLevel = searchWindowPoint.size() / samplePointNum;

	for (int windowIndex = 0; windowIndex < searchWindowPoint.size(); windowIndex++) {

		int hight = searchWindowPoint.at(windowIndex).y;
		vector<Convolution> convolutionInfoVec;


		// 특정 y 축위에서 match filter와 convolution시킴
		for (int filterIndex = 0; filterIndex < searchWindowLength * 2 - matchFilterWidth; filterIndex += 2) {

			Convolution convolutionInfo;
			double convolutionValue = 0;
			int startIndex = searchWindowPoint.at(windowIndex).x + filterIndex;

			//cout << "===========================" << endl;
			for (int i = 0; i < matchFilterWidth; i++) {
				//cout << "test gaussian: " << gaussianKernel[i] << ", ";
				if (startIndex + i < frame.cols && hight < frame.rows) {
					int intensity = cv::sum(frame.at<uchar>(cv::Point(startIndex + i, hight)))[0];
					convolutionValue += gaussianKernel[i] * intensity;
					//cout << "test intensity: " << intensity << endl;

					//cout << "test convolution Result: " << convolutionValue << endl;
				}
			}

			convolutionInfo.convolutionValue = convolutionValue;
			convolutionInfo.point = cv::Point(startIndex + matchFilterWidth / 2, hight);
			convolutionInfoVec.push_back(convolutionInfo);

		}

		// convolution 값을 기준으로 하여 오름 차순으로 정렬
		sort(convolutionInfoVec.begin(), convolutionInfoVec.end(), [](const Convolution& lhs, const Convolution& rhs) {
			return lhs.convolutionValue < rhs.convolutionValue;
		});

		convolutionResultVec.push_back(convolutionInfoVec.at(convolutionInfoVec.size() - 1)); // search window 상에서 가장 큰 convolution 값을 갖는 value를 push함.
		//cout << "test1: " << convolutionInfoVec.at(0).convolutionValue << endl;
		//cout << "test2: " << convolutionInfoVec.at(convolutionInfoVec.size() - 1).convolutionValue << endl;
	}

	//cout << "test convolutionResultVec size: " << convolutionResultVec.size() << endl;
	// convolutionResultVec를 같은 레벨(y축 hight 기준)에서 sorting하기
	for (int i = 0; i < samplePointNum; i++) {
		vector<Convolution> temp;
		for (int k = 0; k < samplePoitHomogeneousLevel; k++) {
			//i + (k * samplePointNum)
			temp.push_back(convolutionResultVec.at(i + (k * samplePointNum)));
		}
		sort(temp.begin(), temp.end(), [](const Convolution& lhs, const Convolution& rhs) {
			return lhs.convolutionValue < rhs.convolutionValue;
		});
		if (temp.at(temp.size() - 1).convolutionValue >50){ 
		//cout << "test Convolutino in CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC" << endl;
		laneMarkerPoints.push_back(temp.at(temp.size() - 1).point);}
	}
	//cout << "laneMarkerPoints.size()" << laneMarkerPoints.size() << endl;
}

void LaneDetector::fitsLine(cv::Mat temp, int option) {
	cv::Vec4f line;
	float laneInterval; // 검출된 양쪽 차선간 간격

	if (option == RIGHT_OPTION) {
		cv::fitLine(rightLaneMarkerPoints, line,
			CV_DIST_L2, // 거리 유형
			0,   // L2 거리를 사용하지 않음
			0.01, 0.01); // 정확도

		// 단위 방향 벡터(cv::Vec4f의 첫 두개 값), 
		// 선에 놓인 한 점의 좌표(cv::Vec4f의 마지막 두 값) 형태인 선 방정식의 파라미터를 제공
		// 마지막 두 파라미터는 선 파라미터에 대한 요구 정확도를 지정
		// 함수에서 요구 하는 std::vector 내에 포함된 입력 점은 cv::Mat로 전달

						 // 선 방정식은 일부 속성계산에 사용
						 // 올바른 선을 계산하는지 확인하기 위해 영상에 예상 선을 그림
						 // 200화소 길이와 3화소 두께를 갖는 임의의 검은 세그먼트를 그림
	}
	else if (option == LEFT_OPTION) {
		cv::fitLine(leftLaneMarkerPoints, line,
			CV_DIST_L2, // 거리 유형
			0,   // L2 거리를 사용하지 않음
			0.01, 0.01); // 정확도
	}
	else if (option == TOTAL_OPTION) {
		cv::fitLine(totalLaneMarkerPoints, line,
			CV_DIST_L2, // 거리 유형
			0,   // L2 거리를 사용하지 않음
			0.01, 0.01); // 정확도
	}


	int x0 = line[2]; // 선에 놓은 한 점
	int y0 = line[3];

	//circle(temp, cv::Point(x0, y0), 3, cv::Scalar(0, 255, 0), 2, 8);

	double slope = 1 * line[1] / line[0]; // 기울기
	int upperX = (-1 * y0) / slope + x0;
	int upperY = 0;
	int lowerX = (temp.rows - y0) / slope + x0;
	int lowerY = temp.rows;
	if (upperX < 0) upperX = 0;
	if (lowerX < 0) lowerX = 0;



	if (option == LEFT_OPTION) {
		isLeftLaneUpdated = true;
		//cout << "left in " << endl;
		ipLeftLaneSlope = slope;
		//cout << "left slope: " << slope << endl;
		topLeftPoint1 = cv::Point(upperX + LEFT_ROI_X, upperY);
		topLeftPoint2 = cv::Point(lowerX + LEFT_ROI_X, lowerY);
		

		//cv::circle(temp, leftPoint1, 3, cv::Scalar(0, 255, 0), 1);
		//cv::circle(temp, leftPoint2, 3, cv::Scalar(0, 255, 0), 1);
		//imshow("test points in fit line", temp);

		//https://stackoverflow.com/questions/17852182/runnig-cvwarpperspective-on-pointswarpPerspective on Point

		vector<cv::Point2f> dstPoints, srcPoints;
		dstPoints.push_back(topLeftPoint1);
		dstPoints.push_back(topLeftPoint2);

		cv::perspectiveTransform(dstPoints, srcPoints, transformMatrix.inv());
		//cout << "dstPoints" << dstPoints << endl;
		//cout << "srdPoints" << srcPoints << endl;

		leftLanePoints[0] = cv::Point((srcPoints.at(0).x + normalViewOffsetX), (srcPoints.at(0).y + normalViewOffsetY));
		leftLanePoints[1] = cv::Point((srcPoints.at(1).x + normalViewOffsetX),(srcPoints.at(1).y + normalViewOffsetY));

		//cv::circle(srcImage, srcPoints.at(0), 3, cv::Scalar(255), 1);
		//cv::circle(srcImage, srcPoints.at(1), 3, cv::Scalar(255), 1);

		//cv::line(originalFrame2, leftPointRecord[0], leftPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
		//imshow("Lane detection result", originalFrame2);
	}
	else if (option == RIGHT_OPTION) {
		isRightLaneUpdated = true;
		//cout << "right in~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ " << endl;
		ipRightLaneSlope = slope;
		topRightPoint1 = cv::Point(upperX + MID_ROI_X + RIGHT_MID_OFFSET, upperY);
		//cout <<  "test3 topRightPOint1 >> " << topRightPoint1 << endl;
		topRightPoint2 = cv::Point(lowerX + MID_ROI_X + RIGHT_MID_OFFSET, lowerY);
		

		vector<cv::Point2f> dstPoints, srcPoints;
		dstPoints.push_back(topRightPoint1);
		dstPoints.push_back(topRightPoint2);

		cv::perspectiveTransform(dstPoints, srcPoints, transformMatrix.inv());
		//cout << "dstPoints" << dstPoints << endl;
		//cout << "srdPoints" << srcPoints << endl;

		rightLanePoints[0] = cv::Point((srcPoints.at(0).x + normalViewOffsetX),(srcPoints.at(0).y + normalViewOffsetY));
		//cout << "test3 - rightLanePoints[0] >>>>>" << rightLanePoints[0] << endl;
		rightLanePoints[1] = cv::Point((srcPoints.at(1).x + normalViewOffsetX),(srcPoints.at(1).y + normalViewOffsetY));

		//cv::circle(normalViewOriginalFrame, srcPoints.at(0), 3, cv::Scalar(255), 1);
		//cv::circle(normalViewOriginalFrame, srcPoints.at(1), 3, cv::Scalar(255), 1);


		//cv::line(originalFrame2, rightPointRecord[0], rightPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
		//imshow("Lane detection result", originalFrame2);
	}
	else if (option == TOTAL_OPTION) {
		topTotalPoint1 = cv::Point(upperX + LEFT_ROI_X, upperY);
		topTotalPoint2 = cv::Point(lowerX + LEFT_ROI_X, lowerY);

		vector<cv::Point2f> dstPoints, srcPoints;
		dstPoints.push_back(topTotalPoint1);
		dstPoints.push_back(topTotalPoint2);

		cv::perspectiveTransform(dstPoints, srcPoints, transformMatrix.inv());
		//cout << "dstPoints" << dstPoints << endl;
		//cout << "srdPoints" << srcPoints << endl;

		totalLanePoints[0] = cv::Point((srcPoints.at(0).x + normalViewOffsetX),(srcPoints.at(0).y + normalViewOffsetY));
		totalLanePoints[1] = cv::Point((srcPoints.at(1).x + normalViewOffsetX),(srcPoints.at(1).y + normalViewOffsetY));

		//cv::circle(normalViewOriginalFrame, srcPoints.at(0), 3, cv::Scalar(255), 1);
		//cv::circle(normalViewOriginalFrame, srcPoints.at(1), 3, cv::Scalar(255), 1);

		//totalPointRecord[0] = totalLanePoints[0];
		//totalPointRecord[1] = totalLanePoints[1];

		//cv::line(normalViewOriginalFrame, totalPointRecord[0], totalPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
		//imshow("Lane detection result!!!", normalViewOriginalFrame);
	}

	//int x1 = x0 - 15 * line[0]; // 200 길이를 갖는 벡터 추가
	//int y1 = y0 - 15 * line[1]; // 단위 벡터 사용
	//cv::line(temp, p1, p2, cv::Scalar(255, 255, 0), 3);
	//cv::line(temp, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 5);

	//cv::namedWindow("Estimated line");
	//cv::imshow("Estimated line", temp);
}

void LaneDetector::setOriginalFrame(cv::Mat frame) {
	normalViewOriginalFrame = frame;
	isLeftLaneUpdated = false;
	isRightLaneUpdated = false;
}

cv::Mat LaneDetector::getResultFrame(void){
	return normalViewOriginalFrame;
}

cv::Mat LaneDetector::getHoughClusterFrame(void){
	return ipImage;
}

// 참고: https://stackoverflow.com/questions/36805703/poly-opencv-semi-transperent
void LaneDetector::drawLaneDetectResult(int option) {
	
	if (option == SPLIT_ROI_OPTION) {
	//cout << "split" << endl;
	laneInfoUpdate(SPLIT_ROI_OPTION);
		vector<cv::Point> vertices{
			leftPointRecord[0], rightPointRecord[0], rightPointRecord[1], leftPointRecord[1]


			//cv::Point(leftLanePoints[0], leftLanePoints[1]),
			//cv::Point(rightLanePoints[0], rightLanePoints[1]) ,
			//cv::Point(rightLanePoints[2], leftLanePoints[3]),
			//cv::Point(leftLanePoints[2], leftLanePoints[3])
		};
		cout << "leftPointRecord[0]: " << leftPointRecord[0] << endl;
		cout << "leftPointRecord[1]: " << leftPointRecord[1] << endl;
		cout << "rightPointRecord[0]: " << rightPointRecord[0] << endl;
		cout << "rightPointRecord[1]: " << rightPointRecord[1] << endl;
		
		cv::line(normalViewOriginalFrame, leftPointRecord[0], leftPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
		cv::line(normalViewOriginalFrame, rightPointRecord[0], rightPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
		vector<vector<cv::Point> > pts{ vertices };
		//cv::cvtColor(originalFrame2, originalFrame2, CV_BGR2BGRA);
		//fillPoly(originalFrame2, pts, cv::Scalar(0, 255, 0, 0));


		cv::Mat polyImage = cv::Mat(normalViewOriginalFrame.rows, normalViewOriginalFrame.cols, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::fillPoly(polyImage, pts, CV_RGB(0, 200, 10));

		float transFactor = 0.7f;

		for (int y = 0; y < normalViewOriginalFrame.rows; y++) {
			for (int x = 0; x < normalViewOriginalFrame.cols; x++) {
				if (polyImage.at<cv::Vec3b>(y, x) != cv::Vec3b(0, 0, 0)) {
					normalViewOriginalFrame.at<cv::Vec3b>(y, x) = (transFactor)*normalViewOriginalFrame.at<cv::Vec3b>(y, x) + (1.0f - transFactor)*polyImage.at<cv::Vec3b>(y, x);
					//cout << "transparent in " << endl;
				}
			}
		}
	} else if (option == NON_SPLIT_ROI_OPTION) {
		laneInfoUpdate(NON_SPLIT_ROI_OPTION);
		cv::line(normalViewOriginalFrame, totalPointRecord[0], totalPointRecord[1], cv::Scalar(255, 0, 0), 4, 8);
	} else if(option == SPLIT_LEFT_ROI_OPTION){
		laneInfoUpdate(SPLIT_LEFT_ROI_OPTION);
		cv::line(normalViewOriginalFrame, leftPointRecord[0], leftPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
	} else if(option == SPLIT_RIGHT_ROI_OPTION){
		laneInfoUpdate(SPLIT_RIGHT_ROI_OPTION);
		cv::line(normalViewOriginalFrame, rightPointRecord[0], rightPointRecord[1], cv::Scalar(20, 200, 50), 4, 8);
	}



	cv::line(normalViewOriginalFrame, cv::Point(cameraCenterX, normalViewOriginalFrame.rows), cv::Point(cameraCenterX, 0), cv::Scalar(20, 200, 50), 4, 8);
	cv::line(normalViewOriginalFrame, cv::Point(0, vanishingLineHight), cv::Point(normalViewOriginalFrame.cols, vanishingLineHight), cv::Scalar(20, 200, 50), 4, 8);
	//imshow("Lane detection Result", normalViewOriginalFrame);
}



void LaneDetector::laneInfoUpdate(int option) {
	// 직선도로
	laneInfo.setIsCurvedDirection(-1);
	if (option == SPLIT_ROI_OPTION) {
		if (((leftPointRecord[0].x - leftPointRecord[1].x) != 0) && ((rightPointRecord[0].x - rightPointRecord[1].x) != 0)) {			
			float leftLineSlope = ((float)leftPointRecord[0].y - (float)leftPointRecord[1].y) / ((float)leftPointRecord[0].x - (float)leftPointRecord[1].x);
			float leftLineYIntercept = (float)leftPointRecord[0].y - (float)leftLineSlope * (float)leftPointRecord[0].x;
			float rightLineSlope = ((float)rightPointRecord[0].y - (float)rightPointRecord[1].y) / ((float)rightPointRecord[0].x - (float)rightPointRecord[1].x);
			float rightLineYIntercept = (float)rightPointRecord[0].y - (float)rightLineSlope * (float)rightPointRecord[0].x;
			if (leftLineSlope != rightLineSlope) {
				laneInfo.setleftBottomPoint(leftPointRecord[1]);
				laneInfo.setRightBottomPoint(rightPointRecord[1]);

				laneInfo.setIsCurvedRoad(false);
				laneInfo.setIsOnlyOneLaneDetectedR(false);
				laneInfo.setIsOnlyOneLaneDetectedL(false, false);

				float intersectX = (leftLineYIntercept - rightLineYIntercept) / (rightLineSlope - leftLineSlope);
				float intersectY = leftLineSlope * intersectX + leftLineYIntercept;
				//cout << "test!!!!! Xxxx : " << intersectX << endl;
				laneInfo.setInterceptionPoint(cv::Point((int)intersectX,(int) intersectY));
				circle(normalViewOriginalFrame, laneInfo.getInterceptionPoint(), 5, cv::Scalar(0, 0, 255), 2, 4);
			}
		}
		//else cout << "~~~~~~~~~~~~~~~ else!!!!!!!!!11" << endl;
	}
	// 급커브도로
	else if (option == NON_SPLIT_ROI_OPTION){ // option == NON_SPLIT_ROI_OPTION
		if ((totalPointRecord[0].x - totalPointRecord[1].x) != 0) {	
			float totalLineSlope = ((float)totalPointRecord[0].y - (float)totalPointRecord[1].y) / ((float)totalPointRecord[0].x - (float)totalPointRecord[1].x);
			float totalLineYIntercept = (float)totalPointRecord[0].y - (float)totalLineSlope *(float)totalPointRecord[0].x;
			if (totalLineSlope != 0) {
				laneInfo.setScarpBottomPoint(totalPointRecord[1]);
				//centerbottomline logic
				//if(-1.4 <= totalLineSlope && totalLineSlope <=0) laneInfo.setIsCurvedRoad(true, 0);
				if(-1.6 <= totalLineSlope && totalLineSlope <=0) laneInfo.setIsCurvedRoad(true, 0);
				else if(0 <= totalLineSlope && totalLineSlope <= 1.6) laneInfo.setIsCurvedRoad(true, 1);
				laneInfo.setIsOnlyOneLaneDetectedR(false);
				laneInfo.setIsOnlyOneLaneDetectedL(false, false);

				float intersectX = (vanishingLineHight - totalLineYIntercept) / totalLineSlope;
				laneInfo.setInterceptionPoint(cv::Point(intersectX, vanishingLineHight));
				circle(normalViewOriginalFrame, laneInfo.getInterceptionPoint(), 5, cv::Scalar(0, 0, 255), 2, 4);
			}
		}
	} else if(option == SPLIT_LEFT_ROI_OPTION){
		if ((leftPointRecord[0].x - leftPointRecord[1].x) != 0) {	
			float leftLineSlope = ((float)leftPointRecord[0].y - (float)leftPointRecord[1].y) / ((float)leftPointRecord[0].x - (float)leftPointRecord[1].x);
			float leftLineYIntercept = (float)leftPointRecord[0].y - (float)leftLineSlope *(float)leftPointRecord[0].x;
			if (leftLineSlope != 0) {
				laneInfo.setLeftBottomPointOneLane(leftPointRecord[1]);
				
				laneInfo.setIsCurvedRoad(false);
								
				if(leftLineSlope >= 0.5 && leftLineSlope <= 1){
					laneInfo.setIsOnlyOneLaneDetectedR(false);
					laneInfo.setIsOnlyOneLaneDetectedL(true, true);
				} else{									
					laneInfo.setIsOnlyOneLaneDetectedR(false);
					laneInfo.setIsOnlyOneLaneDetectedL(true, false);
				}


				float intersectX = (vanishingLineHight - leftLineYIntercept) / leftLineSlope;
				laneInfo.setInterceptionPoint(cv::Point(intersectX, vanishingLineHight));
				circle(normalViewOriginalFrame, laneInfo.getInterceptionPoint(), 5, cv::Scalar(0, 0, 255), 2, 4);
			}
		}
	} else if(option == SPLIT_RIGHT_ROI_OPTION){
		if ((rightPointRecord[0].x - rightPointRecord[1].x) != 0) {	
			float rightLineSlope = ((float)rightPointRecord[0].y - (float)rightPointRecord[1].y) / ((float)rightPointRecord[0].x - (float)rightPointRecord[1].x);
			float rightLineYIntercept = (float)rightPointRecord[0].y - (float)rightLineSlope *(float)rightPointRecord[0].x;
			if (rightLineSlope != 0) {
				laneInfo.setRightBottomPointOneLane(rightPointRecord[1]);

				laneInfo.setIsCurvedRoad(false);
				laneInfo.setIsOnlyOneLaneDetectedR(true);
				laneInfo.setIsOnlyOneLaneDetectedL(false, false);

				float intersectX = (vanishingLineHight - rightLineYIntercept) / rightLineSlope;
				laneInfo.setInterceptionPoint(cv::Point(intersectX, vanishingLineHight));
				circle(normalViewOriginalFrame, laneInfo.getInterceptionPoint(), 5, cv::Scalar(0, 0, 255), 2, 4);
			}
		}
	}
}


LaneInfo LaneDetector::getLaneInfo(void) {
	return laneInfo;
}

cv::Mat LaneDetector::getIPLeftImage(void){
	return ipLeftImage;
}

cv::Mat LaneDetector::getIPRightImage(void){
	return ipRightImage;
}

cv::Mat LaneDetector::getWarpCoordinateTestFrame(void){
	return ROS_Invers_Matrix_Frame;
}

cv::Mat LaneDetector::getLaneMarkerFrameL(void){
	return laneMarkerFrameL;
}

cv::Mat LaneDetector::getLaneMarkerFrameR(void){
	return laneMarkerFrameR;
}

cv::Vec2f LaneDetector::twoPoints2Polar(const cv::Vec4i & line){
	// Get points from the vector
	cv::Point2f p1(line[0], line[1]);
	cv::Point2f p2(line[2], line[3]);
	if((p1.x - p2.x) != 0){
		float polarSlope = -1 / ((p1.y - p2.y) / (p1.x - p2.x));
		float slope = (p1.y - p2.y) / (p1.x - p2.x);
		float yIntercept = p2.y - slope * p2.x;
		float interceptX;
		float interceptY;
		interceptX = yIntercept / (polarSlope - slope);
		interceptY = slope * interceptX + yIntercept;

		float rho = sqrt(interceptX * interceptX + interceptY * interceptY);
		float theta = atan2(interceptY, interceptX);

		if(theta < 0){
			rho = -rho;		
		}
		return cv::Vec2f(rho, theta);
	}
	return cv::Vec2f(0.0, 0.0);
}

void LaneDetector::setLatestUpdateInfo(int option){

	if(option == BOTH_LANE_UPDATE){
		currentLaneUpdateInfo = BOTH_LANE_UPDATE;
	} else if(option == SCARP_LANE_UPDATE){
		currentLaneUpdateInfo = SCARP_LANE_UPDATE;
	} else if(option == LEFT_LANE_UPDATE){
		currentLaneUpdateInfo = LEFT_LANE_UPDATE;
	} else if(option == RIGHT_LANE_UPDATE){
		currentLaneUpdateInfo = RIGHT_LANE_UPDATE;
	}
}



void LaneDetector::clusterTotalLaneMarkerPoints(void){
	if(totalLaneMarkerPoints.size() > 4){
	cout << "IN!!!!!!!!!!!!!!!!!!!: " << totalLaneMarkerPoints.size() << endl;
	cv::Point first = totalLaneMarkerPoints.at(0);
	vector<vector<cv::Point>> cluster;
	vector<cv::Point> tempCluster;
	int i = 1;
	while(i < totalLaneMarkerPoints.size()){	
		tempCluster.push_back(first);
		if(abs(first.x - totalLaneMarkerPoints.at(i).x) < 10){
			first = totalLaneMarkerPoints.at(i);
			if(i ==  totalLaneMarkerPoints.size() -1){
				tempCluster.push_back(first);
				cluster.push_back(tempCluster);
				tempCluster.clear();
			}
			i++;
		} else {
			
			cluster.push_back(tempCluster);
			first = totalLaneMarkerPoints.at(i);
			tempCluster.clear();
			i++;
		}
	}
	cout << "out!!!!!!!!!!!! " << endl;
	sort(cluster.begin(), cluster.end(), [](const vector<cv::Point> & lhs, const vector<cv::Point> & rhs) {
		return lhs.size() < rhs.size();
	});
		for(int j = 0 ; j < cluster.size(); j++){
		cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : " << cluster.at(j).size() << endl;
	}
	/*
	cout <<"===================================== "<< endl;
	for(int k = 0 ; k < cluster.at(cluster.size() -1).size(); k++){
		cout << cluster.at(cluster.size()-1).at(k) << endl;
	}
	cout <<"=====================================" << endl;
	*/
		totalLaneMarkerPoints.clear();
		totalLaneMarkerPoints = cluster.at(cluster.size() -1);
	}

}

float LaneDetector::getHoughlineCluterSlope(void){
	return houghLinesInfo.leftLineSlope;
}

void LaneDetector::setLaneInfoFutureScarp(bool value){
	laneInfo.setIsFutureScarp(value);
}

void LaneDetector::setStartLineCount(int count){
	laneInfo.setcountStartLine(count);
}

//

