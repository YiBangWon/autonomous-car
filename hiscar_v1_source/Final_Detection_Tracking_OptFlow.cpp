//#include <opencv.hpp>
//#include <iostream>
//#include <string.h>
//
//#define BLACK Scalar(0, 0, 0)
//#define RED Scalar(0, 0, 255)
//#define GREEN Scalar(0, 255, 0)
//#define BLUE Scalar(255, 0, 0)
//#define AQUA Scalar(255, 255, 0)
//#define PURPLE Scalar(155, 0, 155)
//#define YELLOW Scalar(0,255,255)
//
//#define ToRadian( degree )  ( (degree) * (CV_PI / 180.0f) )
//#define ToDegree( radian )  ( (radian) * (180.0f / CV_PI) )
//
//using namespace cv;
//using namespace std;
//
//Rect detectedROI;
//Point tl, tr, bl, br, center, intersection;
//Mat currFrame;
//char labelName;
//
//void warning(Mat frame, int y, int bonnet);
//int detection(Mat gray, Rect roi_rect, Ptr<ml::SVM> svm, HOGDescriptor d);
//void drawLane(Mat frame);
//int searchROI(Mat gray, Ptr<ml::SVM> svm, HOGDescriptor d, Mat frame, int y);
//void tracking_OptFlow(Mat frame, Rect detectedROI, int undetected_cnt);
//
//int main() {
//	Ptr<ml::SVM> svm = ml::SVM::create();
//	svm = Algorithm::load<ml::SVM>("0730_2017_3304_RBF_1e5_1e-6.xml");
//	//svm = Algorithm::load<ml::SVM>("Final_SVM_JK.xml");
//
//	HOGDescriptor d(Size(64, 64), Size(32, 32), Size(16, 16), Size(16, 16), 9);
//
//	//VideoCapture cap("C:/Users/smartcar/Desktop/JAEKYUNG/초기데이터/영상/front1542_1551.mpg");
//	VideoCapture cap("front1542_1551.mpg");
//	if (!cap.isOpened()) {
//		cout << "Error initializing video capture" << endl;
//		return -1;
//	}
//
//	Mat frame, gray;
//
//	int fps = cap.get(cv::CAP_PROP_FPS);
//	cout << "fps = " << fps << endl;
//	int Frames = cap.get(CAP_PROP_FRAME_COUNT);
//	cout << "Total frames = " << Frames << endl;
//
//	int frame_rate = 10, frameNum = 0;
//	int detected_cnt = 0, undetected_cnt = 0;
//	bool tracking = false;
//
//	intersection = Point(687, 418);
//
//	while (cap.read(frame)) {
//		cap >> frame;
//		cvtColor(frame, gray, CV_RGB2GRAY);
//		currFrame = frame.clone();
//
//		drawLane(currFrame);
//		int y_coordinate = intersection.y;
//
//		/* search ROI smaller to bigger */
//
//		//for (int y = y_coordinate; y < frame.rows - 80; y += 20) {
//		//   detected_cnt = searchROI(gray, svm, d, frame, y);
//		//   if (detected_cnt > 0) {
//		//      break;
//		//   }
//		//}
//
//		/* search ROI Bigger to smaller */
//		int bonnet = frame.rows - 80;
//		for (int y = bonnet; y > y_coordinate; y -= 20) {
//			detected_cnt = searchROI(gray, svm, d, frame, y);
//
//			if (detected_cnt > 0) {
//				warning(currFrame, y, bonnet);
//				tracking = true; // it means detection is started 
//				undetected_cnt = 0;
//				break;
//			}
//		}
//		imshow("detected", currFrame);
//
//		if (detected_cnt == 0 && tracking == true) {
//			tracking_OptFlow(frame, detectedROI, ++undetected_cnt);
//		}
//
//
//		// key
//		char key = waitKey(frame_rate);
//		if (key == 32) { // space bar -> stop
//			if (frame_rate == 10)
//				frame_rate = 0;
//			else
//				frame_rate = 10;
//		}
//		else if (key == ']') {
//			cap.set(CV_CAP_PROP_POS_FRAMES, frameNum + 90);
//			frameNum += 90;
//		}
//		else if (key == '[') {
//			cap.set(CV_CAP_PROP_POS_FRAMES, frameNum - 10);
//			frameNum -= 90;
//		}
//		else if (key == 27) {
//			break;
//		}
//		frameNum++;
//	}
//
//	waitKey(0);
//	return 0;
//}
//
//void warning(Mat frame, int y, int bonnet) {
//	int distance;
//	char alarm[10];
//
//	distance = bonnet - y;
//	if (distance <= 100) strcat(alarm, "Dangerous");
//	else strcat(alarm, "Safe");
//
//	putText(frame, format("Distance : %2d, %5s", distance, alarm), Point(20, 50), 0, 1, Scalar(0, 0, 255), 2);
//}
//
//int detection(Mat gray, Rect roi_rect, Ptr<ml::SVM> svm, HOGDescriptor d) {
//	int result;
//	vector< float> descriptorsValues;
//	Mat roi = gray(roi_rect);
//	imshow("currentROI", roi);
//	resize(roi, roi, Size(64, 64));
//
//	d.compute(roi, descriptorsValues);
//	Mat fm = Mat(descriptorsValues);
//	fm = fm.reshape(0, 1);
//	fm.convertTo(fm, CV_32F);
//	result = svm->predict(fm);
//
//	return result;
//}
//
//void drawLane(Mat frame) {
//	// lane points
//	tl = Point(630, 460); // top left
//	tr = Point(730, 460); // top right
//	bl = Point(390, 700); // bottom left
//	br = Point(970, 700); // bottom right
//
//	circle(frame, bl, 4, AQUA, 2);
//	circle(frame, br, 4, AQUA, 2);
//	circle(frame, intersection, 3, RED, -1, CV_AA);
//
//	// draw left and right lane
//	line(frame, bl, intersection, PURPLE, 1);
//	line(frame, br, intersection, PURPLE, 1);
//
//	imshow("detected", frame);
//}
//
//int searchROI(Mat gray, Ptr<ml::SVM> svm, HOGDescriptor d, Mat frame, int y) {
//	Rect currentROI;
//	int currentResult = 0, cnt = 0;
//	// 양쪽 차선의 기울기
//	int l_slope, r_slope;
//	//HEE BEOM contributes
//	r_slope = abs(tl.y - bl.y) / abs(tl.x - bl.x);
//	l_slope = -1 * abs(tr.y - br.y) / abs(tr.x - br.x);
//
//	int intersection_x, intersection_y;
//	intersection_x = (tr.y - tl.y - (r_slope * tr.x) + (l_slope * tl.x)) / (l_slope - r_slope);
//	intersection_y = l_slope * (intersection_x - tl.x) + tl.y;
//	intersection = Point(intersection_x, intersection_y); // 소실점
//
//	int l_x = (y - tl.y) / l_slope + tl.x;
//	int r_x = (y - tr.y) / r_slope + tr.x;
//	// roiSize는 현재 y 좌표 높이에서의 차선 간격의 0.6배.
//	int roiSize = abs(r_x - l_x) / 10 * 6;
//	int roiXL = l_x;
//	int roiXR = r_x - roiSize;
//	int roiy = y - roiSize;
//	int x_Sum = 0, car = 0, truck = 0, bus = 0;
//
//	for (int roix = roiXL; roix < roiXR; roix += 10) {
//		currentROI = Rect(roix, roiy, roiSize, roiSize);
//		currentResult = detection(gray, currentROI, svm, d);
//
//		if (currentResult != -1) {
//			x_Sum += roix;
//			cnt++;
//			switch (currentResult) {
//			case 1: // Car - Red color
//			{
//				car++; // 'Car'로 인식된 횟수
//				//rectangle(currFrame, currentROI, RED, 1, 8, 0);
//				break;
//			}
//			//case 2: // Truck - Green color
//			//{
//			//   truck++; // 'Truck'로 인식된 횟수
//			//   rectangle(currFrame, currentROI, GREEN, 1, 8, 0);
//			//   break;
//			//}
//			//case 3: // Bus - Blue color
//			//{
//			//   bus++; // 'Bus'로 인식된 횟수
//			//   rectangle(currFrame, currentROI, BLUE, 1, 8, 0);
//			//   break;
//			//}
//			default: break;
//			}
//		}
//	}
//	//imshow("detected", currFrame);
//
//	if (cnt > 0) {
//		labelName = 'C';
//		if (truck >= car) {
//			if (bus >= truck) labelName = 'B';
//			else labelName = 'T';
//		}
//		else if (bus >= car) {
//			labelName = 'B';
//		}
//
//		currentROI.x = x_Sum / cnt;
//		detectedROI = currentROI;
//		Mat curr = frame(currentROI);
//		imshow("detected ROI", curr);
//
//		switch (labelName) {
//		case 'C':
//		{
//			//cout << "Car" << endl;
//			rectangle(currFrame, currentROI, RED, 3, 8, 0);
//			//putText(currFrame, "Car", Point(20, 50), 0, 1, RED, 2);
//			break;
//		}
//		case 'T':
//		{
//			cout << "Truck" << endl;
//			rectangle(currFrame, currentROI, GREEN, 3, 8, 0);
//			putText(currFrame, "Truck", Point(20, 50), 0, 1, GREEN, 2);
//			break;
//		}
//		case 'B':
//		{
//			cout << "Bus" << endl;
//			rectangle(currFrame, currentROI, BLUE, 3, 8, 0);
//			putText(currFrame, "Bus", Point(20, 50), 0, 1, BLUE, 2);
//			break;
//		}
//		}
//	}
//
//	//imshow("detected", currFrame);
//	return cnt;
//}
//
//void tracking_OptFlow(Mat frame, Rect detectedROI, int undetected_cnt) {
//	Mat detectedimg = frame(detectedROI);
//	Mat currgray, prevgray;
//	Mat flow(detectedROI.size(), CV_32FC2);
//	Rect trackROI = detectedROI;
//
//	Point pt1 = Point(detectedROI.x, detectedROI.y);
//	Point pt2 = Point(detectedROI.x + detectedROI.width, detectedROI.y + detectedROI.height);
//	Point ptStart, ptEnd;
//	bool cal_OptFlow = false;
//
//
//	if (undetected_cnt < 100) {
//		cvtColor(detectedimg, currgray, CV_RGB2GRAY);
//
//		if (prevgray.size() != currgray.size() || prevgray.empty())
//			currgray.copyTo(prevgray);
//
//		// traking using optical flow
//		cal_OptFlow = true;
//	}
//	else {
//		cout << "100 frames 이상 인식이 되지 않았습니다." << endl;
//		cal_OptFlow = false;
//		imshow("detected", currFrame);
//	}
//
//
//	if (!prevgray.empty() && cal_OptFlow) {
//
//		calcOpticalFlowFarneback(prevgray, currgray, flow, 0.4, 1, 12, 2, 8, 1.2, 0);
//
//		double cnt = 0, sumLen = 0, sumTheta = 0;
//		double theta;
//
//		for (int y = 0; y < detectedimg.cols; y += 5) {
//			for (int x = 0; x < detectedimg.rows; x += 5) {
//				// get the flow from y, x position * 7 for better visibility
//				const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
//
//				ptStart = Point(x, y);
//				ptEnd = Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y));
//
//				if (pt2.x == 0) theta = CV_PI / 2;
//				else theta = atan2(double(pt2.y - pt1.y), double(pt2.x - pt1.x));
//
//				//draw initial point
//				//circle(detectedimg, Point(x, y), 1, BLACK, -1);
//				//draw line at flow direction
//				//line(detectedimg, ptStart, ptEnd, RED);
//				//length of the arrow
//				double  arrowLen = sqrt(pow((float)(ptStart.y - ptEnd.y), 2) + pow((float)(ptStart.x - ptEnd.x), 2));
//
//				if (arrowLen > 20) {
//					cnt++;
//					sumTheta += theta;
//					sumLen += arrowLen;
//				}
//			}
//		}
//
//
//		if (cnt > 0) {
//			double meanTheta = sumTheta / cnt;
//			double meanLen = sumLen / cnt;
//
//			// to find out where the ROI moved
//			if (meanTheta >= 0) {
//				if (meanTheta > CV_PI / 2) {
//					trackROI.x = detectedROI.x - meanLen * sin(180 - ToDegree(meanTheta));
//					trackROI.y = detectedROI.y - meanLen * cos(180 - ToDegree(meanTheta));
//				}
//				else {
//					trackROI.x = detectedROI.x + meanLen * sin(ToDegree(meanTheta));
//					trackROI.y = detectedROI.y - meanLen * cos(ToDegree(meanTheta));
//				}
//			}
//			else {
//				if (-meanTheta > CV_PI / 2) {
//					trackROI.x = detectedROI.x - meanLen * sin(180 - ToDegree(-meanTheta));
//					trackROI.y = detectedROI.y + meanLen * cos(180 - ToDegree(-meanTheta));
//				}
//				else {
//					trackROI.x = detectedROI.x + meanLen * sin(ToDegree(-meanTheta));
//					trackROI.y = detectedROI.y + meanLen * cos(ToDegree(-meanTheta));
//				}
//			}
//		}
//
//		Mat trackWindow = frame(trackROI);
//		rectangle(currFrame, trackROI, PURPLE, 2);
//		imshow("trackROI", trackWindow);
//
//		//imshow("Optical Flow", originROI);
//		imshow("detected", currFrame);
//		currgray.copyTo(prevgray);
//	}
//	else currgray.copyTo(prevgray);
//}