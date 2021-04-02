#include "patternCheck.h"

void PatternCheck::updatePatternCheckFrame(cv::Mat infoFrame){
	frame = infoFrame;
}


//pattern_check

void PatternCheck::checkPattern(bool isCheck) {
	frameNum++;

	//defining bounding rectangle
	//vector<Rect> boundRect(contours_video.size());
	//for (int i = 0; i < contours_video.size(); i++) {
	//	boundRect[i] = boundingRect(Mat(contours_video[i]));
	//}

	//draw rectangles on the contours
	//for (int i = 0; i < contours_video.size(); i++) {
	//	rectangle(roi, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 255), 2, 8, 0);
	//}
//	patternProcessedFrame = frame;
	if (true) {
		cv::Mat grayImage, blurImage;
		cv::Mat inRangeImage;
	
		cv::cvtColor(frame, grayImage, CV_RGB2GRAY);
			// testtttttttttttttttttttttttt
		patternProcessedFrame = grayImage;

		//cv::inRange(grayImage, 100, 255, inRangeImage); // pattern.mp4, kukmin_track.mp4 = 230~255
			cv::adaptiveThreshold(grayImage, inRangeImage, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 55, -10);
			cv::Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
			
			cv::morphologyEx(inRangeImage, inRangeImage, MORPH_OPEN, element); //white area gets bigger
			
			cv::erode(inRangeImage, inRangeImage, cv::Mat(), cv::Point(-1, -1), 6);
			cv::dilate(inRangeImage, inRangeImage, cv::Mat(), cv::Point(-1, -1), 12);							
		//cv::erode(inRangeImage, inRangeImage, cv::Mat(), cv::Point(-1, -1), 6);
		//cv::dilate(inRangeImage, inRangeImage, cv::Mat(), cv::Point(-1, -1), 4);
		//imshow("afterErode", inRangeImage);
		cv::blur(inRangeImage, blurImage, cv::Size(3, 3));
		//imshow("blurImage", blurImage);
		rosPatternFrame = blurImage;	
		vector<vector<cv::Point>> contours_video;
		vector<cv::Vec4i> hierarchy;

		//finding contours
		cv::findContours(inRangeImage, contours_video, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);		
		vector<cv::Rect> boundRect;
		rosPatternFrame = inRangeImage;

		int countSquare = 0;
		for (unsigned int i = 0; i < contours_video.size(); i++)
		{
			boundRect.push_back(cv::boundingRect(cv::Mat(contours_video[i])));	

			if (boundRect[i].width > 200) {
					//boundRect[i] = boundingRect(Mat(contours_video[i]));

					//approxPolyDP(contours_video[i], poly, 1, true);
					//if (poly.size() == 4) {
						//countSquare++;
				//rectangle(rosPatternFrame, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 255), 2, 8, 0); // send
				rectPoints.push_back(boundRect[i].tl());	
				rectPoints.push_back(boundRect[i].br());
				if (frame_startLine < frameNum) {
					isStartLine = true;
				}

				if (isStartLine && isCheck) {
					countStartLine++;
					isStartLine = false;
					frame_startLine = frameNum + 100;
				}
						

					//}
			}
			else {
				rectPoints.push_back(cv::Point(0, 0));
				rectPoints.push_back(cv::Point(1, 1));			
			}
		}
	
				rectPoints.push_back(cv::Point(0, 0));
				rectPoints.push_back(cv::Point(1, 1));	
	}
	

}



cv::Mat PatternCheck::getRosPatternFrame(void){
	return rosPatternFrame;
}

cv::Mat PatternCheck::getPatternProcessedFrame(void){
	return patternProcessedFrame;
}

int PatternCheck::getCountStartLine(void){
	return countStartLine;
}

vector<cv::Point> PatternCheck::getRectPoints(void){
	return rectPoints;
}
