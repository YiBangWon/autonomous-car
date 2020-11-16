
#include "preprocess.h"
void PreProcess::preprocessElec(vector<cv::Point> point) {
	//processFrame = patternFrame;
	cv::Mat grayFrame = processFrame.clone();
//	cv::Mat grayFrame = patternFrame.clone();	
	cv::Mat processedFrame;
	cv::Mat temp;
	//cv::medianBlur(grayFrame, grayFrame, 5);
	cv::blur(grayFrame, grayFrame, cv::Size(7, 7));
	//cv::blur(grayFrame, grayFrame, cv::Size(7,7));
	cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5, 5), 1.5);
	//cv::blur(grayFrame, grayFrame, cv::Size( 7, 7));
	//cv::bilateralFilter(grayFrame, temp, -1, 5, 5); // ������ ���͸�
	//grayFrame = temp;
	
	cv::adaptiveThreshold(grayFrame, processedFrame, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, -0.3); // not bad ver1
	//cv::adaptiveThreshold(grayFrame, processedFrame, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, -0.5); // not bad ver1
	
	// adaptive size 7//imshow("After adaptive threshold", resultFrame);
	cv::Mat element(5, 5, CV_8U, cv::Scalar(1));
	for (int i = 0; i < 15; i++) {
		cv::erode(processedFrame, processedFrame, cv::Mat());
		cv::dilate(processedFrame, processedFrame, cv::Mat());
	} 
	//imshow("Adaptive thresholded for electric car", processedFrame);
///*
	cv::Rect roi = cv::Rect(point.at(0), point.at(1));
	cv::Mat setZero = processedFrame(roi);
	for(int i = 0; i < setZero.cols; i++){
		for(int j = 0; j < setZero.rows; j++){
			setZero.at<uchar>(j,i) = 0;		
		}
	}		
//*/	
	preprocessedFrame_E = processedFrame;
	//morphologyEx(resultFrame, resultFrame, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 1);
}

void PreProcess::preprocess() {
   //for (int i = 0; i < ROI_NUM; i++) {
   //   // ������ ���� ������ �»�ܰ� ���ϴ� Point
   //   cv::Mat grayFrame = roiFrame[i]; // grayscale ����
   //   cv::Mat blurredGrayFrame; // blur ó���� gray����
   //   cv::Mat sharpFrame; // grayframe�� sharpening�� ����
   //   cv::Mat sharpBlurFrame; // sharpeneing�� ������ blur��Ų

   //   int thresholdValue;
   //   if(i == 4) imshow("gray frame", grayFrame);
   //   // add current frame to frame stack, and if the stack is full call temporalBlur
   //   //if (addToFrameStack(blurredGrayFrame)) {
   //   //   temporalProcessedFrame = temporalBlur(blurredGrayFrame);
   //   //   isTemporalFrameReady = true;
   //   //}
   //   //cout << frameNum << endl;
   //   /*
   //   ������ ���͸�
   //   cv::blur(image, result, cv::Size(5,5)) => �� ȭ���� ���� �簢�� �̿��� ��հ����� ��ġ�Ͽ� ������ �ε巴�� ��
   //   cv::GaussianBlur(image, result, cv::Size(5,5), 1.5)
   //   - �߽� ȭ�ҿ����� �Ÿ��� ���� ����ġ�� ������ mask���
   //   - ���� ���
   //   - Linear operation : does not preserve edges(sigma���� ���� smooothing�� edge �������� ������)
   //   cv::medianBlur(src, dst, int ksize)
   //   - non-linear filter: edge preserving
   //   cv::bilateralFilter(src, dst, int d, double sigmaColor, double sigmaSpace, int borderType = BORDER_DEFAULT) - ����� ����
   //   - �߽� ȭ�ҿ����� �Ÿ��Ӹ� �ƴ϶� �߽�ȭ�ҿ��� ��� ���̵� �����
   //   - edge�� �����ϳ� �ӵ��� �����ٴ� ������ ����
   //   - sigmaColor: Filter sigma in the color space. A larger value of the parameter means that farther colors
   //   within the pixel neighborhood (see sigmaSpace) will be mixed together, resulting in larger areas of semi-equal color.
   //   - sigmaSpace: Filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will
   //   influence each other as long as their colors are close enough (see sigmaColor).
   //   When d>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, d is proportional to sigmaSpace.
   //   - borderType: border mode used to extrapolate pixels outside of the image
   //   - Sigma values: For simplicity, you can set the 2 sigma values to be the same.
   //   If they are small (< 10), the filter will not have much effect, whereas if they are large (> 150),
   //   they will have a very strong effect, making the image look "cartoonish".
   //   - Filter size: Large filters (d > 5) are very slow, so it is recommended to use d=5 for real-time applications,
   //   and perhaps d=9 for offline applications that need heavy noise filtering.
   //   */
   //   //cv::medianBlur(processFrame, temp, 5);
   //   //cv::blur(grayFrame, processFrame , cv::Size(3, 3));
   //   //cv::bilateralFilter(processFrame, temp, -1, 15, 15);
   //   //cv::bilateralFilter(processFrame, temp, -1, 8, 8); // ������ ���͸�
   //   cv::GaussianBlur(grayFrame, blurredGrayFrame, cv::Size(3, 3), 1.5);
   //   //imshow("After smoothing gray Frame", blurredGrayFrame);
   //   cv::addWeighted(grayFrame, 1.5, blurredGrayFrame, -0.8, 0, sharpFrame); // grayframe�� sharpening�Ͽ� sharpFrame�� ����
   //                                                         //cv::blur(sharpFrame, sharpBlurFrame, cv::Size(5, 5));
   //                                                         //cv::bilateralFilter(sharpFrame, sharpBlurFrame, -1, 8, 8); // ������ ���͸�
   //   cv::medianBlur(sharpFrame, sharpBlurFrame, 3);
   //   //get thresholdValue using Otus's method
   //   thresholdValue = getOtsuThreshold(blurredGrayFrame, grayFrame);
   //   cv::threshold(sharpBlurFrame, otsuProcessedFrame, thresholdValue, 255, CV_THRESH_BINARY);
   //   otsuProcessedFrame.copyTo(grayFrame);
   //   if (i == 4) imshow("After otsu thresholding", otsuProcessedFrame);
   //}
   //imshow("Roi Result", processFrame);

   // ������ ���� ������ �»�ܰ� ���ϴ� Point
   cv::Mat grayFrame = processFrame; // grayscale ����
   cv::Mat blurredGrayFrame; // blur ó���� gray����
   cv::Mat sharpFrame; // grayframe�� sharpening�� ����
   cv::Mat sharpBlurFrame; // sharpeneing�� ������ blur��Ų ����
   int thresholdValue;
   //imshow("gray frame", grayFrame);

   cv::GaussianBlur(grayFrame, blurredGrayFrame, cv::Size(3, 3), 1.5);

   // add current frame to frame stack, and if the stack is full call temporalBlur
   if (addToFrameStack(blurredGrayFrame)) {
      temporalProcessedThresholdedFrame = temporalBlur(blurredGrayFrame);
      isTemporalFrameReady = true;
   }

   /*
   ������ ���͸�
   cv::blur(image, result, cv::Size(5,5)) => �� ȭ���� ���� �簢�� �̿��� ��հ����� ��ġ�Ͽ� ������ �ε巴�� ��
   cv::GaussianBlur(image, result, cv::Size(5,5), 1.5)
   - �߽� ȭ�ҿ����� �Ÿ��� ���� ����ġ�� ������ mask���
   - ���� ���
   - Linear operation : does not preserve edges(sigma���� ���� smooothing�� edge �������� ������)
   cv::medianBlur(src, dst, int ksize)
   - non-linear filter: edge preserving
   cv::bilateralFilter(src, dst, int d, double sigmaColor, double sigmaSpace, int borderType = BORDER_DEFAULT) - ����� ����
   - �߽� ȭ�ҿ����� �Ÿ��Ӹ� �ƴ϶� �߽�ȭ�ҿ��� ��� ���̵� �����
   - edge�� �����ϳ� �ӵ��� �����ٴ� ������ ����
   - sigmaColor: Filter sigma in the color space. A larger value of the parameter means that farther colors
   within the pixel neighborhood (see sigmaSpace) will be mixed together, resulting in larger areas of semi-equal color.
   - sigmaSpace: Filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will
   influence each other as long as their colors are close enough (see sigmaColor).
   When d>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, d is proportional to sigmaSpace.
   - borderType: border mode used to extrapolate pixels outside of the image
   - Sigma values: For simplicity, you can set the 2 sigma values to be the same.
   If they are small (< 10), the filter will not have much effect, whereas if they are large (> 150),
   they will have a very strong effect, making the image look "cartoonish".
   - Filter size: Large filters (d > 5) are very slow, so it is recommended to use d=5 for real-time applications,
   and perhaps d=9 for offline applications that need heavy noise filtering.
   */
   //cv::medianBlur(processFrame, temp, 5);
   //cv::blur(grayFrame, processFrame , cv::Size(3, 3));
   //cv::bilateralFilter(processFrame, temp, -1, 15, 15);
   //cv::bilateralFilter(processFrame, temp, -1, 8, 8); // ������ ���͸�
   cv::GaussianBlur(grayFrame, blurredGrayFrame, cv::Size(3, 3), 1.5);

   //imshow("After smoothing gray Frame", blurredGrayFrame);

   cv::addWeighted(grayFrame, 1.5, blurredGrayFrame, -0.8, 0, sharpFrame); // grayframe�� sharpening�Ͽ� sharpFrame�� ����
                                                         //cv::blur(sharpFrame, sharpBlurFrame, cv::Size(5, 5));
                                                         //cv::bilateralFilter(sharpFrame, sharpBlurFrame, -1, 8, 8); // ������ ���͸�

   cv::medianBlur(sharpFrame, sharpBlurFrame, 3);
   cv::GaussianBlur(grayFrame, blurredGrayFrame, cv::Size(3, 3), 1.5);
   //get thresholdValue using Otus's method
   thresholdValue = getOtsuThreshold(blurredGrayFrame, grayFrame);
   cv::threshold(sharpBlurFrame, otsuProcessedFrame, thresholdValue, 255, CV_THRESH_BINARY);
   //imshow("After otsu thresholding", otsuProcessedFrame);

}

// getting threshod value using 'basic global thresholding method'
int PreProcess::getBasicGlobalThresholdValue(cv::Mat &frame) {
   int thresholdValue, low_cnt, high_cnt, low_sum, high_sum, i, j, th;
   thresholdValue = 120;
   th = 15; // ���� ���� ����
   low_cnt = high_cnt = low_sum = high_sum = 0;

   while (1) {
      for (j = 0; j < frame.rows; j++) {
         for (i = 0; i < frame.cols; i++) {
            if (frame.at<uchar>(j, i) < thresholdValue) {
               low_sum += frame.at<uchar>(j, i);
               low_cnt++;
            }
            else {
               high_sum += frame.at<uchar>(j, i);
               high_cnt++;
            }
         }
      }

      if (abs(thresholdValue - (low_sum / low_cnt + high_sum / high_cnt) / 2.0f) < th) {
         break;
      }
      else {
         thresholdValue = (low_sum / low_cnt + high_sum / high_cnt) / 2.0f;
         low_cnt = high_cnt = low_sum = high_sum = 0;
      }

   }
   return thresholdValue;
}

// return threshold integer value of Laplacian frame 
int PreProcess::getOtsuThreshold(cv::Mat &processFrame, cv::Mat &grayFrame) {
   // compute edge image as absolute value of the laplacian

   cv::Mat src = processFrame; // bluró���� gray����
   cv::Mat laplacianFrame; // laplacian operation�̵� frame(����� Canny operation�� �Ǿ� ����)
   cv::Mat otsuHistogramFrame; // otsu's method�� ���� histogram
   cv::Mat cannyFrame; // canny edge detection�� �� ����
   cv::Mat sharpBlurFrame; // sharpeneing�� ������ blur��Ų ����
   cv::Mat blurFrame = src; // bluró���� gray����
   cv::Mat multiplyFrame; // grayFrame & laplacianFrame
   int otsuThreshold; // threshold ��

   cv::Canny(src, laplacianFrame, 10, 300);
   //cv::Mat temp;
   //cv::Laplacian(src, laplacianFrame, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
   //cv::convertScaleAbs(laplacianFrame, laplacianFrame);
   //cv::imshow("laplacian Frame", laplacianFrame);
   //cv::threshold(laplacianFrame, temp, 20, 255, CV_THRESH_BINARY);
   //cv::imshow("thresholded laplacian frame - 20", temp);
   //cv::threshold(laplacianFrame, laplacianFrame, 0, 255, CV_THRESH_BINARY);
   //cv::imshow("thresholded laplacian frame - 0", laplacianFrame);
   //int threshold = getBasicGlobalThresholdValue(laplacianFrame); // get threshold value of laplacianFrame
   //cv::threshold(laplacianFrame, temp, 90, 255, 0); // thresholding laplacian frame
   //cv::imshow("thresholded laplacian frame", laplacianFrame);

   multiplyFrame = grayFrame & laplacianFrame; // and operation of grayFrame & laplacianFrame(����ũ�� Ȱ��)

                                    //cv::imshow("and of gray and T-laplacian", multiplyFrame);

                                    /* otsu's method���� ������׷� ���꿡 ����� otsuHistogramFrame */
   cv::MatIterator_<uchar> start, end, origin;
   origin = multiplyFrame.begin<uchar>();
   for (start = multiplyFrame.begin<uchar>(), end = multiplyFrame.end<uchar>(); start != end; ++start) {
      if (*start != 0) otsuHistogramFrame.push_back(*start);

   }

   //imshow("otsu Frame", otsuFrame);

   otsuThreshold = cv::threshold(otsuHistogramFrame, otsuHistogramFrame, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
   return otsuThreshold;
}

void PreProcess::updateFrame(const cv::Mat &frame) {
   processFrame = frame(cv::Rect(upperLeft, lowerRight));
   normalViewROIFrame = processFrame.clone();
   cv::cvtColor(processFrame, processFrame, CV_BGR2GRAY);

  
 // processFrame = ~processFrame;

   //imshow("update frame", processFrame);
   frameNum++; // temproal blurring���� ����ϴ� ����
   //leftOriginalFrame = frame(cv::Rect(upperLeft, lowerRigntOfLeftFrame));
   //rightOriginalFrame = frame(cv::Rect(upperLeftOfRightFrame, lowerRight));
   //roiFrame[ROI_LEVEL1] = processFrame(cv::Rect(UPPER_LEFT_1, LOWER_RIGHT_1));
   //roiFrame[ROI_LEVEL2] = processFrame(cv::Rect(UPPER_LEFT_2, LOWER_RIGHT_2));
   //roiFrame[ROI_LEVEL3] = processFrame(cv::Rect(UPPER_LEFT_3, LOWER_RIGHT_3));
   //roiFrame[ROI_LEVEL4] = processFrame(cv::Rect(UPPER_LEFT_4, LOWER_RIGHT_4));
   //frameCnt++;
}

bool PreProcess::addToFrameStack(cv::Mat frame) {
   /* frameStack�� 5���� �������� �����ص� */
   if (frameStackCount == STACK_FRAME_NUM) {
      // if (frameNum % 2 == 1 ) {
      frameStack.erase(frameStack.begin());
      frameStack.push_back(frame);
      return true;
      //cout << frameStack.size() << endl;
   }
   else if (frameStackCount < STACK_FRAME_NUM) {
      frameStack.push_back(frame);
      frameStackCount++;
      return false;
   }
}

cv::Mat PreProcess::temporalBlur(cv::Mat frame) {
   cv::Mat avgFrame;
   cv::Mat blurFrame;
   cv::Mat tempFrame;
   cv::Mat tempFrame2;
   cv::Mat resultFrame;

   double sharpSrcRate = 1.9;
   double sharpBlurRate = -0.9;

   // 6������ �������� 3���� �������� ��ջ��
   // 6������ �������� 4���� �������� ��� ���
   // 3������ �������� 8���� �������� ��� ���
   // 3������ �������� 6���� �������� ��� ���
   //int frameStepSize = 3;
   //int averageNum = 6;
   int frameStepSize = 1;
   int averageNum = 2;

   // �� �����Ӹ��� sharpening �۾�, avg frame �ʱ�ȭ
   cv::GaussianBlur(frameStack.at(0), blurFrame, cv::Size(3, 3), 1.5);
   cv::addWeighted(frameStack.at(0), sharpSrcRate, blurFrame, sharpBlurRate, 0, tempFrame);

   cv::GaussianBlur(frameStack.at(frameStepSize), blurFrame, cv::Size(3, 3), 1.5);
   cv::addWeighted(frameStack.at(frameStepSize), sharpSrcRate, blurFrame, sharpBlurRate, 0, tempFrame2);
   add(tempFrame / 2, tempFrame2 / 2, avgFrame);

   for (int i = 0; i < averageNum - 2; i++) {
      cv::GaussianBlur(frameStack.at(frameStepSize * (i + 2)), blurFrame, cv::Size(3, 3), 1.5);
      cv::addWeighted(frameStack.at(frameStepSize * (i + 2)), sharpSrcRate, blurFrame, sharpBlurRate, 0, tempFrame);
      add(tempFrame / (i + 3), avgFrame * (i + 2) / (i + 3), avgFrame);
   }


   //int frameStepSize = 6;
   //add(frameStack.at(0) / 2, frameStack.at(frameStepSize) / 2, avgFrame);
   //add(frameStack.at(frameStepSize * 2) / 3, avgFrame * 2 / 3, avgFrame);

   //cv::GaussianBlur(avgFrame, blurFrame, cv::Size(3, 3), 1.5);
   //cv::addWeighted(avgFrame, 1.9, blurFrame, -0.6, 0, resultFrame);
   //imshow("temporal avg", avgFrame);

   //medianBlur(avg, avg, 11);
   //return avgFrame;

   /////////////////////////////////////// Adaptive threshold ///////////////////////////////////////////
   //cv::Mat blurFrame;
   //      cv::Mat resultFrame;
   cv::GaussianBlur(avgFrame, avgFrame, cv::Size(5, 5), 2.5);
   //cv::blur(avgFrame, blurFrame, cv::Size(5, 5));
   //cv::addWeighted(avgFrame, 1.8, blurFrame, -0.6, 0, resultFrame);
   resultFrame = avgFrame;
   temporalProcessedFrame = avgFrame.clone();
   //imshow("Blurred temporal Image", temporalProcessedFrame);
   cv::blur(resultFrame, resultFrame, cv::Size(5,5));
   cv::adaptiveThreshold(resultFrame, resultFrame, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, -2);
   //imshow("After adaptive threshold", resultFrame);
   cv::medianBlur(resultFrame, resultFrame, 5);
   //morphologyEx(resultFrame, resultFrame, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 1);
   cv::erode(resultFrame, resultFrame, cv::Mat());
   cv::dilate(resultFrame, resultFrame, cv::Mat());
   cv::erode(resultFrame, resultFrame, cv::Mat());
   cv::dilate(resultFrame, resultFrame, cv::Mat());
   //imshow("Adaptive thresholded temporal blur", resultFrame);
   //cv::erode(resultFrame, resultFrame, cv::Mat());
   //cv::dilate(resultFrame, resultFrame, cv::Mat());
   //imshow("After morphology closing operation", resultFrame);
   return resultFrame;


}

cv::Mat PreProcess::getTemporalThresholdProcessedFrame() {
   return temporalProcessedThresholdedFrame;
}

bool PreProcess::isTemporalProcessComplete() {
   return isTemporalFrameReady;
}

cv::Mat PreProcess::getOtsuProcessedFrame() {
   return otsuProcessedFrame;
}

cv::Mat PreProcess::getTemporalProcessedFrame() {
   return temporalProcessedFrame;
}


cv::Mat PreProcess::getPreprocessedFrame_E(void) {
	return preprocessedFrame_E;
}

cv::Mat PreProcess::getGrayFrame(void) {
	return processFrame;
}

int PreProcess::getOffsetX(void) {
	return upperLeft.x;
}

int PreProcess::getOffsetY(void) {
	return upperLeft.y;
}

int PreProcess::getROIHight(void){
	return hight;
}

int PreProcess::getROIWidth(void){
	return width;
}

int PreProcess::getFrameHight(void) {
	return frameHight;
}

int PreProcess::getFrameWidth(void) {
	return frameWidth;
}

cv::Mat PreProcess::getNormalViewROIFrame(void){
	return normalViewROIFrame;
}
