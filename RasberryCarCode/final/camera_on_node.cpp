#define ROS
//#define CAMERA_SHOW
//#define CAMERA_SHOW_MORE

#ifdef ROS
#include <ros/ros.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include <unistd.h>
#endif

#include <cv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>

#define PI 3.1415926

using namespace std;
using namespace cv;

const int Width = 320;
const int Height = 240;
const int XHalf = (Width/2)+20;
const int YHalf = (Height/2);
const int YPoint = 30;
const float slope_threshold = 0.5;
const Scalar Red = Scalar(0, 0, 255);
const Scalar Blue = Scalar(255, 0, 0);
const Scalar Yellow = Scalar(50, 250, 250);
const Scalar Sky = Scalar(215, 200, 60);
const Scalar Pink = Scalar(220, 110, 230);

clock_t be = clock();
int track_count = 0;
int a = 0;
int cnt = 0;

clock_t start=0;
clock_t ending=0;

double interval=0;
int stop_sig = 0;
int find_yellow = 0;

Mat region_of_interest(Mat img_edges, Point* points)
{


   Mat img_mask = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);


   Scalar ignore_mask_color = Scalar(255, 255, 255);
   const Point* ppt[1] = { points };
   int npt[] = { 4 };


   //filling pixels inside the polygon defined by "vertices" with the fill color
   fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), 8);


   //returning the image only where mask pixels are nonzero
   Mat img_masked;
   bitwise_and(img_edges, img_mask, img_masked);


   return img_masked;
}

void filter_colors_yellow(Mat img_original, Mat& img_filtered)
{

	Scalar lower_white_color = Scalar(100, 200, 205);
	Scalar upper_white_color = Scalar(160, 255, 255);

	Mat img_bgr;
	img_original.copyTo(img_bgr);

	Mat white_mask, white_image;

	//Filter Red
	inRange(img_bgr, lower_white_color, upper_white_color, white_mask);

	img_filtered = Scalar::all(0);
	img_bgr.copyTo(white_image, white_mask);


	white_image.copyTo(img_filtered);

}



void filter_colors_white(Mat img_original, Mat& img_filtered)
{

	Scalar lower_white_color = Scalar(190, 190, 190);
	Scalar upper_white_colpr = Scalar(255, 255, 255);

	Mat img_bgr;
	img_original.copyTo(img_bgr);

	Mat white_mask, white_image;

	//Filter Red
	inRange(img_bgr, lower_white_color, upper_white_colpr, white_mask);

	img_filtered = Scalar::all(0);
	img_bgr.copyTo(white_image, white_mask);


	white_image.copyTo(img_filtered);

}

int orange_detect(Mat &img){

	Mat thresh, roi;
	
	//Rect rect_roi(0, 100, Width, 120);
	//roi = img(rect_roi);
	Rect rect_roi(img.cols*0.1, img.rows*0.55, img.cols*0.85, img.rows*0.35);
	Mat image_roi = img(rect_roi);
	Rect yellow_roi(0, 90, Width/2, 120);
	Mat roi_yellow = img(yellow_roi);
	
	//image_roi.convertTo(image_roi, -1, 1, 30); //increase the brightness by 30 for each pixel 
	Mat image_white, image_yellow;
	filter_colors_white(image_roi, image_white);
	filter_colors_yellow(roi_yellow, image_yellow);
	
	//cvtColor(roi, thresh, COLOR_BGR2GRAY);
	
	//threshold(thresh, thresh, 150, 255,THRESH_BINARY);
	
//adaptiveThreshold(thresh,thresh,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,31,5);

	/*int white_point;
	for(int i = 0; i < thresh.rows; i++){
		for(int j = 0; j < thresh.cols; j++)
		white_point = thresh.at<uchar>(i,j);
		if(white_point >= 250){
			pixel++;
		}
	}*/
	int pixel = 0;
	for(int i = 0; i < roi.rows; i++){
		for(int j = 0; j < roi.cols; j++){
				if (image_white.at<Vec3b>(i, j)[0] >= 190 && image_white.at<Vec3b>(i, j)[0] <= 255) {
					if (image_white.at<Vec3b>(i, j)[1] >= 190 && image_white.at<Vec3b>(i, j)[1] <= 255) {
						if (image_white.at<Vec3b>(i, j)[2] >= 190 && image_white.at<Vec3b>(i, j)[2] <= 255) {
							
							pixel++;
							
						}
					}
				}
		}
	}
	int yellow_point = 0;
	for(int i = 0; i < roi_yellow.rows; i++){
		for(int j = 0; j < roi_yellow.cols; j++){
			if (image_yellow.at<Vec3b>(i, j)[0] >= 100 && image_yellow.at<Vec3b>(i, j)[0] <= 160) {
					if (image_yellow.at<Vec3b>(i, j)[1] >= 200 && image_yellow.at<Vec3b>(i, j)[1] <= 255) {
						if (image_yellow.at<Vec3b>(i, j)[2] >= 205 && image_yellow.at<Vec3b>(i, j)[2] <= 255) {
							
							yellow_point++;
							
						}
					}
				}
		}
	}
	//line(roi, Point(130,20),Point(190,20),Scalar(0,0,255),3);
	
	cout << "white pixel : " << pixel << endl;
	cout << "yellow pixel : " << yellow_point << endl;
	
	if(yellow_point >= 2800){
	 	find_yellow = 1;
	}
	//	imshow("adadad",	thresh);
	//	imshow("NONONOONO", roi);
	return track_count;


}



void show_lines(Mat &img, vector<Vec4i> &lines, Scalar color = Scalar(0, 0, 0),int thickness=2){

	bool color_gen = false;



	if (color == Scalar(0, 0, 0))

		color_gen = true;

	for (int i = 0; i < lines.size(); i++)

	{

		if (color_gen == true)

			color = Scalar(rand() % 256, rand() % 256, rand() % 256);

		line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), color, thickness);

	}

}



void  split_left_right(vector<Vec4i> lines, vector<Vec4i>&left_lines, vector<Vec4i> &right_lines)
{
	vector<float> slopes;
	vector<Vec4i> new_lines;

	for (int i = 0; i < lines.size(); i++)
	{
		int x1 = lines[i][0];
		int y1 = lines[i][1];
		int x2 = lines[i][2];
		int y2 = lines[i][3];

		float slope;
		//Calculate slope
		if (x2 - x1 == 0) //corner case, avoiding division by 0
			slope = 999.0; //practically infinite slope
		else
			slope = (y2 - y1) / (float)(x2 - x1);

		//Filter lines based on slope
		if (abs(slope) > slope_threshold) {
			slopes.push_back(slope);
			new_lines.push_back(lines[i]);
		}
	}

	for (int i = 0; i < new_lines.size(); i++)
	{
		Vec4i line = new_lines[i];
		float slope = slopes[i];

		int x1 = line[0];
		int y1 = line[1];
		int x2 = line[2];
		int y2 = line[3];

		if (slope > 0 && x1 > XHalf && x2 > XHalf)//
			right_lines.push_back(line);
		else if (slope < 0 && x1 < XHalf && x2 < XHalf)//slope < 0 && x1 < cx && x2 < cx
			left_lines.push_back(line);
	}
}

bool find_line_params(vector<Vec4i> &left_lines, float *left_m, float *left_b)
{

	float  left_avg_x = 0, left_avg_y = 0, left_avg_slope = 0;

	if(left_lines.size() == 0)
		return false;
	
	for (int i = 0; i < left_lines.size(); i++)//calculate right avg of x and y and slope
	{
		//line(roi, Point(left_lines[i][0],left_lines[i][1]), Point(left_lines[i][2],left_lines[i][3]), color, 3);
		left_avg_x += left_lines[i][0];
		left_avg_x += left_lines[i][2];
		left_avg_y += left_lines[i][1];
		left_avg_y += left_lines[i][3];
		left_avg_slope += (left_lines[i][3] - left_lines[i][1]) / (float)(left_lines[i][2] - left_lines[i][0]);
	}
	left_avg_x = left_avg_x / (left_lines.size() * 2);
	left_avg_y = left_avg_y / (left_lines.size() * 2);
	left_avg_slope = left_avg_slope / left_lines.size();

	// return values
	*left_m = left_avg_slope;
	//b=y-mx //find Y intercet
	*left_b = left_avg_y - left_avg_slope * left_avg_x;
	

	return true;
}




void find_lines(Mat &img, vector<cv::Vec4i> &left_lines, vector<Vec4i>& right_lines, float *rdistance, float *ldistance)
{
	static float left_slope_mem = 1, right_slope_mem = 1, right_b_mem = 0, left_b_mem = 0;

   float left_b, right_b, left_m, right_m;
   
   bool draw_left = find_line_params(left_lines, &left_m, &left_b);
   if (draw_left) {
	   float left_x0 = (-left_b) / left_m;
	   float left_x120 = (YHalf - left_b) / left_m;
	   left_slope_mem = left_m;
	   left_b_mem = left_b;
#ifdef CAMERA_SHOW
	   line(img, Point(left_x0, 0), Point(left_x120, YHalf), Blue, 3);

#endif 
		cout << left_lines.size() << " left lines,";
   }
   else {
	   cout << "\tNo Left Line,";
   }

   bool draw_right = find_line_params(right_lines, &right_m, &right_b);
   if (draw_right) {
	   float right_x0 = (-right_b) / right_m;
	   float right_x120 = (YHalf - right_b) / right_m;
	   right_slope_mem = right_m;
	   right_b_mem = right_b;
#ifdef CAMERA_SHOW
	   line(img, Point(right_x0, 0), Point(right_x120, YHalf), Red, 3);
#endif
	   cout << right_lines.size() << " right lines" <<endl;
   }
   else {
	   cout << "\tNo RIght Line" << endl;
   }
   // y = mx + b ==> x0 = (y0-b)/m
   float left_xPt = ((YPoint - left_b_mem) / left_slope_mem);
   float right_xPt = ((YPoint - right_b_mem) / right_slope_mem);


//	cout << "left _ xPt : " << left_xPt << "right _ xPt" << right_xPt << endl;


   *ldistance =  ( XHalf - left_xPt ) ;
   *rdistance =  ( right_xPt - XHalf ) ;
}

///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

/*
int find_stop_line(Mat &frame_inrange){
	vector<Vec4i> line_set;
	Mat gaussian_inrange, edge;
	int cnt_delay=0;
	GaussianBlur(frame_inrange,gaussian_inrange,Size(3,3),1.5);

	Canny(gaussian_inrange,edge,70,150,3);

	HoughLinesP(edge,line_set,1,CV_PI/180,30,50,10);
	for(int i=0; i<line_set.size(); i++){
		int x1 = line_set[i][0];
		int y1 = line_set[i][1];
		int x2 = line_set[i][2];
		int y2 = line_set[i][3];
		float slope;
		slope = (y2-y1)/(float)(x2-x1);
	//cout<<"slope:"<<slope<<endl;
		if(abs(slope)<0.3){
			//cout<<"stop line detected"<<endl;
			ending = clock();
			interval = (double)(ending-start)/CLOCKS_PER_SEC;
			start = clock();
			//cout<<"laptime:"<<	interval<<endl;
			}
		if(interval>5.0){
			cnt_delay++;
			//cout<<"cbt_delay:"<<cnt_delay<<endl;
			if(cnt_delay==1){
				cnt++;
				
				}
			if(cnt==2){
				cout<<"car has stop"<<endl;
				return 9999;
				}
			}
		
	}
cout<<"cnt ::::::::::::::::::::::::::: "<<cnt<<endl;

	return 0;

}

*/
///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



int img_process(Mat &frame){
    Mat grayframe, edge_frame, roi_gray_ch3;
	Mat roi;
    cvtColor(frame, grayframe, COLOR_BGR2GRAY);
    Rect rect_roi(0, YHalf, Width, YHalf);
    roi = frame(rect_roi);

	//ros::NodeHandle nh3;
 	//std_msgs::Int16 cam_msg2;
	//ros::Publisher pub3 = nh3.advertise<std_msgs::Int16>("cam_msg2",100);

///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/*
Mat cha;
Mat inrange;
Mat rect_inrange;
	cvtColor(frame, cha, COLOR_BGR2YCrCb);
	inRange(cha, Scalar(0, 150, 100), Scalar(255,225,250),inrange);
Size inrange_size=inrange.size();

Rect rect(100,inrange_size.height/2,200,inrange_size.height/2);
rect_inrange=inrange(rect);
stop_sig=find_stop_line(rect_inrange);





cout << "stttttttttttttttttttttttttop sig ========================== " << stop_sig << endl;
//imshow("rect_inrange",rect_inrange);
*/
///@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    cvtColor(roi,grayframe,COLOR_BGR2GRAY) ;
    GaussianBlur(grayframe,grayframe,Size(5,5),3);
	cvtColor(grayframe, roi_gray_ch3, COLOR_GRAY2BGR);
    Canny(grayframe,edge_frame,190,150,3); //min_val, max val , filter size

    vector<cv::Vec4i> lines_set;

    cv::HoughLinesP(edge_frame,lines_set,1,PI/180,30,30,10);
#ifdef CAMERA_SHOW_MORE
	show_lines(roi_gray_ch3, lines_set);
#endif
	vector<Vec4i> right_lines;
	vector<Vec4i> left_lines;
	split_left_right(lines_set, left_lines, right_lines);
#ifdef CAMERA_SHOW
	show_lines(roi, left_lines, Sky, 2);
	show_lines(roi, right_lines, Pink, 2);
#endif
	float ldistance, rdistance;
	find_lines(roi, left_lines, right_lines, &rdistance, &ldistance);

	int differ = ldistance-rdistance;
#ifdef CAMERA_SHOW
//    circle (roi, Point(XHalf, YPoint), 5, Scalar(250,250,250),-1);
//	circle(roi, Point(XHalf + differ, YPoint), 5, Scalar(0, 0, 255), 2);
//    putText(roi,format("%3d - %3d = %3d",(int)ldistance, (int)rdistance, differ),Point(XHalf-100,YHalf/2),FONT_HERSHEY_SIMPLEX,0.5,Yellow,2);
//	imshow("roi",roi);
    imshow("edgeframe",edge_frame);
#endif
#ifdef CAMERA_SHOW_MORE
	imshow("frame", frame);
//	imshow("roi_gray_ch3", roi_gray_ch3);
#endif
//	cout << endl <<  "differ : " << differ << endl;
	return differ;
}


int main(int argc, char**argv){
	VideoCapture cap(0);
	geometry_msgs::Twist cmd;
  	//VideoCapture cap("/cseecar/catkin_ws/video/clockwise.mp4");
	  Mat frame;

	  if(!cap.isOpened()){
		std::cout<<"no camera!"<< std::endl;
		return -1;
	  }
	/*
	Size size = Size((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));

		VideoWriter writer;
	double fps = 30.0;
	writer.open("chris.avi", VideoWriter::fourcc('M','J','P','G'), fps, size, true);
	*/
	#ifdef ROS
	  ros::init(argc, argv, "cam_msg_publisher");
	  ros::NodeHandle nh;
	  ros::NodeHandle nh3;
	  std_msgs::Int16 cam_msg;
	  std_msgs::Int16 cam_msg2;
	  
	  ros::Publisher pub2 = nh3.advertise<geometry_msgs::Twist>("data_msg_key", 100);
	  ros::Publisher pub = nh.advertise<std_msgs::Int16>("cam_msg",100);
	  ros::Publisher pub3 = nh3.advertise<std_msgs::Int16>("cam_msg2",100);
 	  
	  int init_past=1;
	  //--------------------------------------------------
	  ros::Rate loop_rate(50);
	  cout<<"start"<<endl;
	#endif

	  int differ, key, fr_no = 0;
	  bool capture = true;
	  //clock_t tStart = clock();
	  int time = 0;
	  int spd_send = 0;
	  int angle_send = 30;
	  for(;;){
			if (capture) {
				  cap >> frame;

				  if (frame.empty()){
						cout << "frame is empty" << endl;
						sleep(1);
						break;
					}
			}

		//writer.write(frame);


			if ((key = waitKey(60)) >= 0) {
				if (key == 27){
					break;
				}
				else if (key = ' ') {
					capture = !capture;
				}
			}

			if (capture == false)
				continue;

			fr_no++;
			resize(frame,frame,Size(Width,Height));

			differ = img_process(frame);

/*
			if(stop_sig==9999){
				cam_msg2.data = stop_sig;
				pub3.publish(cam_msg2);
			}
*/
			cout << "differ : " << differ << endl;
			if(differ >= 200)	
				differ = 200;

			if(differ < -200)
				differ = -200;
			if(abs(differ) < 140)
				differ *= 0.2;
			
			orange_detect(frame);
			
						
			if(find_yellow == 1){
				time += 1;
			}
			
			if(find_yellow == 1 && time >= 0 && time <25){
				differ = 270;
			}
			else if(find_yellow == 1 && time >= 25 && time < 49){
				differ = -200;
			}
			else if(find_yellow == 1 && time == 49){
				cout << "stop! " << endl;
				spd_send=0;
				angle_send=90;
				cmd.linear.x=spd_send;
				cmd.angular.z=angle_send;
				pub2.publish(cmd);
			}
			else if(find_yellow == 1 && time ==50){
				cout << "back back " << endl;
				
				spd_send = -110;
				cmd.linear.x=spd_send;
				pub2.publish(cmd);
				cout << "spd : " << spd_send << endl;
				
			}
			else if(find_yellow == 1 && time ==51){
				cout << "stop! " << endl;
				spd_send=0;
				angle_send=90;
				cmd.linear.x=spd_send;
				cmd.angular.z=angle_send;
				pub2.publish(cmd);
			}
			else if(find_yellow == 1 && time =>52 && time < 80){
				cout << "back back " << endl;
				
				spd_send = -110;
				cmd.linear.x=spd_send;
				pub2.publish(cmd);
				cout << "spd : " << spd_send << endl;
				
			}
			if(time >= 80 && time < 100){
				cout << "time over" << endl;
				//cam_msg2.data = 2;
				//pub3.publish(cam_msg2);
				//loop_rate.sleep();
				
				spd_send = 0;
				angle_send = 90;
				cmd.angular.z = angle_send;
				cmd.linear.x = spd_send;
				
				pub2.publish(cmd);				
			}
			if(time == 100){
				spd_send = 65;
				cmd.linear.x = spd_send;
				pub2.publish(cmd);	
			}
			if(find_yellow == 1 && time >= 100 && time < 145){
				cout << "go right "<<endl;
				angle_send = 20;
				cmd.angular.z = angle_send;
				pub2.publish(cmd);
			}
			else if(find_yellow == 1 && time >= 145 && time < 165){
				cout << "go left " << endl;
				angle_send = 135;
				cmd.angular.z = angle_send;
				pub2.publish(cmd);
			}
			if(time >= 165){
				cout << "go straight" << endl;
				cmd.angular.z = 90;
				pub2.publish(cmd);
			}
			if(time >= 200){
				cout << "time end! " << endl;
				cam_msg2.data = track_count;
				pub3.publish(cam_msg2);
				loop_rate.sleep();
				
				spd_send = 0;
				cmd.linear.x = spd_send;
				
				pub2.publish(cmd);
				break;
			}
			
			cout << " :: yollow :: " << find_yellow << endl;
			cout << " :: differ_result :: " << differ << endl;
			
			if(time < 49){
				cam_msg.data = differ;
				pub.publish(cam_msg);
			}
			
			if(track_count ==2){
				cam_msg2.data = track_count;
				pub3.publish(cam_msg2);
				loop_rate.sleep();
				
				spd_send=0;
				cmd.linear.x=spd_send;
				
				pub2.publish(cmd);
				exit(1);
			}
			
		/*#ifdef ROS
	
		#else
			std::cout << fr_no << ":" << differ << endl;
		#endif
		*/	}

	  std::cout<<"Camera off"<<endl;

	  return 0;
}
 
