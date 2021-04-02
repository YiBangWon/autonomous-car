#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "std_msgs/Int8.h"
#include <fcntl.h>
#include <sys/wait.h>
#include <termio.h>

#include <stdio.h>
#include <ctime>
#include <cmath>
#include "bits/time.h"
#include <unistd.h>
#include <string>

#include "opencv2/core/opengl.hpp"
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <geometry_msgs/Twist.h>

// #include <cv_bridge/cv_bridge.h>
#include "rgb_colors.h"
#include "cv_bridge.h"



using namespace std;
using namespace cv;
using namespace cv::cuda;

geometry_msgs::Twist cmd;
ros::Subscriber sub_image;

ackermann_msgs::AckermannDriveStamped ack_msg;
ros::Publisher ack_pub;

int kbhit();
int getch();

void filter_colors(Mat img_original, Mat& img_filtered)
{
   //이미지에서 하얀색만 필터링합니다. 차선 인식을 위해.
   //차선 색깔 범위
   Scalar lower_white = Scalar(200, 200, 200); //흰색 차선 (RGB)
   Scalar upper_white = Scalar(255, 255, 255);

   Mat img_bgr;
   img_original.copyTo(img_bgr);

   Mat white_mask, white_image;

   //Filter white
   inRange(img_bgr, lower_white, upper_white, white_mask); // image에 있는 색상 중 lower이상이고
                                             //   upper 이하인 색상에 대해 마스크를 만들어 white_mask에 저장
                                             // lower과 upper 사이에 있는 값은 255 아니면 0으로 하여 white_mask에 저장
                                             // 이 코드에서는 B, G, R 셋 다 200부터 255사이에 있는 픽셀 값만 저장

   //bitwise_and(img_bgr, img_bgr, white_image, white_mask); //mask값이 0이 아닌 부분만 and연산으로 더한다
                                             // original image와 original image를 더하는데
                                             // 이 때 white_mask에서 흰색인 부분(차선)만 더하여 white_image에 저장
                                             // 쉽게 말해, white_mask에서 흰색 부분에 해당하는 img_bgr을 white_image에 저장

   img_filtered = Scalar::all(0);
   img_bgr.copyTo(white_image, white_mask);
                                 //이렇게 해도 bitwise_and(img_bgr, img_bgr, white_image, white_mask); 와 동일함
   white_image.copyTo(img_filtered);
}

void set_ROI(Point points[], Mat& img, Mat& result) {
  points[0] = Point(img.cols / 2 - 158, img.rows); //왼쪽 아래
  points[1] = Point(img.cols / 2 - 91, (img.rows + (img.rows / 2 + 70)) / 2 - 10); //왼쪽 위
  points[2] = Point(img.cols / 2 + 146, (img.rows + (img.rows / 2 + 70)) / 2 - 10); //오른쪽 위
  points[3] = Point(img.cols / 2 + 255, img.rows); //오른쪽 아래


   const Point* ppt[1] = { points };
   int   npt[] = { 4 };
   fillPoly(img, ppt, npt, 1, Scalar(0, 255, 0), 8);

   //cuda::addWeighted(result, 0.85, img, 0.15, 0.0, result); // 반투명하게 roi 영역 보이기
}

Mat region_of_interest(Mat img_edges, Point *points)
{


   Mat img_mask = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);


   Scalar ignore_mask_color = Scalar(255, 255, 255);
   const Point* ppt[1] = { points };
   int npt[] = { 4 };


   //filling pixels inside the polygon defined by "vertices" with the fill color
   fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), 8);


   //returning the image only where mask pixels are nonzero
   Mat img_masked;

   //bitwise_and(img_edges, img_mask, img_masked);

   Mat img_result;
   img_result = Scalar::all(0);
   img_edges.copyTo(img_masked, img_mask);

   return img_masked;
}



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //image_transport::Publisher image_pub_2;

  ros::Publisher ack_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux/input/teleop", 1);
  ackermann_msgs::AckermannDriveStamped ack_msg;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    // image_sub_ = it_.subscribe("/usb_cam/image_raw/compressed", 100, &ImageConverter::imageCb);

    // image_pub_ = it_.advertise("/image_converter/output_video", 1);
    //image_pub_2 = it_.advertise("/image_converter/output_video2", 1);

  }

  ~ImageConverter()
  {
    //destroyWindow(OPENCV_WINDOW);
    //destroyWindow(OPENCV_GRAYIMAGE);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  // void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)

  {
    cout << "in img" << endl;
    cv_bridge::CvImagePtr cv_ptr;
    clock_t begin = clock();
    try
    {
      // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
      cout << cv_ptr->image.rows << " , " << cv_ptr->image.cols << endl;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    Mat img = cv_ptr->image;
    // cv_ptr->image = imageProcess(img);

    imshow("mysrc", img);

    // Mat ex1 ;
    // inRange(img,Scalar(40,210,210),Scalar(252,255,255),ex1);
    // Rect rect(0,0,640,480);
    // Mat rect_roi=ex1(rect) ;
    //
    // imshow("linemy", rect_roi);
    //
    // imshow("zzz", img);

    ack_msg.drive.speed = 0.5;

    ack_pub.publish(ack_msg);

    clock_t end = clock();
    double time = double(end - begin) / CLOCKS_PER_SEC;

    waitKey(33);
    // cout << "time: " << time << endl;

// 	///////////////////////////////
//
// 	//car_angle = car_angle + 0.1; // 1: left & -1: right
// 	//car_angle_speed = car_angle_speed + 1.0;
//
// //	car_angle += interval;
// // 	if( car_angle >= Max ) interval *= -1;
// //	else if( car_angle <= Min ) interval *= -1; // 1: left & -1: right
// // 	car_angle = Max;
//
//
// 	///////////////////////////////
//
//
//     // Update GUI Window
//
//
// //    ack_msg.drive.steering_angle = car_angle * car_angle_speed + 0.03;
//     ack_msg.drive.steering_angle = controller.getCarAngle() * car_angle_speed;
// /*
//     if(controller.getCarRunSpeed() == 0){
// 	return;
//     }
// */
//     ack_msg.drive.speed = car_direction * controller.getCarRunSpeed();
//
//
//     // Output modified video stream
//     image_pub_.publish(cv_ptr->toImageMsg());
//     ack_pub.publish(ack_msg);
//     //image_pub_.publish(cv_ptr_gray->toImageMsg());
//
//     clock_t end = clock();
//     double time = double(end - begin) / CLOCKS_PER_SEC;
//  //   cout << "time: " << time << endl;
  }
};


int main(int argc, char** argv)
{

    cout << "test1" << endl;
   // Mat src = imread("/home/nvidia/img/line1.jpg", IMREAD_GRAYSCALE);

   ros::init(argc, argv, "hiscar_v2");

   // ros::NodeHandle nh_;
   // ack_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux/input/teleop", 1);
   // // ackermann_msgs::AckermannDriveStamped ack_msg;
   //
   // cout << "test2" << endl;
   // sub_image = nh_.subscribe("/usb_cam/image_raw/compressed", 100, getImage);
   // // sub_image = nh_.subscribe("/usb_cam/image_raw", 1, getImage);
   //
   // cout << "test3" << endl;

   ImageConverter ic;
   ros::spin();

   // while(ros::ok())
   // {
   //    // cout << "test4" << endl;
   //     ros::spinOnce();
   // }

   cout << "test5" << endl;


   // VideoCapture cap(0);
   // Mat src;
   //
   // if(!cap.isOpened()) {
   //   cout << "error open camera!!!!!" << endl;
   //   return -1;
   // }
   //
   // ack_msg.drive.speed=0.5;
   // ack_msg.drive.steering_angle = 0;


   // if (src.empty())
   // {
   //    // help();
   //    cout << "can not open " << endl;
   //    return -1;
   // }
//    while(0) {
//
//      cap >> src;
// cout << "mymy" << endl;
//      if( src.empty() ) {
//        cout << "video is empty!" << endl;
//        break;
//      }
//
//      imshow("aa", src);
//      // fr_no++;
//
//      Mat mask;
//      Mat mask_faker;
//      filter_colors(src, mask_faker);
//      Canny(mask_faker, mask, 100, 200, 3);
//
//      Mat dst_cpu;
//
//      dst_cpu = mask.clone();
//      cv::cvtColor(mask, dst_cpu, COLOR_GRAY2BGR);
//
//      Point points[4]; // ROI of lower line
//      set_ROI(points, src, mask); // ROI할 point 설정
//      mask = region_of_interest(mask, points);
//      Mat dst_gpu = mask.clone();
//
//
//      /* cpu part
//      vector<Vec4i> lines_cpu;
//      {
//         const int64 start = getTickCount();
//
//         cv::HoughLinesP(mask, lines_cpu, 1, CV_PI / 180, 50, 60, 5);
//
//         const double timeSec = (getTickCount() - start) / getTickFrequency();
//         cout << "CPU Time : " << timeSec * 1000 << " ms" << endl;
//         cout << "CPU Found : " << lines_cpu.size() << endl;
//      }
//      cout<<"Here"<<endl;
//      for (size_t i = 0; i < lines_cpu.size(); ++i)
//      {
//         Vec4i l = lines_cpu[i];
//         line(dst_cpu, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
//      }
//      */
//
//
//      // GpuMat d_src(mask);
//      GpuMat d_src(mask); // TODO need to fix
//      GpuMat d_lines;
//      {
//         const int64 start = getTickCount();
//
//         Ptr<cuda::HoughSegmentDetector> hough = cuda::createHoughSegmentDetector(1.0f, (float)(CV_PI / 180.0f), 50, 5);
//
//         hough->detect(d_src, d_lines);
//
//         const double timeSec = (getTickCount() - start) / getTickFrequency();
//         cout << "GPU Time : " << timeSec * 1000 << " ms" << endl;
//         cout << "GPU Found : " << d_lines.cols << endl;
//      }
//      vector<Vec4i> lines_gpu;
//      if (!d_lines.empty())
//      {
//         lines_gpu.resize(d_lines.cols);
//         Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_gpu[0]);
//         d_lines.download(h_lines);
//      }
//      cout<<"Here22"<<endl;
//      for (size_t i = 0; i < lines_gpu.size(); ++i)
//      {
//         Vec4i l = lines_gpu[i];
//         line(dst_gpu, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
//      }
//      cout << "here333"<<endl;
//      // imshow("source", src);
//      cout << "here444"<<endl;
//      //imshow("detected lines [CPU]", dst_cpu);
//      // imshow("detected lines [GPU]", dst_gpu);
//
//      // imshow("aaa", src);
//      // imshow("bbb", mask);
//
//      // ack_pub.publish(ack_msg);
//
//       waitKey(33);
//    }


   // Sleep(5000);
   // usleep(100000 * 1000)
   return 0;
}
