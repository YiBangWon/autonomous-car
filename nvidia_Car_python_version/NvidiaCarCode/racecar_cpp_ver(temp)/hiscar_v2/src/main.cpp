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

#include "opencv2/core/opengl.hpp"
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
#include "cv_bridge.h"
#include "rgb_colors.h"


using namespace std;
using namespace cv;
using namespace cv::cuda;

ros::Subscriber sub_image;

ackermann_msgs::AckermannDriveStamped ack_msg;
ros::Publisher ack_pub;

float spd_send=0;
float angle_send=0;

string cmd;
float temp = 0;

char choice = 0;

class ImageConverter
{
protected:

  Mat img_original;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //image_transport::Publisher image_pub_2;

  ros::Publisher ack_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux/input/teleop", 1);
  ackermann_msgs::AckermannDriveStamped ack_msg;

  // ros::NodeHandle nh_;
  // ros::Publisher ack_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux/input/teleop", 1);
  // ackermann_msgs::AckermannDriveStamped ack_msg;

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

  int kbhit(void)
  {
      struct termios oldt, newt;
      int ch;
      int oldf;
      tcgetattr(STDIN_FILENO, &oldt);
      newt = oldt;
      newt.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
      oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
      fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
      ch = getchar();
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
      fcntl(STDIN_FILENO, F_SETFL, oldf);
      if(ch != EOF)
      {
      ungetc(ch, stdin);
      return 1;
      }
      return 0;
  }
  int getch(){
      int ch;
      struct termios buf, save;
      tcgetattr(0,&save);
      buf = save;
      buf.c_lflag &= ~(ICANON|ECHO);
      buf.c_cc[VMIN] = 1;
      buf.c_cc[VTIME] = 0;
      tcsetattr(0, TCSAFLUSH, &buf);
      ch = getchar();
      tcsetattr(0, TCSAFLUSH, &save);
      return ch;
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


    if(kbhit()){
      choice = getch();
      switch (choice) {
        case 27:
          // return 0;
          break;
        case 'a':
          angle_send = angle_send + 0.05;
          if(angle_send > 0.4) angle_send = 0.4;
          break;
        case 'd':
          angle_send = angle_send - 0.05;
          if(angle_send < -0.4) angle_send = -0.4;
          break;
        case 'w':
          spd_send = spd_send + 0.08;

          break;
        case 'x':
          spd_send = spd_send - 0.08;
          break;
        case 'h':
          angle_send = 0;
          break;
        case 's':
          spd_send = 0;
          angle_send = 0;
          break;

        default:
          cout << "idontknow" << endl;
          break;
      }

      cout << "spd: " << spd_send << "angle: " << angle_send << endl;
    }


    ack_msg.drive.speed = spd_send;
    ack_msg.drive.steering_angle=angle_send;

    ack_pub.publish(ack_msg);
    Mat img2;

    cv::resize(img, img2, Size(320,240));

    imshow("mysrc", img2);
    // img_original = img;
    // Mat ex1 ;
    // inRange(img,Scalar(40,210,210),Scalar(252,255,255),ex1);
    // Rect rect(0,0,640,480);
    // Mat rect_roi=ex1(rect) ;
    //
    // imshow("linemy", rect_roi);
    //
    // imshow("zzz", img);

    // ack_msg.drive.speed = 0.5;
    //
    // ack_pub.publish(ack_msg);

    // clock_t end = clock();
    // double time = double(end - begin) / CLOCKS_PER_SEC;

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

  // Mat get_img(){
  //   return img_original;
  // }

};

int kbhit();
int getch();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hiscar_v2");

  // ros::NodeHandle nh_;
  // ros::Publisher ack_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux/input/teleop", 1);
  // ackermann_msgs::AckermannDriveStamped ack_msg;

  cout << "testetstsetsetsetetsetse" << endl;

  int key = 0;
  bool turn=true;
  printf("\nControl Your robot! #####\n");
    printf("---------------------------\n");
    printf("Moving around:\n");
    printf("         w    r\n");
    printf("    a    s    d\n");
  printf("         x     \n");
	printf("\nESC to quit");

  ImageConverter ic;
  // Mat img = ic.get_img();
  // imshow("img", img);

  ros::spin();


    //printf("    z    x    c\n\n");


	// float spd_send=0;
	// float angle_send=0;
  //
  // string cmd;
  // float temp = 0;
  //
  // char choice = 0;
  // // while(true) {
  // while(false) {
  //
  //   if(kbhit()){
  //     choice = getch();
  //     switch (choice) {
  //       case 27:
  //         return 0;
  //         break;
  //       case 'a':
  //         angle_send = angle_send + 0.04;
  //         break;
  //       case 'd':
  //         angle_send = angle_send - 0.04;
  //         break;
  //       case 'w':
  //         spd_send = spd_send + 0.1;
  //         break;
  //       case 'x':
  //         spd_send = spd_send - 0.1;
  //         break;
  //       case 's':
  //         spd_send = 0;
  //         angle_send = 0;
  //
  //       default:
  //         cout << "idontknow" << endl;
  //         break;
  //     }
  //
  //     cout << "spd: " << spd_send << "angle: " << angle_send << endl;
  //   }
  //
  //
  //   ack_msg.drive.speed = spd_send;
  //   ack_msg.drive.steering_angle=angle_send;
  //
  //   ack_pub.publish(ack_msg);
  //
  //   usleep(30 * 1000);
  //   // system("cls");
  // }

	// while(1)
	// {
	// 	// if(temp != 0 && kbhit){
	// 	// 	key = getch();
	// 	// }else{
	// 	// 	key = 0;
  //   //   cout << "empty" << endl;
	// 	// }
  //
  //   if(kbhit()) {
  //     cin >> key;
  //     cout << "good" << key << endl;
  //   }
  //
	// 	// switch(key){
	// 	// 	case 27:
  //   //            printf("ESCAPE\n");
  //   //         	return 0;
  //   //         	break;
	// 	// 	case 'w':
  //   //             //msg2.data = 1;
  //   //     spd_send = spd_send + 0.005;
	// 	// 		if( spd_send> 0.8)
  //   //                  spd_send = 0.8;
	// 	// 		ack_msg.drive.speed=spd_send;
  //   //            printf("speed=%f\n", spd_send);
	// 	// 		//ack_msg.drive.steering_angle=90;
	// 	// 		break;
	// 	// 	//left 2.00
	// 	// 	case 'a':
	// 	// 		//ack_msg.drive.speed=50;
  //   //     angle_send = angle_send + 0.005;
	// 	// 		if(angle_send > 0.34)
	// 	// 			  angle_send = 0.34;
	// 	// 		ack_msg.drive.steering_angle=angle_send;
  //   //             printf("angle=%f\n", angle_send);
  //   //             //msg2.data = 3;
	// 	// 		break;
  //   //
	// 	// 	//right 3.00
	// 	// 	case 'd':
	// 	// 		//ack_msg.drive.speed=50;
  //   //     angle_send = angle_send - 0.005;
  //   //     if(angle_send < -0.34)
	// 	// 			  angle_send = -0.34;
	// 	// 		ack_msg.drive.steering_angle=angle_send;
	// 	// 		// ack_msg.drive.speed=spd_send;
  //   //             printf("angle=%f\n", angle_send);
  //   //             //msg2.data = 2;
	// 	// 		break;
	// 	// 	case 'x':
	// 	// 		//ack_msg.drive.speed=50;
  //   //     spd_send = spd_send - 0.005;
	// 	// 		if( spd_send < -0.5)
  //   //                  spd_send = -0.5;
	// 	// 		ack_msg.drive.speed=spd_send;
  //   //             printf("speed=%f\n", spd_send);
  //   //             //msg2.data = 2;
	// 	// 		break;
	// 	// 	case 's':
	// 	// 		spd_send=0;
	// 	// 		angle_send=0;
	// 	// 		ack_msg.drive.speed=spd_send;
	// 	// 		ack_msg.drive.steering_angle=angle_send;
	// 	// 		printf("angle=%f\n",angle_send);
  //   //             //msg2.data = 2;
	// 	// 		break;
	// 	// 	case 'r':
	// 	// 		turn=!turn;
	// 	// 		ack_msg.linear.y=turn;
	// 	// 	    printf("mode changed to  %s\n", turn? "camera mode": "keypad mode");
  //   //             //msg2.data = 2;
	// 	// 		break;
  //   //
  //   //
	// 	// }
  //
  //   usleep(500 * 1000);
  // }




    // ack_msg.drive.jerk = 1;
    // ack_msg.drive.acceleration = 1;


		// ack_pub.publish(ack_msg);
		//loop_rate.sleep();

    // ros::spin();


}
