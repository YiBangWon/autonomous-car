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


using namespace std;
using namespace cv;
using namespace cv::cuda;

int kbhit();
int getch();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hiscar_v2");

  ros::NodeHandle nh_;
  ros::Publisher ack_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux/input/teleop", 1);
  ackermann_msgs::AckermannDriveStamped ack_msg;







  int key = 0;
	bool turn=true;
	printf("\nControl Your robot!\n");
    printf("---------------------------\n");
    printf("Moving around:\n");
    printf("         w    r\n");
    printf("    a    s    d\n");
	printf("         x     \n");
    //printf("    z    x    c\n\n");

	printf("\nESC to quit");
	float spd_send=0.45;
	float angle_send=0;

  string cmd;
  float temp = 0;

  char choice = 0;
  while(true) {

    if(kbhit()){
      choice = getch();
      switch (choice) {
        case 27:
          return 0;
          break;
        case 'a':
          angle_send = angle_send + 0.08;
          break;
        case 'd':
          angle_send = angle_send - 0.08;
          break;
        case 'w':
          spd_send = spd_send + 0.1;
          break;
        case 'x':
          spd_send = spd_send - 0.1;
          break;
        case 's':
          spd_send = 0;

        default:
          cout << "idontknow" << endl;
          break;
      }

      cout << "spd: " << spd_send << "angle: " << angle_send << endl;
    }


    ack_msg.drive.speed = spd_send;
    ack_msg.drive.steering_angle=angle_send;

    ack_pub.publish(ack_msg);

    usleep(30 * 1000);
    // system("cls");
  }

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
