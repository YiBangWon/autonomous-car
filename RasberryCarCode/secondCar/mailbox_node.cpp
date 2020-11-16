#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
//#include <iostream>
#include <geometry_msgs/Twist.h>

using namespace std;

geometry_msgs::Twist cmd;
int kbhit();
int getch();
int sub_value=0;
int lab_count=0;

ros::Publisher pub2;

ros::Subscriber sub2;
ros::Subscriber sub4;
std_msgs::Int8 msg2;

int key = 0;
int stop_msg;

int spd=80;
// int const spd=0;
int angle=90;
int fri = 0;
int past_value = 0;
int derivative = 0;
float p_value = 0.2, i_value = 0.05, d_value = 0.7;
int integration = 0;

int pid(int value){

	derivative = value - past_value;
	
int pid_value = float(p_value*value) + float(i_value*value) + float(d_value*derivative);

pid_value = pid_value+90;
past_value = value;
integration += derivative;

	return pid_value;

}





void msgCallstop(const std_msgs::Int16::ConstPtr& given_msg2) {
	lab_count = (given_msg2->data);

	if(lab_count == 2)
		cmd.linear.x = 0;

	pub2.publish(cmd);
}


/*
void msgCallstop(const std_msgs::Int16::ConstPtr& given_msg2) {
	

	stop_sig = (given_msg2->data);
	
if(stop_sig == 9999)
	cmd.linear.x = 0;

cout<<"asdadasdasdsadsadasd	::::::::::::::::::::	"	<<stop_sig<<endl;

	pub2.publish(cmd);
	
}
*/
void msgCallback(const std_msgs::Int16::ConstPtr& given_msg) {
	//printf("do\n");
	
	sub_value = (given_msg->data);
	//printf("%d",sub_value);
	
	
	//angle=sub_value+90;-

	
/*
	if(sub_value < 15 && sub_value > -15){ //straight
		sub_value = 0;
		spd = 70;
}	
	
	else if(sub_value<-15){
		sub_value=(sub_value*2)/8; //-150-> -60, right
		spd = 53;
}

	else if(sub_value>=15){
		sub_value=(sub_value*2)/7; //150 -> 60, left

		spd = 53;
}

	else if(sub_value < -80){
		sub_value=(sub_value*3)/4; //150 -> 60, right

		spd = 53;
}


	else if(sub_value>=80){
		sub_value=(sub_value*2)/5; //150 -> 60, left

		spd = 53;
}
	*/

	angle= pid(sub_value);


	std:: cout << "angle : " << angle << endl;

	if(angle>150) angle=150;
	if(angle<30)  angle=30;

	if(angle > 100) spd = 75;
	else if(angle < 85) spd = 70;
	else spd = 80;

	// add by seung
	cmd.linear.x = spd;

	cmd.angular.z=angle;

//std:: cout<<"angle : " << angle << endl;
std:: cout<<"spd : " << spd << endl;
	//std::cout << "sub : " << sub_value << endl;
	pub2.publish(cmd);
}


int main(int argc, char **argv)
{
	cout << "input speed ";
	cin >> spd;
	if(spd > 80) spd = 80;
	cout << "speed: " << spd << endl;

	//for pub to motor
	cmd.linear.x=spd;
	ros::init(argc, argv, "msg_publisher");
	ros::NodeHandle nh2;
	ros::NodeHandle nh4;


	pub2 = nh2.advertise<geometry_msgs::Twist>("data_msg", 100);
sub4 = nh4.subscribe("cam_msg2",100,msgCallstop);

	sub2 = nh2.subscribe("cam_msg",100,msgCallback);
	

	ros::Rate loop_rate(1);

	//for sub from cam

	printf("mailbox is started");

	while(ros::ok())
	{
		ros::spinOnce();
		/*
		printf("\nESC to quit");
		if(kbhit){
			key = getch();
		}else{
			key = 0;
		}
		switch(key){
			case 27:
				msg2.data = 5;
				pub2.publish(msg2);
				printf("stop and turn off");
				return 0;
			case 's':
				msg2.data = 2;
				pub2.publish(msg2);
				break;
		}
		/*
		ROS_INFO("send msg = %d \n", msg2.data);
		*/

		//loop_rate.sleep();

	}

	return 0;
}
/*
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
*/
