#include <iostream>
#include "controller.h"
#include "math.h"

using namespace std;

//long cntframe_v2;
void Controller::updateLaneInfo(LaneInfo info, float car_angle, float interval, float speed_Min, float speed_Max) {
	laneInfo = info;
	
/*	//original
	int center_x = laneInfo.getCameraCenterX();
	int center_y = laneInfo.getCameraCenterY();
	cv::Point interception = laneInfo.getInterceptionPoint();

	twiddle(interception.x, center_x);

	int error = cal_error(interception.x, center_x);
	//std::cout << "test~~~~~~~~~~~~~~~~~~~~~~ This is error =  " << error << endl;

	// round under 9th degit	
	p1 = p[0] * error;
	error_sum += error;
	i1 = p[1]*error_sum;
	d1 = p[2]*(error - error_old);
	error_old = error;

	pid = p1 + i1 + d1;
	std::cout << "test~~~~~~~~~~~~~~~~~~~~~~ This is pid = " << pid << std::endl;
	if(-0.06 < pid && pid < 0.06){	
        //if(-0.10 < pid && pid < 0.15){
	//if (-0.065 < pid && pid < 0.065){
		//car_run_speed += interval;
		//if (car_run_speed > speed_Max) car_run_speed = speed_Max;
		std::cout << "!!~~~~~ Go Straight ~~~~~!!" << std::endl;
		car_run_speed = 0.2;
	}
	else {
		//car_run_speed -= interval;
		//if (car_run_speed < speed_Min) car_run_speed = speed_Min; 
		std::cout << "!! Curve !!" << std::endl;
		car_run_speed = 0.15;
	}
}
*/

///////////////////// kangjae edit
	/*int center_x = laneInfo.getCameraCenterX();
	int center_y = laneInfo.getCameraCenterY();
	cv::Point interception = laneInfo.getInterceptionPoint();

	twiddle(interception.x, center_x);

	int error = cal_error(interception.x, center_x);
	//std::cout << "test~~~~~~~~~~~~~~~~~~~~~~ This is error =  " << error << endl;

	// round under 9th degit	
	p1 = p[0] * error;
	error_sum += error;
	i1 = p[1]*error_sum;
	d1 = p[2]*(error - error_old);
	error_old = error;

	pid = p1 + i1 + d1;
	std::cout << "test~~~~~~~~~~~~~~~~~~~~~~ This is pid = " << pid << std::endl;
	//if(-0.20 < pid && pid < 0.25){
	//if(-0.10 < pid && pid < 0.15){		
	if (-0.065 < pid && pid < 0.065){
		std::cout << "!!~~~~~ Go Straight ~~~~~!!" << std::endl;
		car_run_speed += interval * 2;
		if (car_run_speed > speed_Max) car_run_speed = speed_Max;
		//std::cout << "!!~~~~~ Go Straight ~~~~~!!" << std::endl;
		//car_run_speed = 0.2;
	}
	else {
		std::cout << "!! Curve !!" << std::endl;	
		car_run_speed -= interval;
		if (car_run_speed < speed_Min) car_run_speed = speed_Min; 
		//std::cout << "!! Curve !!" << std::endl;
		//car_run_speed = 0.15;
	}
}*/

//////////////////////// laneDeparture //////////////////////

	int center_x = laneInfo.getCameraCenterX();
	int center_y = laneInfo.getCameraCenterY();
	int width = laneInfo.getFrameWidth();
	cv::Point lb = laneInfo.getLeftBottomPoint();
	cv::Point rb = laneInfo.getRightBottomPoint();
	cv::Point scarp = laneInfo.getScarpBottomPoint();
	cv::Point interception = laneInfo.getInterceptionPoint();
 	bool is_curve = laneInfo.getIsCurvedRoad();
	//bool driveStart = false;



	//////////////////////// Check Drive Start /////////////////////////// 
/*	if ((lb.x < 265 && lb.x > 180) || (rb.x < 575 && rb.x > 505)){
		driveStart = true;
	}
	cout << "~~~~~~~~~~~~~~ Drive Start  ===== " << driveStart << endl;

	if (driveStart){
*/
		//pattern_check
		countStartLine = laneInfo.getcountStartLine();
		cout << countStartLine << "####################################################################" << endl;
		//

		//countStartLine < 3
	
	
	 
		/*** Calculate center of bottom lanes ***/	
		if(laneInfo.getIsCurvedRoad()){
		// When road is scarp and only one lane is detected => calculate centerOfBottomLane
			if(laneInfo.getIsCurvedDirection() == 0){
				// left lane
				centerOfBottomLane = scarp.x + LANE_SPACE_PERSPECTIVE / 2;
				cout << "controller IsCurved - right" << endl;
			} else if(laneInfo.getIsCurvedDirection() == 1) {
				//right Lane
				centerOfBottomLane = scarp.x - LANE_SPACE_PERSPECTIVE / 2;
				cout << "Controller IsCurved - left" << endl;
			}
		cntFrame = 0;
		cntCurveFrame++;
		//cout << "~~~~~~~~~~~~~~~ This is cnfFrame in Curved: " << cntFrame << endl;
		} else if(laneInfo.getIsOnlyOneLaneDetectedR()){
		// When road is not scarp and only one lane is detected
			centerOfBottomLane = laneInfo.getRightBottomPointOneLane().x - LANE_SPACE_PERSPECTIVE /2; 		
		} else if(laneInfo.getIsOnlyOneLaneDetectedL()){
			centerOfBottomLane = laneInfo.getLeftBottomPointOneLane().x + LANE_SPACE_PERSPECTIVE /2;
		} else{
		// when road is not scarp and left, right lane is detected 
			centerOfBottomLane = (rb.x + lb.x)/2;
		}
	
		int distance = centerOfBottomLane - width/2;

	
	/*** check laneDeparture ***/ 
	calPid(centerOfBottomLane, center_x);

	if(laneInfo.getIsCurvedRoad()){
	//if(laneInfo.getIsCurvedRoad() || laneInfo.getIsOnlyOneLaneDetectedR() || laneInfo.getIsOnlyOneLaneDetectedL()){

		/* Speed Down*/		
		//car_run_speed -= interval*4;
		//if (car_run_speed < CURVE_SPEED) car_run_speed = CURVE_SPEED; 
		///////////////
		if(cntCurveFrame == 1)
			//car_run_speed = speed_Min;
			car_run_speed = 0;
		else {
			car_run_speed += interval*2;
			if (car_run_speed > CURVE_SPEED) car_run_speed = CURVE_SPEED; 
		}


		pid = calPid(centerOfBottomLane, center_x)*0.8;
		//car_run_speed = CURVE_SPEED;
		if(distance > 83) { // Car is located at the left side of lane
			isSafe = false;
			controlStatus = 0; // left status
		}
		else if (distance < -5){ // Car is located at the right side of lane
			isSafe = false;
			controlStatus = 1; //right status
		}
		else isSafe = true;
	}
	else{
		cntFrame++;
		//cout << "~~~~~~~~~~~~~~~ This is cnfFrame : " << cntFrame << endl;
		if (cntFrame > 15) {
			cntCurveFrame = 0;
			/* speed up */
			car_run_speed += interval*4;
			if (car_run_speed > GO_SPEED) car_run_speed = GO_SPEED;
			///////////////

			/* straight road */
			//car_run_speed = GO_SPEED;
			if (distance > 65) { // Car is located at the left side of lane
				//calPid(centerOfBottomLane, center_x);
				//pid = -0.01;
				pid = -0.04;
				controlStatus = 0; // left status
				//isSafe = false;
				}
			else if (distance < -5){ // Car is located at the right side of lane
				//calPid(centerOfBottomLane, center_x);
				//pid = 0.07;
				pid = 0.02;
				controlStatus = 1; //right status
				//isSafe = false;
			}
			//else isSafe = true;
			//else pid = calPid(centerOfBottomLane, center_x)*0.1;
			//else pid = 0.03;
			else pid = 0.0;
			
		}
		else{
			/* Speed Down*/		
			car_run_speed += interval*2;
			if (car_run_speed > CURVE_SPEED) car_run_speed = CURVE_SPEED; 
			///////////////

			//car_run_speed = CURVE_SPEED;
			//pid = pid*0.8;
		}
	}

	cout << "	~~~~~~~~~~~~~~~~~~~~~~~~~~ CURVE FRAME ==== "  << cntCurveFrame << endl;


	if (cntCurveFrame > 10){
		if(pid>0.40) pid = 0.40;
		else if(pid < -0.40) pid = -0.40;
	}
	
		/*else {
			calPid(centerOfBottomLane, center_x); 
			car_run_speed = CURVE_SPEED;
			if(distance > 83) { // Car is located at the left side of lane
				isSafe = false;
				controlStatus = 0; // left status
			}
			else if (distance < -5){ // Car is located at the right side of lane
				isSafe = false;
				controlStatus = 1; //right status
			}

*/


	/*** Follow center of bottom lanes ***/

	//if(isSafe){

	//	calPid(centerOfBottomLane, center_x);
		//car_run_speed = GO_SPEED;	

		//pid = pid + 0.318; // offset
		//if(-0.20 < pid && pid < 0.25){
		//if(-0.10 < pid && pid < 0.15){		
		//if (-0.065 <= pid && pid <= 0.065){
		//if (!is_curve){
			//std::cout << "!!~~~~~ Go Straight ~~~~~!!" << std::endl;
			//car_run_speed += interval * 2;
			//if (car_run_speed > speed_Max) car_run_speed = speed_Max;	
		//}
	//}
	//std::cout << "test~~~~~~~~~~~~~~~~~~~~~~ This is pid = " << pid << std::endl;
	//std::cout << "test~~~~~~~~~~~~~~~~~~~~~~ This is center = " << centerOfBottomLane << std::endl;

	/*** pattern_check ***/
	float target_frameNum = 0.05 / 0.03 / car_run_speed;

	if (countStartLine == 2) {
		//cout << "done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!dddddd " << endl;
		
	//	if (pattern_frameNum > target_frameNum) {
	//	if(true){
			car_run_speed = 0;
			pid = -0.15;
	//	}
	//	else {
	//		pattern_frameNum++;
	//	}
	}
	//
/*	}
	else {
		car_run_speed = 0.0;
		pid = 0.0;
	}
*/
}



float Controller::cal_error(int target, int current){
	return (float)(current - target);
}

void Controller::twiddle(int x, int center_x){
	int err;
	int best_err = cal_error(x, center_x);
	float threshold = 0.0000000000000000000000000000001;
	
	while ((dp[0]+dp[1]+dp[2]) > threshold) {
		for (int i = 0; i<3; i++){
			p[i] += dp[i];
			err = cal_error(x, center_x);

			if(err < best_err) { // there was some improvement
				best_err = err;
				dp[i] *= 1.1;
			}
			else {
				p[i] -= 2*dp[i];
				err = cal_error(x, center_x);
				if (err < best_err){
					best_err = err;
					dp[i] *=1.05;				
				}
				else {
					p[i] += dp[i];
					dp[i] *= 0.95;				
				}
			}
		}	
	}
}

float Controller::calPid(int target, int center){
	twiddle(target, center);
	error = cal_error(target, center);

	// round under 9th degit	
	p1 = p[0] * error;
	error_sum += error;
	i1 = p[1] * error_sum;
	d1 = p[2]*(error - error_old);
	//i1 = 0;
	//d1 = 0;
	error_old = error;

	return pid = p1 + i1 + d1;	
}

float Controller::getCarAngle(void) {
	return pid;
}

//float Controller::getCarSpeed(void){
//	return car_run_speed;
//}

float Controller::getPidValue(void){
	return pid;
}

float Controller::getCarRunSpeed(void){
	return car_run_speed;
}

float Controller::getCenterOfBottomLane(void){
	return centerOfBottomLane;
}

int Controller::getControlStatus(void){
	return controlStatus;
}

int Controller::getCountStartLine(void){
	return laneInfo.getcountStartLine();
}

void Controller::updateController(LaneInfo info){
	laneInfo = info;
}

int Controller::getStraightLaneCount(void){
	return cntFrame;
}

void Controller::setCountStartLine(int num){
	countStartLine = num;
}



