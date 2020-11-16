
//#pragma once

#include "laneInfo.h"
#include "opencv2/opencv.hpp"
#include "opencv/cv.hpp"

void LaneInfo::setInterceptionPoint(cv::Point interceptPoint) {
	interceptionPoint = interceptPoint;
}


void LaneInfo::setCameraCenterX(int x) {
	cameraCenterX = x;
}

void LaneInfo::setCameraCenterY(int y) {
	cameraCenterY = y;
}

void LaneInfo::setIsCurvedRoad(bool truth, int option) {
	isCurvedRoad = truth;
	isCurvedDirection = option;
}

cv::Point LaneInfo::getInterceptionPoint(void) {
	return interceptionPoint;
}

void LaneInfo::setFrameHight(int h) {
	frameHight = h;
}

void LaneInfo::setFrameWidth(int w) {
	frameWidth = w;
}

int LaneInfo::getFrameWidth(void) {
	return frameWidth;
}

bool LaneInfo::getIsCurvedRoad(void) {
	return isCurvedRoad;
}

int LaneInfo::getCameraCenterX(void) {
	return cameraCenterX;
}

int LaneInfo::getCameraCenterY(void) {
	return cameraCenterY;
}

int LaneInfo::getFrameHight(void) {
	return frameHight;
}

void LaneInfo::setleftBottomPoint(cv::Point point){
	leftBottomPoint = point;
}

void LaneInfo::setRightBottomPoint(cv::Point point){
	rightBottomPoint = point;
}

cv::Point LaneInfo::getLeftBottomPoint(void){
	return leftBottomPoint;
}

cv::Point LaneInfo::getRightBottomPoint(void){
	return rightBottomPoint;
}

void LaneInfo::setScarpBottomPoint(cv::Point point){
	scarpBottomPoint = point;
}

cv::Point LaneInfo::getScarpBottomPoint(void){
	return scarpBottomPoint;
}

void LaneInfo::setIsOnlyOneLaneDetectedL(bool value , bool isRight){
	if(isRight){
		isOnlyOneLaneDetectedL = !value;
		isOnlyOneLaneDetectedR = value;
	} else{
		isOnlyOneLaneDetectedL = value;
	}	
}

void LaneInfo::setIsOnlyOneLaneDetectedR(bool value){
	isOnlyOneLaneDetectedR = value;
}

bool LaneInfo::getIsOnlyOneLaneDetectedL(void){
	return isOnlyOneLaneDetectedL;
}

bool LaneInfo::getIsOnlyOneLaneDetectedR(void){
	return isOnlyOneLaneDetectedR;
}

void LaneInfo::setLeftBottomPointOneLane(cv::Point point){
	leftBottomPointOneLane = point;
}

void LaneInfo::setRightBottomPointOneLane(cv::Point point){
	rightBottomPointOneLane = point;
}

cv::Point LaneInfo::getLeftBottomPointOneLane(void){
	return leftBottomPointOneLane;
}

cv::Point LaneInfo::getRightBottomPointOneLane(void){
	return rightBottomPointOneLane;
}

int LaneInfo::getIsCurvedDirection(void){
	return isCurvedDirection;
}


// pattern check

void LaneInfo::setcountStartLine(int count_StartLine) {
	countStartLine = count_StartLine;
}


int LaneInfo::getcountStartLine(void) {
	return countStartLine;
}

void LaneInfo::setIsFutureScarp(bool value){
	isFutureScarp = value;
}

bool LaneInfo::getIsFutureScarp(void){
	return isFutureScarp; 
}

void LaneInfo::setIsCurvedDirection(int direction){
	isCurvedDirection = direction;
}


