/******************************************************************************
 *  MIT License
 *
 *  Copyright (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

/**
 * @file        ToyDetection.cpp
 * @author      Ashwin Kuruttukulam
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam
 * @date        Dec 6, 2019
 * @brief       Function definotion for ToyDetection Class
 */

#include <iostream>
#include <vector>
#include "ROSModule.hpp"
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "kids_next_door/toyFound.h"
#include "ToyDetection.hpp"

ToyDetection::ToyDetection() {
	initializeSubscribers();
	initializeServiceServers();
}

void ToyDetection::initializeSubscribers(){
	arucoSub = nh.subscribe("/knd/aruco_detected",10, &ToyDetection::detectionCb,this);
	ros::spinOnce();
}

void ToyDetection::initializeServiceServers() {
	server = nh.advertiseService("/knd/toyFound", &ToyDetection::findToySrv, this);
	ros::spinOnce();
}
bool ToyDetection::findToySrv(kids_next_door::toyFound::Request& req,
                           kids_next_door::toyFound::Response& resp) {
    std_msgs::Int32 id = req.id;
	ros::param::set("/aruco_single/marker_id", id.data);	
	int flag = ToyDetection::detectArUco();
	
	// geometry_msgs::PoseStamped emptyPose;
	if(flag==1)
		resp.detection = this->detectionFlag;
	else{
		std_msgs::Bool falseMsg;
		falseMsg.data  = false;	
		resp.detection	= falseMsg;
	}
	resp.toyPose = ToyDetection::toyPose;
	return true;
}

void ToyDetection::detectionCb(const std_msgs::Bool::ConstPtr& detectionFlagMsg) {
	this->detectionFlag = *detectionFlagMsg;
}

// cb for aruco detected (bool)
int ToyDetection::detectArUco(){
	int flag;
	try{
		flag = 1;
		listener.lookupTransform("/map", "/aruco_marker_frame",ros::Time(0),transform);
	

		ToyDetection::toyPose.header.frame_id="/map";
		ToyDetection::toyPose.header.stamp = ros::Time::now();
		
		ToyDetection::toyPose.pose.position.x = transform.getOrigin().x();
		ToyDetection::toyPose.pose.position.y = transform.getOrigin().y()-0.5;

		ToyDetection::toyPose.pose.position.z = 0.0;

		ToyDetection::toyPose.pose.orientation.x = 0.0; //transform.getRotation().x();
		ToyDetection::toyPose.pose.orientation.y = 0.0; //transform.getRotation().y();
		// ToyDetection::toyPose.pose.orientation.z = transform.getRotation().z();
		// ToyDetection::toyPose.pose.orientation.w = transform.getRotation().w();
		ToyDetection::toyPose.pose.orientation.z = -0.7071;
		ToyDetection::toyPose.pose.orientation.w = 0.7071;
	}
	catch(const std::exception&){
		flag = 0;
	}
	return flag;
};
