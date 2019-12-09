// @copyright 2019 Ashwin Varghese Kuruttukulam, 

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

void ToyDetection::initializePublishers(){
}
// cb for aruco detected (bool)
int ToyDetection::detectArUco(){
	int flag;
	try{
		flag = 1;
		listener.lookupTransform("/aruco_marker_frame", "/base_footprint",ros::Time(0),transform);
	

		ToyDetection::toyPose.header.frame_id="/base_footprint";
		ToyDetection::toyPose.header.stamp = ros::Time::now();
		
		ToyDetection::toyPose.pose.position.x = transform.getOrigin().x();
		ToyDetection::toyPose.pose.position.y = transform.getOrigin().y();
		ToyDetection::toyPose.pose.position.z = transform.getOrigin().z();

		ToyDetection::toyPose.pose.orientation.x = transform.getRotation().x();
		ToyDetection::toyPose.pose.orientation.y = transform.getRotation().y();
		ToyDetection::toyPose.pose.orientation.z = transform.getRotation().z();
		ToyDetection::toyPose.pose.orientation.w = transform.getRotation().w();
	}
	catch(const std::exception&){
		flag = 0;
	}
	return flag;
}

