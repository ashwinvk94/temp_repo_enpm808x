// @copyright 2019 Ashwin Varghese Kuruttukulam, 

#include "ToyDetection.hpp"

ToyDetection::ToyDetection() {
	initializePublishers();
	initializeSubscribers();
}

void ToyDetection::initializeSubscribers(){
	ros::Subscriber arucoSub = nh.subscribe("/knd/aruco_detected",10, &ToyDetection::detectionCb,this);
	ros::ServiceServer server = nh.advertiseService("/knd/toyFound", &ToyDetection::findToySrv, this);
	ros::spin();
}

bool ToyDetection::findToySrv(kids_next_door::toyFound::Request& req,
                           kids_next_door::toyFound::Response& resp) {
    std_msgs::Int32 id = req.id;
	ros::param::set("/aruco_single/marker_id", id.data);	
	resp.detection = this->detectionFlag;
	ToyDetection::detectArUco();
	// geometry_msgs::PoseStamped emptyPose;
	resp.toyPose = ToyDetection::toyPose;
	return true;
}

void ToyDetection::detectionCb(const std_msgs::Bool::ConstPtr& detectionFlag) {
	this->detectionFlag = *detectionFlag;
}

void ToyDetection::initializePublishers(){
}
// cb for aruco detected (bool)
void ToyDetection::detectArUco(){
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

int main(int argc, char** argv){
	ros::init(argc, argv, "toy_detection"); //node name

	ROS_INFO_STREAM("Started ToyDetection node");
	
	ROSModule * toy_detection =  new ToyDetection();

	ros::spin();
	return 0;
}
