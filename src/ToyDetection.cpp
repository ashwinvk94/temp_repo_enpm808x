// @copyright 2019 Ashwin Varghese Kuruttukulam, 

#include "ToyDetection.hpp"

ToyDetection::ToyDetection() {
	initializeSubscribers();
	initializePublishers();
}

void ToyDetection::initializeSubscribers(){
	ros::Subscriber arucoSub = nh.subscribe("/knd/aruco_detected",10, &ToyDetection::detectionCb,this);
	ros::ServiceServer server = nh.advertiseService("/knd/toyFound", &ToyDetection::findToySrv, this);
}

bool ToyDetection::findToySrv(kids_next_door::toyFound::Request& req,
                           kids_next_door::toyFound::Response& resp) {
    std_msgs::Int32 id = req.id;
	ros::param::set("/aruco_single/marker_id", id.data);	
	resp.detection = this->detectionFlag;
	// geometry_msgs::PoseStamped emptyPose;
	resp.toyPose = toyPose;
	return true;
}

void ToyDetection::detectionCb(const std_msgs::Bool::ConstPtr& detectionFlag) {
	this->detectionFlag = *detectionFlag;
}

void ToyDetection::initializePublishers(){
	ros::Publisher tagPosePub = nh.advertise<geometry_msgs::PoseStamped>("knd/tag_pose", 1);	
	ros::spin();
}
// cb for aruco detected (bool)
void ToyDetection::detectArUco(){
	listener.lookupTransform("/aruco_frame", "/base_footprint",ros::Time(0),transform);
	

	toyPose.header.frame_id="/base_footprint";
	toyPose.header.stamp = ros::Time::now();
	
	toyPose.pose.position.x = transform.getOrigin().x();
	toyPose.pose.position.y = transform.getOrigin().y();
	toyPose.pose.position.z = transform.getOrigin().z();

	toyPose.pose.orientation.x = transform.getRotation().x();
	toyPose.pose.orientation.y = transform.getRotation().y();
	toyPose.pose.orientation.z = transform.getRotation().z();
	toyPose.pose.orientation.w = transform.getRotation().w();
	
}

int main(int argc, char** argv){
	ros::init(argc, argv, "toy_detection"); //node name

	ROS_INFO_STREAM("Started ToyDetection node");
	
	ToyDetection toy_detection();

	ros::spin();
	return 0;
}
