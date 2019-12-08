// @copyright 2019 Ashwin Varghese Kuruttukulam, 

#include "ToyDetection.hpp"

ToyDetection::ToyDetection(ros::NodeHandle* nodehandle):nh_(*nodehandle){
	initializeSubscribers();
	initializePublishers();
}

void ToyDetection::initializeSubscribers(){
	
}

void ToyDetection::initializePublishers(){
	ros::Publisher tagPosePub = nh_.advertise<geometry_msgs::PoseStamped>("knd/tag_pose", 1);	
}

void ToyDetection::detectArUco(){
	ToyDetection::listener.lookupTransform("/aruco_frame", "/base_footprint",ros::Time(0),ToyDetection::transform);
	
	ToyDetection::poseStamped;
	poseStamped.header.frame_id="/base_footprint";
	ToyDetection::poseStamped.header.stamp = ros::Time::now()	
	
	ToyDetection::poseStamped.pose.position.x = transform.getOrigin().x();
	ToyDetection::poseStamped.pose.position.y = transform.getOrigin().y();
	ToyDetection::poseStamped.pose.position.z = transform.getOrigin().z();

	ToyDetection::poseStamped.pose.orientation.x = transform.getRotation().x();
	ToyDetection::poseStamped.pose.orientation.y = transform.getRotation().y();
	ToyDetection::poseStamped.pose.orientation.z = transform.getRotation().z();
	ToyDetection::poseStamped.pose.orientation.w = transform.getRotation().w();

	tagPosePub.publish(ToyDetection::poseStamped);
	
}

int main(int argc, char** argv){
	ros::init(argc, argv, "toy_detection"); //node name

	ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

	ROS_INFO_STREAM("Started ToyDetection node");
	
	ToyDetection toy_detection(&nh);

	ros::spin();
	return 0;
}
