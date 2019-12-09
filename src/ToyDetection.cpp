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
}

int main(int argc, char** argv){
	ros::init(argc, argv, "toy_detection"); //node name

	ROS_INFO_STREAM("Started ToyDetection node");
	
	ROSModule * toy_detection =  new ToyDetection();

	ros::spin();
	return 0;
}
