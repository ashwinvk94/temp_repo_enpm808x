#include "ToyDetection.hpp"

int main(int argc, char** argv){
	ros::init(argc, argv, "toy_detection"); //node name

	ROS_INFO_STREAM("Started ToyDetection node");
	
	ToyDetection toy_detection;

	ros::spin();
	return 0;
}
