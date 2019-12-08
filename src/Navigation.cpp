#include "../include/Navigation.hpp"

Navigation::Navigation() {
    

    initializeSubscribers();

}

void Navigation::initializeSubscribers() {
    ros::Subscriber goalSub = handler.subscribe("/knd/goalPose",10, &Navigation::goalPosCb,this);
    ros::Subscriber moveToSub = handler.subscribe("/knd/moveTo",10, &Navigation::moveToCb,this);
    ros::ServiceServer server = handler.advertiseService("/knd/moveTo", &Navigation::moveToSrv, this);
    ROS_INFO_STREAM("Started subscriber");
    ros::spin();
}
void Navigation::initializeServiceServers() {
    ROS_INFO_STREAM("Running Service");
    ros::spin();
}

void Navigation::goalPosCb(const geometry_msgs::Pose::ConstPtr& data) {
	ROS_INFO_STREAM("Running");
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = data->position.x;
    goal.target_pose.pose.position.y = data->position.y;
    goal.target_pose.pose.position.z = data->position.z;
    goal.target_pose.pose.orientation.x = data->orientation.x;
    goal.target_pose.pose.orientation.y = data->orientation.y;
    goal.target_pose.pose.orientation.z = data->orientation.z;
    goal.target_pose.pose.orientation.w = data->orientation.w;
}

void Navigation::moveToCb(const geometry_msgs::Pose::ConstPtr& data) {
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  
  	MoveBaseClient ac("/move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO_STREAM("Waiting for the action server to come up");
    }

    ROS_INFO_STREAM("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO_STREAM("Action Completed, moved to goal position");
    else
        ROS_INFO_STREAM("Failed to move to goal position");
}

bool Navigation::moveToSrv(kids_next_door::moveTo::Request& req,
                           kids_next_door::moveTo::Response& resp) {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  
  	MoveBaseClient ac("/move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO_STREAM("Waiting for the action server to come up");
    }
    // get new target position
    geometry_msgs::Pose data = req.goalPose;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = data.position.x;
    goal.target_pose.pose.position.y = data.position.y;
    goal.target_pose.pose.position.z = data.position.z;
    goal.target_pose.pose.orientation.x = data.orientation.x;
    goal.target_pose.pose.orientation.y = data.orientation.y;
    goal.target_pose.pose.orientation.z = data.orientation.z;
    goal.target_pose.pose.orientation.w = data.orientation.w;

    ROS_INFO_STREAM("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO_STREAM("Action Completed, moved to goal position");
    else
        ROS_INFO_STREAM("Failed to move to goal position");
    return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation"); //node name

	ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

	ROS_INFO_STREAM("Started navigation node");

	
	ROSModule * nav =  new Navigation();
	ROS_INFO_STREAM("Spinning");
	
	
	return 0;
};
