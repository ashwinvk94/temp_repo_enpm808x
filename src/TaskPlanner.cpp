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
 * @file        TaskPlanner.cpp
 * @author      Rohan Singh
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 7, 2019
 * @brief       Function definition for class TaskPlanner
 */

#include <iterator>
#include <vector>

#include "../include/TaskPlanner.hpp"

TaskPlanner::TaskPlanner() {
    ROS_INFO_STREAM("Constructing TaskPlanner node...");
    /* Check if everything is OK */
    if (!ros::ok()) {
        ROS_FATAL_STREAM("ROS node is not running.");
    }

    toyIDs.push_back(0);
    toyIDs.push_back(1);

    /* List of trajectory points for search */
    geometry_msgs::PoseStamped temp;
    temp.pose.position.x = 0;
    temp.pose.position.y = -7;
    temp.pose.position.z = 0;
    temp.pose.orientation.x = 0;
    temp.pose.orientation.y = 0;
    temp.pose.orientation.z = -0.7071;
    temp.pose.orientation.w = 0.7071;
    searchPoses.push_back(temp);

    initializeServiceClients();
}

TaskPlanner::~TaskPlanner() {

}

// void TaskPlanner::initializePublishers() {
//     ROS_INFO_STREAM("Initializing Publishers...");

//     /* Publisher for toy ID lookup */
//     toyIDPub = nh.advertise<std_msgs::Int32>("/knd/toyID", 1, true);

//     /* Publisher for desired robot pose */
//     goalPosePub = nh.advertise<geometric_msgs::PoseStamped>
//                     ("/knd/goalPose", 1, true);
// }

// void TaskPlanner::initializeSubscribers() {
//     ROS_INFO_STREAM("Initializing Subscribers...");

//     /* Subscriber for checking toy visibility */
//     toyFoundSub = nh.subscribe("/knd/toyFoundFlag", 1,
//                     &TaskPlanner::toyFoundCallback, this);

//     /* Subscriber for current robot pose */
//     currPoseSub = nh.subscribe("/knd/currPose", 1,
//                     &TaskPlanner::currPoseCallback, this);

//     /* Subscriber for current toy pose */
//     toyPoseSub = nh.subscribe("/knd/toyPose", 1,
//                     &TaskPlanner::toyPoseCallback, this);
// }

void TaskPlanner::initializeServiceClients() {
    ROS_INFO_STREAM("Initializing Service Clients...");

    /* Client for checking toy visibility */
    toyFoundClient = nh.serviceClient<kids_next_door::toyFound>(
                    "/knd/toyFound");

    /* Client for setting robot goal pose */
    goalPoseClient = nh.serviceClient<kids_next_door::moveTo>(
                    "/knd/moveTo");

    /* Client for picking up block */
}

// void TaskPlanner::explore() {

// }

int TaskPlanner::search(geometry_msgs::PoseStamped searchPose) {
    ROS_INFO_STREAM("Searching for toys");
    kids_next_door::moveTo srv;
    srv.request.goalPose = searchPose;
    if (goalPoseClient.call(srv)) {
        if (srv.response.reachedGoal.data) {
            return 1;
        } else {
            return 0;
        }
    } else {
        ROS_INFO_STREAM("Failed to call moveTo service.");
        return -1;
    }
}

// bool TaskPlanner::inRangeCheck() {

// }

// void TaskPlanner::addNewTask(int taskID, std::string taskName) {

// }

// int TaskPlanner::currTask() {

// }

void TaskPlanner::taskPlanner() {
    /* Create iterator for toy IDs */
    std::vector<int>::iterator currToyID = toyIDs.begin();

    /* Create iterator for search positions */
    std::vector<geometry_msgs::PoseStamped>::iterator currSearchPose = searchPoses.begin();

    // /* Set node rate */
    // ros::Rate loopRate(nodeHz);

    int reachedToyFlag = 0;
    int pickedToyFlag = 0;
    int reachedStorgeFlag = 0;
    int storedToyFlag = 0;
    int cleanupFlag = 0;

    while (ros::ok()) {
        /* If cleanup of toys needs to be done */
        if (cleanupFlag == 0) {
            /* If toy hasn't been reached */
            if (reachedToyFlag == 0) {
                if (lookForToy(*currToyID) == 1) {   // Look for toy
                    ROS_INFO_STREAM("Found toy with ID : " << *currToyID);
                    reachedToyFlag = goToToy();
                } else {        // Search if can't see toy
                    if (currSearchPose == searchPoses.end()) {
                        /* Current ID could not be found, move to next */
                        ROS_INFO_STREAM("Current ID could not be" <<
                                         "found, moving to next.");
                        storedToyFlag = 1;
                    } else if (search(*currSearchPose) == 1) {
                        currSearchPose++;
                    }
                }
            }
            /* If toy has been reached, pick up toy */
            if (reachedToyFlag == 1 && pickedToyFlag == 0) {
                ROS_INFO_STREAM("Picking up toy with ID : " << *currToyID);
                if (pickUpToy() == 1) {
                    ROS_INFO_STREAM("Picked up toy with ID : " << *currToyID);
                    pickedToyFlag = 1;
                }
            }
            /* If toy has been picked, go to storage */
            if (pickedToyFlag == 1 && reachedStorgeFlag == 0){
                ROS_INFO_STREAM("Going to storage");
                if (goToStorage() == 1) {
                    ROS_INFO_STREAM("Reached storage");
                    reachedStorgeFlag = 1;
                }
            }
            /* If storage has been reached, keep toy back */
            if (reachedStorgeFlag == 1 && storedToyFlag == 0) {
                ROS_INFO_STREAM("Storing toy with ID : " << *currToyID);
                if (storeToy() == 1) {
                    ROS_INFO_STREAM("Stored toy with ID : " << *currToyID);
                    storedToyFlag = 1;
                }
            }
            /* If toy is placed in storage, move to next */
            if (storedToyFlag == 1) {
                currToyID++;
                reachedToyFlag = 0;
                pickedToyFlag = 0;
                reachedStorgeFlag = 0;
                storedToyFlag = 0;
                currSearchPose = searchPoses.begin();
            }

            /* If all toys have been stored */
            if (currToyID == toyIDs.end()) {
                ROS_INFO_STREAM("All toys stored. Cleanup complete!!!.");
                cleanupFlag = true;
            }
        } else {
            shutdownRobot();
            break;
        }
    }
}

int TaskPlanner::lookForToy(int toyID) {
    kids_next_door::toyFound srv;
    srv.request.id.data= toyID;
    if (toyFoundClient.call(srv)) {
        toyPose = srv.response.toyPose;
        if (srv.response.detection.data) {
            return 1;
        } else {
            return 0;
        }
    } else {
        ROS_INFO_STREAM("Failed to call toyFound service.");
        return -1;
    }
}

int TaskPlanner::goToToy() {
    kids_next_door::moveTo srv;
    std::cout << toyPose;
    geometry_msgs::PoseStamped temp;
    temp.pose.position.x = toyPose.pose.position.x;
    temp.pose.position.y = toyPose.pose.position.y;
    temp.pose.position.z = toyPose.pose.position.z;
    temp.pose.orientation.w = 1.0;
    srv.request.goalPose = temp;
    if (goalPoseClient.call(srv)) {
        if (srv.response.reachedGoal.data) {
            return 1;
        } else {
            return 0;
        }
    } else {
        ROS_INFO_STREAM("Failed to call moveTo service.");
        return -1;
    }
}

int TaskPlanner::pickUpToy() {
    return 1;
}

int TaskPlanner::goToStorage() {
    kids_next_door::moveTo srv;
    srv.request.goalPose = storagePose;
    if (goalPoseClient.call(srv)) {
        if (srv.response.reachedGoal.data) {
            return 1;
        } else {
            return 0;
        }
    } else {
        ROS_INFO_STREAM("Failed to call moveTo service.");
        return -1;
    }
}

int TaskPlanner::storeToy() {
    return 1;
}

void TaskPlanner::shutdownRobot() {

}

int main(int argc, char** argv){
	ros::init(argc, argv, "TaskPlanner"); //node name

	ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

	ROS_INFO_STREAM("Started TaskPlanner node");

	
	TaskPlanner tp;

    tp.taskPlanner();
	// ROS_INFO_STREAM("Spinning");
	
	return 0;
};
