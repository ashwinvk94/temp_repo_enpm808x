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

#include <iostream>
#include <vector>
#include <iterator>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "ros/ros.h"
#include "../include/ROSModule.hpp"
#include "../include/UserInterface.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "kids_next_door/moveTo.h"
#include "kids_next_door/toyFound.h"
#include "../include/TaskPlanner.hpp"

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

PointHeadClientPtr pointHeadClient;

TaskPlanner::TaskPlanner() {
    ROS_INFO_STREAM("Constructing TaskPlanner node...");
    /* Check if everything is OK */
    if (!ros::ok()) {
        ROS_FATAL_STREAM("ROS node is not running.");
    }

    UserInterface ui(std::cin, std::cout);

    /* Initialize ist of toy IDs */
    toyIDs = ui.getIDs();

    /* Initialie storage location */
    storagePose = ui.getStorageLocation();

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

TaskPlanner::TaskPlanner(std::istream& inputStream, std::ostream& outputStream) {
    ROS_INFO_STREAM("Constructing TaskPlanner node...");
    /* Check if everything is OK */
    if (!ros::ok()) {
        ROS_FATAL_STREAM("ROS node is not running.");
    }

    UserInterface ui(inputStream, outputStream);

    /* Initialize ist of toy IDs */
    toyIDs = ui.getIDs();

    /* Initialie storage location */
    storagePose = ui.getStorageLocation();

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

int TaskPlanner::moveToPose(geometry_msgs::PoseStamped pose) {
    kids_next_door::moveTo srv;
    srv.request.goalPose = pose;
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


void TaskPlanner::taskPlanner() {
    /* Create iterator for toy IDs */
    std::vector<int>::iterator currToyID = toyIDs.begin();

    /* Create iterator for search positions */
    std::vector<geometry_msgs::PoseStamped>::iterator
                        currSearchPose = searchPoses.begin();

    int reachedToyFlag = 0;
    int pickedToyFlag = 0;
    int reachedStorgeFlag = 0;
    int storedToyFlag = 0;
    int cleanupFlag = 0;
    storagePose.pose.orientation.w = 1;

    while (ros::ok()) {
        /* If cleanup of toys needs to be done */
        if (cleanupFlag == 0) {
            /* If toy hasn't been reached */
            if (reachedToyFlag == 0) {
                ROS_INFO_STREAM("Looking for toy with ID : " << *currToyID);
                if (lookForToy(*currToyID) == 1) {   // Look for toy
                    ROS_INFO_STREAM("Found toy with ID : " << *currToyID);
                    reachedToyFlag = moveToPose(toyPose);
                } else {        // Search if can't see toy
                    ROS_INFO_STREAM("Exploring...");
                    if (currSearchPose == searchPoses.end()) {
                        /* Current ID could not be found, move to next */
                        ROS_INFO_STREAM("Current ID could not be" <<
                                        " found, moving to next.");
                        storedToyFlag = 1;
                    } else if (moveToPose(*currSearchPose) == 1) {
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
                if (moveToPose(storagePose) == 1) {
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
        std::cout<<srv.response.detection<<std::endl;
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

    srv.request.goalPose = toyPose;
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

