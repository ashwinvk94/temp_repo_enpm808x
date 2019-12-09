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
 * @file        TaskPlanner.hpp
 * @author      Abhinav Modi
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 1, 2019
 * @brief       Header file for Task planner module to make high-level decisions for 
 *              switching between navigation, manipulation and obstacle avoidance tasks
 */

#ifndef INCLUDE_TASKPLANNER_HPP_
#define INCLUDE_TASKPLANNER_HPP_

#include <iostream>
#include <vector>
#include <iterator>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/shared_ptr.hpp>
#include <control_msgs/PointHeadAction.h>

#include "ros/ros.h"
#include "../include/ROSModule.hpp"
#include "../include/UserInterface.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "kids_next_door/moveTo.h"
#include "kids_next_door/toyFound.h"

class TaskPlanner : public ROSModule {
  public:

    /**
     * @brief Constructor for class
     *
     * @param None
     *
     * @return None
     */
    TaskPlanner();

    /**
     * @brief Constructor for class with input/output stream arguments
     *
     * @param inputStream Input Stream object
     * 
     * @param outputStream Output Stream object
     *
     * @return None
     */
    TaskPlanner(std::istream& inputStream, std::ostream& outputStream);

    /**
     * @brief Destructor for class
     *
     * @param None
     *
     * @return None
     */
    ~TaskPlanner();

    /**
     * @brief Function to initialize Service Clients
     *
     * @param None
     *
     * @return None
     */
    void initializeServiceClients();

    /**
     * @brief Calls service to move Tiago Base to given position
     *
     * @param pose The goal position robot is to be moved to
     *
     * @return Int with function execution status. -1 for call failure,
     *         0 when call is successful but position not reachable,
     *         1 when position is reached
     */
    int moveToPose(geometry_msgs::PoseStamped pose);

    /**
     * @brief Calls service to look for ArUco markers on toys
     *
     * @param toyID ID of the ArUco marker to be searched
     *
     * @return Int with function execution status. -1 for call failure, 0 when
     *         call is successful but toy not visible, 1 when toy visible
     */
    int lookForToy(int toyID);


    /**
     * @brief Calls service to move Tiago Base to Toy position
     *
     * @param None
     *
     * @return Int with function execution status. -1 for call failure,
     *         0 when call is successful but position not reachable,
     *         1 when position is reached
     */
    int goToToy();

    /**
     * @brief Calls service to move Tiago to Toy Storage Location
     *
     * @param None
     *
     * @return Int with function execution status. -1 for call failure,
     *         0 when call is successful but storage not reachable,
     *         1 when storage is reached
     */
    int goToStorage();

    /**
     * @brief search operation for scanning the area for ArUco markers  
     *
     * @param searchPose Pose of the next location to go to loook for toy
     *
     * @return Int with function execution status. -1 for call failure,
     *         0 when call is successful but position not reachable,
     *         1 when position is reached
     */
    int search(geometry_msgs::PoseStamped searchPose);

    /**
     * @brief Calls service to pickup Toy
     *
     * @param None
     *
     * @return Int with function execution status. -1 for call failure,
     *         0 when call is successful but toy could not be picked up,
     *         1 when toy is successfully picked up
     */
    int pickUpToy();

    /**
     * @brief Calls service to store toy in storage
     *
     * @param None
     *
     * @return Int with function execution status. -1 for call failure,
     *         0 when call is successful but toy could not be placed,
     *         1 when toy is successfully placed in storage
     */
    int storeToy();

    /**
     * @brief Shuts down ROS nodes
     *
     * @param None
     *
     * @return None
     */
    void shutdownRobot();

    /**
     * @brief Main method which switches from the current task to the next one 
     *        based on feedback from the robot.
     *
     * @param None
     *
     * @return None
     */
    int taskPlanner();


  private :
    // /* Map object which contains the list of tasks to be performed
    //  *        by the robot indexed by an integer key for each task */
    // std::map<int, std::string> taskList; 

    /* List of ArUco tag IDs to be picked */
    std::vector<int> toyIDs; 

    /* Create ROS node handle */
    ros::NodeHandle nh;

    /* Current pose of toy */
    geometry_msgs::PoseStamped toyPose;

    /* Pose of storge location */
    geometry_msgs::PoseStamped storagePose;

    /* List of way-points to traverse for exploration */
    std::vector<geometry_msgs::PoseStamped> searchPoses;

    /* Service Clients */
    ros::ServiceClient toyFoundClient, goalPoseClient;
};

#endif  // INCLUDE_TASKPLANNER_HPP_