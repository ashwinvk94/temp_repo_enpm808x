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
 * @file        ToyDetection.hpp
 * @author      Abhinav Modi
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 1, 2019
 * @brief       Header file for ToyDetection module to detect and locate toys in the
 *              robot's base frame using ArUco markers present on the toys.
 */

#ifndef INCLUDE_TOYDETECTION_HPP_
#define INCLUDE_TOYDETECTION_HPP_

#include <iostream>
#include <vector>
#include "../include/ROSModule.hpp"
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "kids_next_door/toyFound.h"

class ToyDetection : public ROSModule {
 public:
  ToyDetection();
  /**
   * @brief detectArUco detects the ArUco marker in the current image frame
   *        and stores the position of the toy and its ID 
   *
   * @param currFrame Current image frame
   *
   * @return None
   */
  void initializeSubscribers();
  
  /**
   * @brief Publishes the pose of the detected toy(ArUco) 
   *
   * @param None
   *
   * @return None
   */
  void detectionCb();
  void initializePublishers();
  int detectArUco();

  /**
   * @brief Callback function for the camera feed ROS subscriber
   *
   * @param None
   *
   * @return None
   */
  void camFeedCb();

  void detectionCb(const std_msgs::Bool::ConstPtr& detectionFlag);
  bool findToySrv(kids_next_door::toyFound::Request& req,
                           kids_next_door::toyFound::Response& resp);

 private :
    
    /**
     * @brief tf listener to get value of transforms in tf
     */
    tf::TransformListener listener;
    
    /**
     * @brief tf tranform to store the transform
     */
    tf::StampedTransform transform;
    
    /**
     * @brief tag ID of the current toy being searched for
     */
    double currToyID;

    /**
     * @brief X coordinate of the tag wrt to the /base_footprint frame
     */
    double tagPosX;
    
    /**
     * @brief Y coordinate of the tag wrt to the /base_footprint frame
     */
    double tagPosY;
	
    geometry_msgs::PoseStamped tagPoseStamped;  

    ros::NodeHandle nh; 
    
    std_msgs::Bool detectionFlag;

    geometry_msgs::PoseStamped toyPose;
};

#endif  // INCLUDE_TOYDETECTION_HPP_
