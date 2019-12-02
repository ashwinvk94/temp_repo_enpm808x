/******************************************************************************
 *  MIT License
 *
 *  Copyright (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 

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

/**@file toyDetection.hpp
 * @brief Header file for ToyDetection module to detect and locate toys in the
 *        robot's base frame using ArUco markers present on the toys.
 *
 * Detailed description follows here.
 * @author     : Abhinav Modi
 * @created on : Dec 1, 2019
 */
#ifndef INCLUDE_TOYDETECTION_HPP_
#define INCLUDE_TOYDETECTION_HPP_

#include <iostream>
#include <vector>
#include <opencv/core/core.hpp>
#include <opencv/highgui/highgui.hpp>
#include <opencv/imgproc/imgproc.hpp>
#include "ros/ros.h"

class ToyDetction {
 public:
    /**
     * @brief detectArUco detects the ArUco marker in the current image frame
     *        and stores the position of the toy and its ID 
     * @param currFrame - current image frame
     * @return None
     */
    void detectArUco(cv::Mat currFrame);
    
    /**(MAYBE NOT REQUIRED)
     * @brief toyPositionPublisher publishes the pose of the detected toy(ArUco) 
     * @param None
     * @return None
     */
    void toyPositionPublisher();

    /**
     * @brief camFeedCb is the callback function for the camera feed ROS 
     *        subscriber
     * @param None
     * @return None
     */
    void camFeedCb();

 private :
    /**
     * @brief vector containing IDs of the toys detected
     */
    std::vector<int> toyIDs;

    /**
     * @brief vector containing Pose msgs for the detected toys
     */
    std::vector<geometry_msgs::Pose> positions;
    
    /**
     * @brief OpenCV container for the current image frame
     */
    cv::Mat currFrame;
}

#endif  // INCLUDE_TOYDETECTION_HPP_