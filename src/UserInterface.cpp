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
 * @file        UserInterface.cpp
 * @author      Rohan Singh
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 9, 2019
 * @brief       Function definotion for UserInterface Class
 */

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "UserInterface.hpp"

UserInterface::UserInterface() :
            inputStream(std::cin), outputStream(std::cout) {}

UserInterface::UserInterface(std::istream& ipStream, std::ostream& opStream) :
            inputStream(ipStream), outputStream(opStream) {}

UserInterface::~UserInterface() {}

geometry_msgs::PoseStamped UserInterface::getStorageLocation() {
    int flag = 1;
    float x, y;
    std::string ans;
    geometry_msgs::PoseStamped storageLocation;
    storageLocation.pose.position.x = 0;
    storageLocation.pose.position.y = 0;
    storageLocation.pose.position.z = 0;
    storageLocation.pose.orientation.x = 0;
    storageLocation.pose.orientation.y = 0;
    storageLocation.pose.orientation.z = 0;
    storageLocation.pose.orientation.w = 1.0;
        
    while (flag == 1) {
        outputStream << "Use start position as storage point ? (y/n)";
        inputStream >> ans;
        if (ans == "y") {
            flag = 0;
        } else if (ans == "n") {
            bool xPosFlag = true;
            bool yPosFlag = true;
            while (xPosFlag) {
                outputStream << "Enter position in x-coordinate between (-4,4) : ";
                inputStream >> x;
                if ((x <= 4.0) && (x >= -4.0)) {
                    storageLocation.pose.position.x = x;
                    xPosFlag = false;
                }
            }
            while (yPosFlag) {
                outputStream << "Enter position in y-coordinate between (-11,1) : ";
                inputStream >> y;
                if ((y <= 1.0) && (y >= -11.0)) {
                    storageLocation.pose.position.y = y;
                    yPosFlag = false;
                }
            }
            flag = 0;
        } else {
            outputStream << "Enter correct choice." << std::endl;
        }
    }
    return storageLocation;
}

std::vector<int> UserInterface::getIDs() {
    std::vector<int> toyIDs;
    /* Initializing list of toy IDs (constant for demo) */
    toyIDs.push_back(0);
    toyIDs.push_back(0);
    toyIDs.push_back(0);
    toyIDs.push_back(1);
    toyIDs.push_back(1);
    toyIDs.push_back(1);
    toyIDs.push_back(2);
    toyIDs.push_back(2);
    toyIDs.push_back(2);

    return toyIDs;
}