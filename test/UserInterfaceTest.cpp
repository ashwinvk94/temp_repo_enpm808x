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
 * @file        UserInterfaceTest.cpp
 * @author      Rohan Singh
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 1, 2019
 * @brief       Unit tests for class UserInterface
 */

#include <gtest/gtest.h>
#include <sstream>

#include "../include/TaskPlanner.hpp"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


/**
 * @brief Test to check getIDs functionality
 *
 * @param None
 *
 * @return none
 */
TEST(UserInterfaceClassTest, TestGetIDs) {
    UserInterface ui;
    std::vector<int> ids = ui.getIDs();

    ASSERT_EQ(9, ids.size());
}

/**
 * @brief Test to check getStorageLocation functionality
 *
 * @param None
 *
 * @return none
 */
TEST(UserInterfaceClassTest, TestLookForToy) {
    std::istringstream mockInputBuffer1("y");
    std::ostringstream mockOutputBuffer;
    UserInterface ui1(mockInputBuffer1, mockOutputBuffer);

    geometry_msgs::PoseStamped test1 = ui1.getStorageLocation();

    std::istringstream mockInputBuffer2("n -5 1 5 -1");
    std::ostringstream mockOutputBuffer;
    UserInterface ui2(mockInputBuffer2, mockOutputBuffer);

    geometry_msgs::PoseStamped test2 = ui2.getStorageLocation();

    ASSERT_EQ(test1.pose.position.x, 0);
    ASSERT_EQ(test1.pose.position.y, 0);
    ASSERT_EQ(test1.pose.position.z, 0);
    ASSERT_EQ(test1.pose.orientation.x, 0);
    ASSERT_EQ(test1.pose.orientation.y, 0);
    ASSERT_EQ(test1.pose.orientation.z, 0);
    ASSERT_EQ(test1.pose.orientation.w, 1.0);

    ASSERT_EQ(test2.pose.position.x, 1);
    ASSERT_EQ(test2.pose.position.y, -1);
    ASSERT_EQ(test2.pose.position.z, 0);
    ASSERT_EQ(test2.pose.orientation.x, 0);
    ASSERT_EQ(test2.pose.orientation.y, 0);
    ASSERT_EQ(test2.pose.orientation.z, 0);
    ASSERT_EQ(test2.pose.orientation.w, 1.0);
}
