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
 * @file        TaskPlannerTest.cpp
 * @author      Rohan Singh
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 1, 2019
 * @brief       Unit tests for class TaskPlanner
 */

#include <gtest/gtest.h>
#include <sstream>

#include "../include/TaskPlanner.hpp"
#include "../include/UserInterface.hpp"
#include "ros/ros.h"
#include "ros/service_client.h"

/**
 * @brief Test to main Node functionality
 *
 * @param none
 *
 * @return none
 */
TEST(TaskPlannerClassTest, TestMainNode) {
    std::istringstream mockInputBuffer("y");
    std::ostringstream mockOutputBuffer;
    TaskPlanner tp(mockInputBuffer, mockOutputBuffer);
    ASSERT_EQ(tp.taskPlanner(), 1);
}

/**
 * @brief Test to check moveTo functionality
 *
 * @param none
 *
 * @return none
 */
TEST(TaskPlannerClassTest, TestMoveToPose) {
    std::istringstream mockInputBuffer("y");
    std::ostringstream mockOutputBuffer;
    TaskPlanner tp(mockInputBuffer, mockOutputBuffer);
    geometry_msgs::PoseStamped test1;
    ASSERT_EQ(tp.moveToPose(test1), -1);
}

/**
 * @brief Test to check lookForToy functionality
 *
 * @param none
 *
 * @return none
 */
TEST(TaskPlannerClassTest, TestLookForToy) {
    std::istringstream mockInputBuffer("y");
    std::ostringstream mockOutputBuffer;
    TaskPlanner tp(mockInputBuffer, mockOutputBuffer);
    ASSERT_EQ(tp.lookForToy(0), -1);
}

