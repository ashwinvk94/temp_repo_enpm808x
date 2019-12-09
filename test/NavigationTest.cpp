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
 * @file        NavigationTest.cpp
 * @author      Rohan Singh
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 1, 2019
 * @brief       Unit tests for class Navigation
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "Navigation.hpp"

/**
 * @brief Test to check moveBasePub functionality
 *
 * @param none
 *
 * @return none
 */
TEST(NavigationClassTest, TestMoveToService) {
    // create node handle object 
    ros::NodeHandle n;
    auto testClient = n.ServiceClient<Navigation::moveToSrv>("/knd/moveTo");
    bool exists(testClient.waitForExistence(ros::Duration(5)));
    EXPECT_TRUE(exists);
}

/**
 * @brief Test to check localizeCb functionality
 *
 * @param none
 *
 * @return none
 */
TEST(NavigationClassTest, TestLocalizeCb) {

}

int main(int argc, char ** argv) {
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "kndTest");
	return RUN_ALL_TESTS();
}