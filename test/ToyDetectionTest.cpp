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
 * @file        ToyDetectionTest.cpp
 * @author      Rohan Singh
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 1, 2019
 * @brief       Unit tests for class ToyDetection
 */

#include <gtest/gtest.h>

#include "ToyDetection.hpp"

/**
 * @brief Test to check toyFound Service
 */
TEST(ToyDetectionClassTest, FindToyServiceTest) {
	// create node handle object
    ros::NodeHandle n;
	ToyDetection detect;
    ros::ServiceClient findToyClient = 
    		n.serviceClient<kids_next_door::toyFound>("/knd/toyFound");
    bool exists = findToyClient.waitForExistence(ros::Duration(5));
    ASSERT_TRUE(exists);
}

/**
 * @brief Test to check toyPositionPub functionality
 *
 * @param none
 *
 * @return none
 */
TEST(ToyDetectionClassTest, detectArucoTest) {
	// should return no detections found
	ToyDetection detect;
	int out = detect.detectArUco();
	ASSERT_EQ(out,0);
}

/**
 * @brief Test to check successful initialization of the module
 */
TEST(ToyDetectionClassTest, SuccessfulInitialization) {
	EXPECT_NO_FATAL_FAILURE(ToyDetection detect);
}