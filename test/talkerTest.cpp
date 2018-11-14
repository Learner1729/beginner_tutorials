/**
 * @file       talkerTest.cpp
 * @author     Ashish Patel
 * @version    1.0
 * @copyright  MIT License (c) 2018 Ashish Patel
 *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE
 *
 * @brief ROS talkerTest file it test the service of talker node
 */

// Including gtest header file
#include <gtest/gtest.h>

// Including ros header file
#include <ros/ros.h>

// Including service client header file
#include <ros/service_client.h>

/**
 * Including C++ header file associated with the service which defines datatype
 * of request and response
 */
#include "beginner_tutorials/changeBaseString.h"

/**
 * Testing the talker node service
 */
TEST(TEST_TALKER, test_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<beginner_tutorials::changeBaseString>("changeBaseString");
  bool exists(client.waitForExistence(ros::Duration(2)));
  EXPECT_TRUE(exists);
  beginner_tutorials::changeBaseString srv;
  srv.request.inputData = "TEST";
  client.call(srv);
  EXPECT_EQ(srv.response.outputData, srv.request.inputData);
}
