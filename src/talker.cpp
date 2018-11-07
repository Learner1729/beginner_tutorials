/**
 * MIT License
 *
 * Copyright (c) 2018 Ashish Patel
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
 */

/**
 * @file       talker.cpp
 * @author     Ashish Patel
 * @version    2.0
 * @copyright  MIT License (c) 2018 Ashish Patel
 *
 * @brief      ROS talker node
 *
 */
#include <sstream>

/**
 * Includes all the headers necessary to use the most common public pieces of
 * the ROS system
 */
#include "ros/ros.h"

/**
 * Includes the std_msgs/String message, which resides in the std_msgs package.
 * The header is generated automatically from the String.msg file in that
 * package.
 */
#include "std_msgs/String.h"

/**
 * Including C++ header file associated with the service which defines datatype
 * of request and response
 */
#include "beginner_tutorials/changeBaseString.h"

// Initializing the base string
std::string message = "Hello World!!";

/**
 * @brief changeBaseString
 * @param request: The request
 * @param response: The response
 * @return true, if everthings works properly
 */
bool changeBaseString(beginner_tutorials::changeBaseString::Request& request,
                    beginner_tutorials::changeBaseString::Response& response) {
  response.outputData = request.inputData;
  message = response.outputData;
  ROS_WARN_STREAM("Changed the Base string to: " << message);
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command 
   * line. For programmatic remappings you can use a different version of 
   * init() which takes remappings directly, but for most command-line programs
   * passing argc and argv is the easiest way to do it.  The third argument to
   * init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the 
   * last NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // initialize parameter variable
  auto frequency = 0;
  
  /** 
   * Accessing the value of the private parameter
   * The get function returns true if the value was read successfully and false
   * if there was a problem. Thus, if it returns false I am assigning a default
   * value to our parameter and display's a logging message.
   */
  bool ok = ros::param::get("~freq",frequency);
  if(!ok){
    ROS_ERROR_STREAM("Using the default frequency := 10");
    frequency = 10;
  }
  else {
    ROS_DEBUG_STREAM("Using frequency := " << frequency);
    if(frequency==0) {
      ROS_FATAL_STREAM("Frequency can't be zero");
      ros::shutdown();
    }
  }

  /**
   * The advertise() function is how you tell ROS that you want to publish on a
   * given topic name. This invokes a call to the ROS master node, which keeps 
   * a registry of who is publishing and who is subscribing. After this 
   * advertise() call is made, the master node will notify anyone who is trying
   * to subscribe to this topic name, and they will in turn negotiate a 
   * peer-to-peer connection with this node.  advertise() returns a Publisher
   * object which allows you to publish messages on that topic through a call
   * to publish().  Once all copies of the returned Publisher object are 
   * destroyed, the topic will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue used
   * for publishing messages.  If messages are published more quickly than we 
   * can send them, the number here specifies how many messages to buffer up
   * before throwing some away.
   */
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  /** 
   * Creating a server object. It takes two types of parameters, first is 
   * service_name it is a string name of the service that I have created and
   * second is the callback function
   */
  auto server = n.advertiseService("changeBaseString", changeBaseString);

  /**
   * A ros::Rate object allows us to specify a frequency that we would like to
   * loop at.
   */
  ros::Rate loop_rate(frequency);

  /**
   * A count of how many messages we have sent. This is used to create a unique
   * string for each message.
   */
  auto count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << message << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter is the
     * message object. The type of this object must agree with the type given
     * as a template parameter to the advertise<>() call, as was done in the
     * constructor above.
     */
    chatter_pub.publish(msg);

    /** 
     * Calling ros::spinOnce() here is not necessary for this simple program,
     * because we are not receiving any callbacks. However, if you were to 
     * add a subscription into this application, and did not have 
     * ros::spinOnce() here, your callbacks would never get called.
     */
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
  return 0;
}
