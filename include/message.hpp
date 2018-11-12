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
 * @file        message.hpp
 * @author      Ashish Patel
 * @version     1.0
 * @copyright   MIT License (c) 2018 Ashish Patel
 *
 * @brief       A service header file to change the base string
 */

#ifndef INCLUDE_MESSAGE_HPP_
#define INCLUDE_MESSAGE_HPP_

// including string C++ header file
#include <string>

/**
 * Including C++ header file associated with the service which defines datatype
 * of request and response
 */
#include "beginner_tutorials/changeBaseString.h"

class Message {
 public:
  /**
   * @brief changeBaseString function
   * @param request: The request
   * @param response: The response
   * @return true, if everthings works properly
   */
  bool changeBaseString(
      beginner_tutorials::changeBaseString::Request& request,
      beginner_tutorials::changeBaseString::Response& response) {
    response.outputData = request.inputData;
    msg_ = response.outputData;
    // WARN changed the Base string
    ROS_WARN_STREAM("Changed the Base string");
    return true;
  }

  /**
   * @brief getMessage function
   * @param none
   * @return string
   */
  std::string getMessage() const {
    return msg_;
  }

 private:
  std::string msg_{"Hello World!!"};
};

#endif  // INCLUDE_MESSAGE_HPP_
