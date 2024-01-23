/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Robotiq, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of Robotiq, Inc. nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2014, Robotiq, Inc
 */

/**
 * \file rq_test_sensor.cpp
 * \date July 14, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Jean-Philippe Roberge <ros@robotiq.com>
 */

#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "rq_tare_sensor");

  ros::NodeHandle n;

  std::string topic_name = "/robotiq_ft_sensor/robotiq_ft_sensor_acc";

  ros::ServiceClient client =
      n.serviceClient<robotiq_ft_sensor::sensor_accessor>(topic_name);

  robotiq_ft_sensor::sensor_accessor srv;
  srv.request.command_id = srv.request.COMMAND_SET_ZERO;

  if (client.call(srv)) {
    ROS_INFO("ret: %s", srv.response.res.c_str());
  } else {

    ROS_ERROR_STREAM("Failed to send command on topic: " << topic_name);
  }

  return 0;
}
