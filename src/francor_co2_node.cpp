/*
 *  Created on: 26.04.2018
 *      Author: feesmrt
 *      E-Mail: martin.fees@posteo.de
 *
 * BSD 3-Clause License
 * Copyright (c) 2018, FRANC0R - Franconian Open Robotics
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ros/ros.h>
#include <std_msgs/UInt16.h>

#include "francor_co2/CanInterface.h"
#include "CO2DataMsg.h"

francor::motor_controller::CanInterface g_can;
ros::Publisher  g_co2_pub;

void callbackReceiveCan(const francor::motor_controller::CanMsg& msg) {
    francor::CO2DataMsg co2_can_msg;

    // Check for CAN ID
    if(msg.getID() == 0x701) {
      // get raw data from can
      const auto can_raw_data = msg.getData();

      // copy to local variable
      memcpy(co2_can_msg._raw_data, can_raw_data.data(), 8);

      // conver to ros message
      std_msgs::UInt16  ros_data;
      ros_data.data = co2_can_msg._co2_ppm;

      g_co2_pub.publish(ros_data);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "co2_node");

  ros::NodeHandle privateNh("~");

  g_can.initialize("can0", callbackReceiveCan);
  g_can.start();

  g_co2_pub = privateNh.advertise<std_msgs::UInt16>("co2", 100);

  ros::spin();

  g_can.stop();

  return 0;
}