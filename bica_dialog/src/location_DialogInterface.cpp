/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jonatan Gines jginesclavero@gmail.com */

/* Mantainer: Jonatan Gines jginesclavero@gmail.com */

#include "bica_dialog/DialogInterface.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>

namespace bica_dialog
{
class ForwarderDF: public bica_dialog::DialogInterface
{
  public:
    ForwarderDF(std::regex intent): nh_(), DialogInterface(intent)
    {
      trigger_sub_ = nh_.subscribe("/listen", 1, &ForwarderDF::triggerCallback, this); //public on /listen to start listening
      locationPublisher = nh_.advertise<std_msgs::String>("/locations", 1);
    }

    void listenCallback(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[ForwarderDF] listenCallback: intent %s", result.intent.c_str());
      result_ = result;
      std_msgs::String msg;
      for (int i = 0; i<result.parameters.size();i++){
        for (int j = 0; j<result.parameters[i].value.size();j++){
          ROS_INFO("[ForwarderDF] listenCallback:value %s", result.parameters[i].value[j].c_str());
          msg.data = result.parameters[i].value[j];
          locationPublisher.publish(msg);
          ros::spinOnce();
        }
      }
    }

    void triggerCallback(const std_msgs::Empty::ConstPtr& msg)
    {
      listen();
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber trigger_sub_;
    ros::Publisher locationPublisher;
    dialogflow_ros_msgs::DialogflowResult result_;
};
}  // namespace bica_dialog

int main(int argc, char** argv)
{

  ros::init(argc, argv, "locationDialogflowNode");

  std::regex intent_in("[[:print:]_]*.location");
  bica_dialog::ForwarderDF forwarder(intent_in);
  while(ros::ok()){
      forwarder.listen();
      ros::spinOnce();
  }
  return 0;
}
