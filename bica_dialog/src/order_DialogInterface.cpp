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

#include "bica/Component.h"
#include "bica_dialog/DialogInterface.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>
#include <regex>

namespace bica_dialog
{
class ForwarderDF: public bica_dialog::DialogInterface,public bica::Component
{
  public:
    ForwarderDF(std::regex intent): nh_(), DialogInterface(intent)
    {
      trigger_sub_ = nh_.subscribe("/listen", 1, &ForwarderDF::triggerCallback, this); //public on /listen to start listening
      orderPublisher = nh_.advertise<std_msgs::String>("/orders", 1);
      finishSpeakPublisher = nh_.advertise<std_msgs::Empty>("/finish_speak", 1);
      errorPublisher = nh_.advertise<std_msgs::String>("/errors", 1);
    }

    void listenCallback(dialogflow_ros_msgs::DialogflowResult result){
        std::regex filter ("(.*)(.order)");
        if (result.query_text != ""){
            ROS_INFO("[ForwarderDF] listenCallback: intent %s", result.intent.c_str());
            if(regex_match(result.intent, filter)){
                result_ = result;
                std_msgs::String msg;
                for (int i = 0; i<result.parameters.size();i++){
                    for (int j = 0; j<result.parameters[i].value.size();j++){
                        ROS_INFO("[ForwarderDF] listenCallback:value %s", result.parameters[i].value[j].c_str());
                        if ( result.parameters[i].value[j] != ""){
                            msg.data = result.parameters[i].value[j];
                            orderPublisher.publish(msg);
                            ros::spinOnce();
                        }
                    }
                }
            }else{
                speak("I didn't catch you, can you repeat it please?");
                attemp++;
                if(attemp == 3){
                    ROS_INFO("[ForwarderDF] number of failed attemps %i", attemp);
                    std_msgs::String msg;
                    msg.data = "unknown command";
                    errorPublisher.publish(msg);
                    attemp = 0;
                }
            }
        }
    }

    void listener(){
        ros::Rate loop_rate(2);
        while(ros::ok()){
            if (isActive()){
                ROS_INFO("listening...");
                listen();
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void triggerCallback(const std_msgs::Empty::ConstPtr& msg)
    {
      listen();
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber trigger_sub_;
    ros::Publisher orderPublisher;
    ros::Publisher finishSpeakPublisher;
    ros::Publisher errorPublisher;
    dialogflow_ros_msgs::DialogflowResult result_;
    int attemp = 0;
};
}  // namespace bica_dialog

int main(int argc, char** argv)
{

  ros::init(argc, argv, "order_DialogInterface");

  std::regex intent_in("[[:print:]_]*.order");
  bica_dialog::ForwarderDF forwarder(intent_in);
  forwarder.listener();
  return 0;
}
