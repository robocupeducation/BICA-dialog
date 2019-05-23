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

/* Author: Daniel Arévalo Rodrigo   */

/* Mantainer: Daniel Arévalo Rodrigo sirl.daniel.arevalo@hotmail.com */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <semaphore.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include "bica_dialog/DialogInterface.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>
#include "bica/Component.h"
#include <regex>

namespace tools
{
class Pinger: public bica::Component
{
  public:
    Pinger(): nh_("~")
    {
      errorPublisher = nh_.advertise<std_msgs::String>("/errors", 1, true);
    }

    int ping(){
        std_msgs::String msg;
        if (system("ping -c 2 www.google.es >/dev/null") != 0){
            msg.data = "network connection unavailable";
            errorPublisher.publish(msg);
            printf("[Pinger] Network status:  Not available \n");
            return 1;
        }
        printf("[Pinger] Network status:  Connected \n");
        return 0;
    }
  private:
    ros::NodeHandle nh_;
    ros::Publisher errorPublisher;
};
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "NetworkTester");
    tools::Pinger pinger;
    ros::Rate loop_rate(2);
    while(ros::ok()){
        if (pinger.isActive()){
            pinger.ping();
            ros::spinOnce();
        }
        loop_rate.sleep();
    }
    return 0;
}
