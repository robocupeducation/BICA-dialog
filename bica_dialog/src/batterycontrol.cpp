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
#include <istream>
#include <fstream>
using namespace std;

namespace tools
{
class Checker
{
  public:
    Checker(): nh_("~")
    {
      talkPublisher = nh_.advertise<std_msgs::String>("/talk", 1, true);
    }

    int check(){
        std_msgs::String msg;
        system("upower -i $(upower -e | grep BAT) | grep --color=never -E 'percentage'|sed 's/percentage:/''/'|sed 's/%//g'|sed 's/ //g' > state");
        ifstream ficheroEntrada;
        string state;
        ficheroEntrada.open ("state");
        getline(ficheroEntrada, state);
        ficheroEntrada.close();
        printf("[Checker] %i \n", atoi( state.c_str() ));
        if (atoi( state.c_str() ) <= 15){
            msg.data = "Laptop battery is low, please plug it";
            talkPublisher.publish(msg);
        }
        return 0;
    }
  private:
    ros::NodeHandle nh_;
    ros::Publisher talkPublisher;
};
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "BatteryCkecker");
    tools::Checker checking;
    ros::Rate loop_rate(0.0166);
    while(ros::ok()){
        checking.check();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
