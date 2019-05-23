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
        // msg.data = "network connection unavailable";
        // errorPublisher.publish(msg);
        if (system("ping -c 2 www.google.es") != 0){
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
    while(ros::ok()){
        if (pinger.isActive()){
            pinger.ping();
            ros::spinOnce();
        }
    }
    return 0;
}
