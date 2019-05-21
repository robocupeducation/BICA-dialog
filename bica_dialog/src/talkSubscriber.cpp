#include <ros/ros.h>
#include "bica_dialog/DialogInterface.h"
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>

void messageCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Recibido");
    bica_dialog::DialogInterface di(msg -> data);
    di.speak(msg -> data);
}

int main(int argc, char **argv)
{

   ros::init(argc, argv, "talksub");
   ros::NodeHandle n;

   // Construimos un subscriptor y le indicamos a que topic  se tiene que suuscribir
   ros::Subscriber sub = n.subscribe("/talk", 1, messageCallback);

   ros::spin();

   return 0;
}
