#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>

int main(int argc, char **argv){

   ros::init(argc, argv, "talkpub");
   ros::NodeHandle n;

   ros::Publisher test= n.advertise<std_msgs::String>("/talk", 1 );

   std_msgs::String msg;
   msg.data = "This is a trial";

  ros::Rate loop_rate(10);
   while(ros::ok()){
      test.publish(msg);
      loop_rate.sleep();
   }
   return 0;
}
