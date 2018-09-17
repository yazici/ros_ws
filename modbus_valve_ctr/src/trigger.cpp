#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int64.h>
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trigger");

  ros::NodeHandle n;

  ros::Publisher trigger_pub = n.advertise<std_msgs::Int64>("valve_trig", 1000);

  ros::Rate loop_rate(30);


  while (ros::ok())
  {
    std_msgs::Int64 msg;
    msg.data = 1;
    ROS_INFO("%d", msg.data);
    trigger_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    }


  return 0;
}