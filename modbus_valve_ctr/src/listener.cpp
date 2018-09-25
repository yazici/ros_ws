#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int64.h>
#include "modbus.h"
#include "modbus_exception.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>

#define ip_address "192.168.2.184"

modbus mb = modbus(ip_address, 502);



void mySigintHandler(int sig)
{
  ros::shutdown();
}

void executionCallback(const std_msgs::Int64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%lli]", msg->data);
  using namespace std::this_thread;                   // sleep_for, sleep_until
  using namespace std::chrono;                        // nanoseconds, system_clock, seconds
  signal(SIGINT, mySigintHandler);
  ros::NodeHandle nh("~");
  ros::Publisher screw_pub = nh.advertise<std_msgs::Int64>("pos_trig", 1000);
  std_msgs::Int64 msgs;
  int count= 1;
  msgs.data = count;
  int XDKIO =0;
  nh.getParam("XDK", XDKIO);
  // uint16_t write_regs[3] = {1,2};

  switch(XDKIO){
  case 0:                                             // without using XDK
    switch(msg->data){
      case 1:
        mb.modbus_write_register(40003, 1);         // write single reg Nr1: unten links
        sleep_for(seconds(3));
        mb.modbus_write_register(40003, 3);//, write_regs);         // write single reg Nr2: oben links
        sleep_for(seconds(5));
        mb.modbus_write_register(40003, 1);         // write single reg Nr1: unten links
        std::cout << "test1 without XDK" << endl ;
        screw_pub.publish(msgs);                      // publish trigger for next position
        ros::spinOnce();
        break;
      case 2:
         mb.modbus_write_register(40003, 1);         // write single reg Nr1: unten links
        std::cout << "test2 without XDK" << endl ;
        break;
    }
    break;
  case 1:                                             // using XDK
    switch(msg->data){
      case 1:
        mb.modbus_write_register(40003, 1);         // write single reg Nr1: unten links
        sleep_for(seconds(3));
        mb.modbus_write_register(40003, 2);         // write single reg Nr2: oben links
        sleep_for(seconds(5));

          //  if (BoschXDK_trig ==1)
          //  screw_pub.publish(msg);
          //  ros::spinOnce();
        mb.modbus_write_register(40003, 1);         // write single reg Nr1: unten links
        std::cout << "test1 using XDK" << endl ;
        screw_pub.publish(msgs);
        ros::spinOnce();
        break;
      case 2:
        mb.modbus_write_register(40003, 1);         // write single reg Nr1: unten links
        std::cout << "test2 using XDK" << endl ;
        break;
    }
    break;

  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  mb.modbus_set_slave_id(1);
  mb.modbus_connect();

  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  ros::Subscriber sub = n.subscribe("valve_trig", 1000, executionCallback);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  mb.modbus_close();
  //delete(&mb);
  return 0;
}
