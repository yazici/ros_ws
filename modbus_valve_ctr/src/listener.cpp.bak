#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int64.h>
#include "modbus.h"
#include "modbus_exception.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>


// using namespace std::this_thread; // sleep_for, sleep_until
// using namespace std::chrono; // nanoseconds, system_clock, seconds


void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
   // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void executionCallback(const std_msgs::Int64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%lli]", msg->data);

    //ros::init(argc, argv, "my_node_name", ros::init_options::NoSigintHandler);

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);


    // create a modbus object
    modbus mb = modbus("10.42.0.157", 502);


    // set slave id
    mb.modbus_set_slave_id(1);

    // connect with the server
    mb.modbus_connect();



    // read holding registers           function 0x03
    // uint16_t read_holding_regs[1];
    // mb.modbus_read_holding_registers(40003, 1, read_holding_regs);{
    // std::cout << *read_holding_regs << "  test" << endl;

//while(true){
    //if(*read_holding_regs==0){
    mb.modbus_write_register(40003, 1);         // write single reg
    // Nr1: unten links
    // Nr2:
    std::this_thread::sleep_for(std::chrono::seconds(1));
    mb.modbus_write_register(40003, 2);         // write single reg
    // Nr1: unten links
    // Nr2:
    std::this_thread::sleep_for(std::chrono::seconds(1));
    //}                     // wait 1 Second
    //mb.modbus_write_register(40003, 2);         // write single reg
    // mb.modbus_read_holding_registers(40003, 1, read_holding_regs);
    // std::cout << *read_holding_regs << endl;
//}


    // close connection and free the memory
    mb.modbus_close();
    delete(&mb);
    ros::spin();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  std::cout << "test" << endl ;

  ros::Subscriber sub = n.subscribe("valve_trig", 1000, executionCallback);

  ros::spin();

  return 0;
}
