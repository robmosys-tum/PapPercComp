
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <json_prolog/prolog.h>
#include "std_msgs/String.h"

using namespace std;
using namespace json_prolog;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    Prolog pl;
    PrologQueryProxy bdgs = pl.query(msg->data.c_str());
    for(PrologQueryProxy::iterator it=bdgs.begin();
      it != bdgs.end(); it++)
    {
      PrologBindings bdg = *it;
      cout << "A = "<< bdg["A"] << endl;
      cout << "B = " << bdg["B"] << endl;
      cout << "C = " << bdg["C"] << endl;
    }
    //TODO(jkabalar): send this output back to robot control program
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bridge_json_prolog");

  Prolog pl;

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  
  ros::spin();
  

  return 0;
}
