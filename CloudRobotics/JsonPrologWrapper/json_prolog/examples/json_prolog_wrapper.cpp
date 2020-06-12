
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <json_prolog/prolog.h>
#include "std_msgs/String.h"

using namespace std;
using namespace json_prolog;

std_msgs::String global_Msg;


ros::Publisher chatter_pub;

void parse(const PrologBindings &bdg, const std::string &var_name){

  global_Msg.data = bdg[var_name].toString();
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    Prolog pl;

    PrologQueryProxy bdgs = pl.query(msg->data.c_str());
    for(PrologQueryProxy::iterator it=bdgs.begin();
      it != bdgs.end(); it++)
    {
      PrologBindings bdg = *it;
      parse(bdg, "A");
      chatter_pub.publish(global_Msg);
      ROS_INFO("I send: [%s]", global_Msg.data.c_str());
    }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bridge_json_prolog");

  Prolog pl;

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  chatter_pub = n.advertise<std_msgs::String>("response", 1000);
  
  ros::spin();
  

  return 0;
}
