#include "ros/ros.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <ctime>


namespace global
{
  ros::Subscriber subpos;
  ros::Subscriber subconsigne;
  boost::shared_ptr<ros::NodeHandle> node_p;
  boost::shared_ptr<std::ofstream> out_p;
};


void posCallback(const nav_msgs::Odometry& msg)
{
  tf::Transform temp;
  tf::poseMsgToTF(msg.pose.pose,temp);
  (*  global::out_p)<<msg.header.stamp.toSec()<<" "<<temp.getOrigin()[0]<<" "<<temp.getOrigin()[1]<<" "<<temp.getOrigin()[2]<<std::endl;
  return;

}

void stateCallback(const std_msgs::String& msg)
{
  if(msg.data=="start")
  {
    global::subpos=global::node_p->subscribe("vo",1,posCallback);
    char date[30];
    time_t date_time;
    time(&date_time);
    strftime(date, 30, "%Y-%m-%d_%H_%M_%S", localtime(&date_time));
    std::string name("traj_");
    name+=date;
    name+=+".dat";
    global::out_p=boost::shared_ptr< std::ofstream> (new std::ofstream(name.c_str()));
  }
  else
  {
    global::subpos.shutdown();
    global::out_p.reset();
  }
  return;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_traj");

  global::node_p=boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());


  global::subconsigne=global::node_p->subscribe("transfertState",1,stateCallback);

  ros::spin();

 return 0;
}
