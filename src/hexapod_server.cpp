#include <ros/ros.h>
#include <ros/console.h>
#include <hexa_control/Transfert.h>

#include <hexa_control/robotHexa.hpp>
#include <hexa_control/controllerDuty.hpp>

#include <boost/shared_ptr.hpp>
#include <sstream>

boost::shared_ptr<RobotHexa> hexapod_p;
ros::Publisher statepub;

bool transfert(hexa_control::Transfert::Request  &req,
               hexa_control::Transfert::Response &res )
{
  try
  {
    ROS_INFO_STREAM("Requested motion duration : " << req.duration);
    if(req.duration <= -2)
    {
      hexapod_p->relax();
      return true;
    }
    else if(req.duration == 0)
    {
      hexapod_p->reset();
      return true;
    }
    else if(req.duration == -1)
    {
      hexapod_p->position_zero();
      return true;
    }

    // Store and display the parameters of the oscillators

    std::vector<float> params;
    std::stringstream param_sstr;
    param_sstr << "Parameters received : ";
    for(int i=0;i<36;i++)
    {
      params.push_back(req.params[i]);
      param_sstr << params[i] << " ";
    }
    ROS_INFO_STREAM(param_sstr.str());

    // Instanciate the controller that generates the values for each oscillator.

    ControllerDuty ctrl(params, std::vector<int>());

    std_msgs::String msg;
    msg.data = "start";
    statepub.publish(msg);

    // Execute the experiment

    //hexapod_p->initial_pos();
    hexapod_p->transfer(ctrl,req.duration,0);

    msg.data = "stop";
    statepub.publish(msg);

    res.covered_distance = hexapod_p->covered_distance();
    ROS_INFO_STREAM("Experiment finished");

    return true;
  }
  catch (dynamixel::Error e)
  {
    ROS_ERROR_STREAM("(dynamixel): " << e.msg() << " " <<__FILE__<<" " << __LINE__);
    ROS_ERROR_STREAM("closing serials");
    hexapod_p->close_usb_controllers();
    exit(1);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hexapod_server");
  boost::shared_ptr<ros::NodeHandle> node_p= boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
  //boost::shared_ptr<dynamixel::Usb2Dynamixel controller> controller;
  try
  {
    hexapod_p = boost::shared_ptr<RobotHexa>(new RobotHexa);
  }
  catch (dynamixel::Error e)
  {
    ROS_ERROR_STREAM("(dynamixel): " << e.msg() << " " <<__FILE__<<" " << __LINE__);
    ROS_ERROR_STREAM("closing serials");
    hexapod_p->close_usb_controllers();
    return 1;
  }

  hexapod_p->initRosNode(argc,argv,node_p);
  ros::ServiceServer service = node_p->advertiseService("Transfert", transfert);
  statepub = node_p->advertise<std_msgs::String>("transfertState", 10);

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }

  ROS_INFO_STREAM("Ready to control the robot.");
  ros::spin();
  hexapod_p->relax();
  return 0;
}
