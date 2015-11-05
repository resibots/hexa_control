#ifndef HEXA_CONTROL_ROBOTHEXA_HPP
#define HEXA_CONTROL_ROBOTHEXA_HPP

#include <stdlib.h>
#include <math.h>
#include <dynamixel/dynamixel.hpp>
#include <hexa_control/controllerDuty.hpp>

#ifdef IMU
#include <imu_razor/imu_razor.hpp>
#endif

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
//#include <rgbdslam/XtionDisplacement.h>
//#include <rgbdslam/SferesCmd.h>


#define DEFAULT_SPEED 150
#define READ_DURATION 0.005f
#define SAMPLING_FREQUENCY 20

class RobotHexa
{
public:
  typedef unsigned char byte_t;

  RobotHexa()
  {
    init();
  }
  ~RobotHexa()
  {
    relax();
  }

  void init();

  void relax()
  {
    ROS_INFO_STREAM("relax...");
    for (size_t i = 0; i < _actuators_ids.size(); ++i)
    {
      _controller.send(dynamixel::ax12::TorqueEnable(_actuators_ids[i], false));
      _controller.recv(READ_DURATION, _status);
    }
    for (size_t i = 0; i < _wheels_ids.size(); ++i)
    {
      _controller.send(dynamixel::ax12::TorqueEnable(_wheels_ids[i], false));
      _controller.recv(READ_DURATION, _status);
    }

    ROS_INFO_STREAM("done");
  }
  void enable()
  {
    for (size_t i = 0; i < _actuators_ids.size(); ++i)
    {
      try {
        _controller.send(dynamixel::ax12::TorqueEnable(_actuators_ids[i], true));
        _controller.recv(READ_DURATION, _status);
      }
      catch (dynamixel::Error e){
        ROS_ERROR_STREAM("dynamixel (enable) id =" << (int)_actuators_ids[i] << " " << e.msg() << " " <<__FILE__<<" " << __LINE__);
      }
    }

    usleep(1e5);
  }

  void reset();
  void position_zero();
  void transfer(ControllerDuty& controller, float duration,int transfer_number);
  size_t nb_actuators() const
  {
    return _actuators_ids.size();
  }
  size_t nb_wheels() const
  {
    return _wheels_ids.size();
  }
  void close_usb_controllers()
  {
#ifdef IMU
    _imu.close_serial();
#endif
    _controller.close_serial();
  }


  const std::vector<float>& get_contact(int i)
  {
    switch (i)
    {
    case 0:
      return _behavior_contact_0;
      break;
    case 1:
      return _behavior_contact_1;
      break;
    case 2:
      return _behavior_contact_2;
      break;
    case 3:
      return _behavior_contact_3;
      break;
    case 4:
      return _behavior_contact_4;
      break;
    case 5:
      return _behavior_contact_5;
      break;

    }
    assert(false);
    return _behavior_contact_0;



  }
  void  initRosNode(  int argc ,char** argv,  boost::shared_ptr<ros::NodeHandle> node_p);
  std::vector<std::vector<float> >& get_angles()
  {


    return _imu_angles;

  }


  //void send_ros_start(int nbrun,int nbtrans);
  // void send_ros_stop(int nbrun,int nbtrans);
  //void posCallback(const rgbdslam::XtionDisplacement& msg);
  void posCallback(const nav_msgs::Odometry& msg);
  float covered_distance()
  {
    return _covered_distance;
  }

  //std::vector<float> final_pos(){return _final_pos;}
  float final_angle(){return _final_angle;}

protected:

  void setPID();
  void applyCorrection(std::vector<int>& pos);
  void getSlamInfo();
  void write_contact(std::string const name);
  void write_angle(std::string const name);
  void contactSmoothing(int length);
  void _read_contacts();
  dynamixel::Usb2Dynamixel _controller;
  float _covered_distance;

#ifdef IMU
  imu::Usb2Imu _imu;
#endif
  tf::Transform _final_pos;
  tf::Transform _prev_pos;
  float _final_angle;
  std::vector<std::vector<float> > _imu_angles;
  ros::Time _request_time;
  dynamixel::Status _status;
  std::vector<byte_t> _wheels_ids;
  std::vector<int> _correction;
  std::vector<float> _behavior_contact_0;
  std::vector<float> _behavior_contact_1;
  std::vector<float> _behavior_contact_2;
  std::vector<float> _behavior_contact_3;
  std::vector<float> _behavior_contact_4;
  std::vector<float> _behavior_contact_5;
  //std::vector<float> _behavior_smooth_contact;
  std::vector<byte_t> _actuators_ids;
  boost::shared_ptr<ros::NodeHandle> _node_p;
  ros::Publisher  _chatter_pub;
  ros::Subscriber _sub;
  ros::Publisher _reset_filter_pub;

  std::string _serial_port, _odom;
  int _serial_baudrate, _baudrate_choice;
};

#endif
