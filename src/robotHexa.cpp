#include <iostream>
#include <limits>
#include <fstream>
#include <cmath>

#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <hexa_control/robotHexa.hpp>

bool msg_recv;

/*void RobotHexa :: posCallback(const rgbdslam::XtionDisplacement& msg)
  {
  msg_recv=true;

  ROS_INFO("Commandes recues: \nX:%f \n Y:%f \n Z:%f \n duraction: %f  ",msg.x,msg.y,msg.z,msg.duration);
  _covered_distance=round(msg.x * 100) / 100.0f;
  //2-sqrt((2-msg.x)*(2-msg.x)+(0-msg.y)*(0-msg.y)+(0-msg.z)*(0-msg.z));
  _final_pos.resize(2);
  _final_pos[0]=-msg.y;
  _final_pos[1]=msg.x;

  _final_angle=msg.z*180/M_PI;

  _slam_duration = msg.duration;
  ROS_INFO("distance parcourue: %f",_covered_distance);
  }*/



void RobotHexa :: posCallback(const nav_msgs::Odometry& msg)
{

  ROS_INFO_STREAM(__FILE__<<"  "<<__LINE__);

  ros::Duration tdiff = this->_request_time - msg.header.stamp;
  ROS_INFO_STREAM("reception delay: "<<tdiff.toSec()<<" sec");

  if(tdiff <= ros::Duration(0,0))// responce received after end of movement.
  {
    ROS_DEBUG_STREAM("odometry message after end of movement; processing");
    msg_recv = true;
  }
  else
  {
    ROS_DEBUG_STREAM("too early the message is; keeping the loop running");
    return;
  }

  tf::Transform temp;
  tf::poseMsgToTF(msg.pose.pose, temp);
  ROS_INFO_STREAM("rcv message:" << msg);
  _final_pos = _prev_pos.inverse()*temp;
  ROS_INFO_STREAM("Temp position: \nX:"<<temp.getOrigin()[0]<<" \n Y:"<<temp.getOrigin()[1]<<" \n Z:"<<temp.getOrigin()[2]<<"\n");
  ROS_INFO_STREAM("Start position: \nX:"<<_prev_pos.getOrigin()[0]<<" \n Y:"<<_prev_pos.getOrigin()[1]<<" \n Z: "<<_prev_pos.getOrigin()[2]<<"\n");
  _prev_pos = temp;
  ROS_INFO_STREAM("move performed: \nX:"<<_final_pos.getOrigin()[0]<<" \n Y:"<<_final_pos.getOrigin()[1]<<" \n Z:"<<_final_pos.getOrigin()[2]<<"\n");
  // Take the distance on XZ plane as the traveled distance, to account for
  // the angle betwee the floor and the RGB-D camera (Xtion).
  _covered_distance = hypot(_final_pos.getOrigin()[0], _final_pos.getOrigin()[2]);
  _covered_distance = round(_covered_distance*100)/100.0f;
  ROS_INFO_STREAM("distance traveled: "<<_covered_distance);
  /*

    if(_prev_pos.size()==0)
    {
    _prev_pos.push_back(msg.pose.pose.position.x);
    _prev_pos.push_back(msg.pose.pose.position.y);
    _prev_pos.push_back(msg.pose.pose.position.z);

    }
    else
    {
    _final_pos.clear();
    _final_pos.push_back(msg.pose.pose.position.x-_prev_pos[0]);
    _final_pos.push_back(msg.pose.pose.position.y-_prev_pos[1]);
    _final_pos.push_back(msg.pose.pose.position.z-_prev_pos[2]);

    _prev_pos.clear();
    _covered_distance=round(_final_pos[0]*100)/100.0f;
    ROS_INFO("distance parcourue: %f",_covered_distance);
    }*/
  return;

}


void RobotHexa :: init()
{
  ros::NodeHandle n_p("~");
  // Load Server Parameters
  n_p.param("SerialPort", _serial_port, std::string("/dev/ttyACM0"));
  n_p.param("SerialBaudrate", _serial_baudrate, B1000000);
  n_p.param("Odom", _odom, std::string("/odometry/filtered"));

  try
  {
    // _controller.open_serial("/dev/ttyACM0",B1000000); // FIXME: use parameters instead
    // _controller.open_serial("/dev/ttyUSB0", B1000000);//B115200); // FIXME: use parameters instead
    _controller.open_serial(_serial_port, _serial_baudrate);

    // Scan actuators IDs
    _controller.scan_ax12s();
    const std::vector<byte_t>& ax12_ids = _controller.ax12_ids();
    if (!ax12_ids.size())
    {
      ROS_ERROR_STREAM("[ax12] no ax12 detected");
      return;
    }
    ROS_INFO_STREAM("[dynamixel] "<< ax12_ids.size()
	            << " dynamixel are connected");

    // Set ids of the actuators
    // Order : leg 1 [limb 1, limb 2, limb 3], leg 2 [limb 1, limb 2, limb 3], etc.
    // front-right
    _actuators_ids.push_back(4);
    _actuators_ids.push_back(14);
    _actuators_ids.push_back(24);

    // middle-right
    _actuators_ids.push_back(5);
    _actuators_ids.push_back(15);
    _actuators_ids.push_back(25);

    // back-right
    _actuators_ids.push_back(6);
    _actuators_ids.push_back(16);
    _actuators_ids.push_back(26);

    // back-left
    _actuators_ids.push_back(1);
    _actuators_ids.push_back(11);
    _actuators_ids.push_back(21);

    // middle-left
    _actuators_ids.push_back(2);
    _actuators_ids.push_back(12);
    _actuators_ids.push_back(22);

    // front-left
    _actuators_ids.push_back(3);
    _actuators_ids.push_back(13);
    _actuators_ids.push_back(23);


    ROS_INFO_STREAM("initialisation completed");
  }
  catch (dynamixel::Error e)
  {
    ROS_ERROR_STREAM("(dynamixel): " << e.msg());
  }

#ifdef IMU
  try
  {

    _imu.open_serial("/dev/ttyUSB1");

  }
  catch (imu::Error e)
  {
    ROS_ERROR_STREAM("(imu): " << e.msg());
  }

#endif
  // motor position correctio (offset)
  _correction=std::vector<int>(18,0);
  // _correction[0] = -550;
  _correction[0] = -256;
  _correction[1] = 0;
  _correction[2] = 1024;

  _correction[3] = 0;
  _correction[4] = 0;
  _correction[5] = 1024;

  // _correction[6] = 550;
  _correction[6] = 256;
  _correction[7] = 0;
  _correction[8] = 1024;

  // _correction[9] = -550;
  _correction[9] = -256;
  _correction[10] = 0;
  _correction[11] = 1024;

  _correction[12] = 0;
  _correction[13] = 0;
  _correction[14] = 1024;

  // _correction[15] = 550;
  _correction[15] = 256;
  _correction[16] = 0;
  _correction[17] = 1024;

  //  setPID();

}

void RobotHexa::setPID()
{

    // no compliance on ax18
  int cw_compliance_margin = 1;
  int ccw_compliance_margin = 1;
  int cw_compliance_slope = 32;
  int ccw_compliance_slope = 32;
  for(int i=1; i<=6;i++)//ax18
  {
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl::cw_compliance_margin,cw_compliance_margin));
    _controller.recv(READ_DURATION, _status);
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl::ccw_compliance_margin,ccw_compliance_margin));
    _controller.recv(READ_DURATION, _status);
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl:: cw_compliance_slope,cw_compliance_slope));
    _controller.recv(READ_DURATION, _status);
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl:: ccw_compliance_slope,ccw_compliance_slope));
    _controller.recv(READ_DURATION, _status);


  }

  int P1=50;
  int I1=30;//254;
  int D1=0;

  int P2=P1;
  int I2=I1;
  int D2=D1;

  for(int i=11; i<=16;i++)
  {
    //D
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl::cw_compliance_margin,D1));
    _controller.recv(READ_DURATION, _status);
    //I
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl::ccw_compliance_margin,I1));
    _controller.recv(READ_DURATION, _status);
    //P
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl:: cw_compliance_slope,P1));
    _controller.recv(READ_DURATION, _status);



  }
  for(int i=21; i<=26;i++)
  {
    //D
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl::cw_compliance_margin,D2));
    _controller.recv(READ_DURATION, _status);

    //I
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl::ccw_compliance_margin,I2));
    _controller.recv(READ_DURATION, _status);

    //P
    _controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl:: cw_compliance_slope,P2));
    _controller.recv(READ_DURATION, _status);

  }
}



void RobotHexa::applyCorrection(std::vector<int>& pos)
{
  assert(pos.size()==_correction.size());
  for(int i=0;i<pos.size();i++)
    pos[i]+= _correction[i];
}

void RobotHexa :: reset()
{
  try
  {
    if(_controller.isOpen()==false)
  	{
  	  ROS_INFO_STREAM("re-opening dynamixel's serial");
  	  _controller.open_serial("/dev/ttyUSB0",B1000000);
  	}
    _controller.flush();
  }
  catch (dynamixel::Error e)
  {
    ROS_ERROR_STREAM("(dynamixel): " << e.msg());
  }
#ifdef IMU
  try
  {

    if(_imu.isOpen()==false)
    {
      ROS_INFO_STREAM("re-opening imu's serial");
      _imu.open_serial("/dev/ttyUSB1");
    }
    _imu.flush();
  }
  catch (imu::Error e)
  {
    ROS_ERROR_STREAM("(imu): " << e.msg());
  }
#endif
  ROS_INFO_STREAM("setting all dynamixel to zero");
  //  setPID();

  enable();

  std::vector<int> pos(_actuators_ids.size());

  // FIXME : what is the point of that code ?
  // for (size_t i = 0; i < _actuators_ids.size(); ++i)
  //   if(_actuators_ids[i]>=20)
  //     pos[i]= 2048;
  //   else if (_actuators_ids[i] >= 10) // mx28
  //     pos[i] = 1024;
  //   else
  //     pos[i] = 2048;
  //
  // applyCorrection(pos);
  // _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
  // _controller.recv(READ_DURATION, _status);
  //
  // pos.clear();
  // pos.resize(_actuators_ids.size());
  // usleep(0.5e6);
  // for (size_t i = 0; i < _actuators_ids.size(); ++i)
  //   if(_actuators_ids[i]>=20)
  //     pos[i]= 2048-512-256;
  //   else if (_actuators_ids[i] >= 10) // mx28
  //     pos[i] = 1024;
  //   else
  //     pos[i] = 2048;
  //
  // applyCorrection(pos);
  // _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
  // _controller.recv(READ_DURATION, _status);
  //
  // pos.clear();
  // pos.resize(_actuators_ids.size());
  // usleep(0.5e6);
  // for (size_t i = 0; i < _actuators_ids.size(); ++i)
  //   if(_actuators_ids[i]>=20)
  //     pos[i]= 2048-256;
  //   else if (_actuators_ids[i] >= 10) // mx28
  //     pos[i] = 2048;
  //   else
  //     pos[i] = 2048;
  //
  // applyCorrection(pos);
  // _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
  // _controller.recv(READ_DURATION, _status);
//
//
//
//
  // pos.clear();
  // pos.resize(_actuators_ids.size());
  // usleep(0.5e6);
  for (size_t i = 0; i < _actuators_ids.size(); ++i)
  {
	  pos[i] = 2048;
  }

  applyCorrection(pos);
  _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
  _controller.recv(READ_DURATION, _status);

  sleep(1);

  ROS_INFO_STREAM("... done");

}


void RobotHexa:: position_zero()
{
  ROS_DEBUG_STREAM("initial position");
  enable();

  std::vector<int> pos(_actuators_ids.size());

  /*  for (size_t i = 0; i < _actuators_ids.size(); ++i)
      if(_actuators_ids[i]>=20)
      pos[i]= 2048;
      else if (_actuators_ids[i] >= 10) // mx28
      pos[i] = 1024;

      else
      pos[i] = 512;
      applyCorrection(pos);
      _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
      _controller.recv(READ_DURATION, _status);
  */



  pos.clear();
  pos.resize(_actuators_ids.size());
  sleep(1);
  for (size_t i = 0; i < _actuators_ids.size(); ++i)
  {
	  pos[i] = 2048;
  }

  try
  {
    applyCorrection(pos);
    _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
    _controller.recv(READ_DURATION, _status);
  }
  catch(dynamixel::Error e)
  {
    ROS_ERROR_STREAM(e.msg());
    sleep(10);
  }

  usleep(0.5e6);


  ROS_DEBUG_STREAM("done");


}




void RobotHexa :: _read_contacts()
{

  _controller.send(dynamixel::ax12::ReadData(11,dynamixel::ax12::ctrl::present_load_lo,2));
  if(  _controller.recv(READ_DURATION, _status))
  {
    if ((int)_status.decode16()>1024)
	  {
      _behavior_contact_0.push_back(-(int)_status.decode16()+1024);
	  }
    else
  	{

  	  _behavior_contact_0.push_back((int)_status.decode16());
  	}
  }
  else
    _behavior_contact_0.push_back(1024);//pas de reponse>>pas de contact



  _controller.send(dynamixel::ax12::ReadData(12,dynamixel::ax12::ctrl::present_load_lo,2));
  if(  _controller.recv(READ_DURATION, _status))
  {
    if ((int)_status.decode16()>1024)
	   _behavior_contact_1.push_back(-(int)_status.decode16()+1024);
    else
	   _behavior_contact_1.push_back((int)_status.decode16());
  }
  else
    _behavior_contact_1.push_back(1024);//pas de reponse>>pas de contact


  _controller.send(dynamixel::ax12::ReadData(13,dynamixel::ax12::ctrl::present_load_lo,2));
  if(  _controller.recv(READ_DURATION, _status))
  {
    if ((int)_status.decode16()>1024)
	   _behavior_contact_2.push_back(-(int)_status.decode16()+1024);
    else
	   _behavior_contact_2.push_back((int)_status.decode16());
  }
  else
    _behavior_contact_2.push_back(1024);//pas de reponse>>pas de contact


  _controller.send(dynamixel::ax12::ReadData(14,dynamixel::ax12::ctrl::present_load_lo,2));
  if(  _controller.recv(READ_DURATION, _status))
  {
    if ((int)_status.decode16()>1024)
	   _behavior_contact_3.push_back(-(int)_status.decode16()+1024);
    else
	   _behavior_contact_3.push_back((int)_status.decode16());
  }
  else
    _behavior_contact_3.push_back(1024);//pas de reponse>>pas de contact


  _controller.send(dynamixel::ax12::ReadData(15,dynamixel::ax12::ctrl::present_load_lo,2));
  if(  _controller.recv(READ_DURATION, _status))
  {
    if ((int)_status.decode16()>1024)
	   _behavior_contact_4.push_back(-(int)_status.decode16()+1024);
    else
	   _behavior_contact_4.push_back((int)_status.decode16());
  }
  else
    _behavior_contact_4.push_back(1024);//pas de reponse>>pas de contact


  _controller.send(dynamixel::ax12::ReadData(16,dynamixel::ax12::ctrl::present_load_lo,2));
  if(  _controller.recv(READ_DURATION, _status))
  {
    if ((int)_status.decode16()>1024)
	   _behavior_contact_5.push_back(-(int)_status.decode16()+1024);
    else
	   _behavior_contact_5.push_back((int)_status.decode16());
  }
  else
    _behavior_contact_5.push_back(1024);//pas de reponse>>pas de contact

}


void RobotHexa::contactSmoothing(int length)
{
  std::vector<float> smooth(_behavior_contact_0.size());

  for (int i=0;i<_behavior_contact_0.size();i++)
  {
    int k=0;
    for (int j=-length;j<=length;j++)
    {
      if (i+j>=0 && i+j<_behavior_contact_0.size())
      {
        smooth[i]+=_behavior_contact_0[i+j];
        k++;
      }
    }
    smooth[i]/=(float)k;

  }
  _behavior_contact_0.clear();




  for (int i=0;i<smooth.size();i++)
  {
    if (smooth[i]>0)
      _behavior_contact_0.push_back(0);
    else
      _behavior_contact_0.push_back(1);
  }

  smooth.clear();
  smooth.resize(_behavior_contact_1.size());
  for (int i=0;i<_behavior_contact_1.size();i++)
  {
    int k=0;
    for (int j=-length;j<=length;j++)
    {
	    if (i+j>=0 && i+j<_behavior_contact_1.size())
      {
	      smooth[i]+=_behavior_contact_1[i+j];
	      k++;
      }
    }
    smooth[i]/=(float)k;
  }
  _behavior_contact_1.clear();
  for (int i=0;i<smooth.size();i++)
  {
    if (smooth[i]>0)
	   _behavior_contact_1.push_back(0);
    else
	   _behavior_contact_1.push_back(1);
  }

  smooth.clear();
  smooth.resize(_behavior_contact_2.size());
  for (int i=0;i<_behavior_contact_2.size();i++)
  {
    int k=0;
    for (int j=-length;j<=length;j++)
    {
      if (i+j>=0 && i+j<_behavior_contact_2.size())
      {
        smooth[i]+=_behavior_contact_2[i+j];
        k++;
      }
    }
      smooth[i]/=(float)k;
    }
  _behavior_contact_2.clear();
  for (int i=0;i<smooth.size();i++)
  {
    if (smooth[i]>0)
	   _behavior_contact_2.push_back(0);
    else
	   _behavior_contact_2.push_back(1);
  }

  smooth.clear();
  smooth.resize(_behavior_contact_3.size());
  for (int i=0;i<_behavior_contact_3.size();i++)
  {
    int k=0;
    for (int j=-length;j<=length;j++)
    {
      if (i+j>=0 && i+j<_behavior_contact_3.size())
      {
	      smooth[i]+=_behavior_contact_3[i+j];
	      k++;
      }
    }
    smooth[i]/=(float)k;
  }
  _behavior_contact_3.clear();
  for (int i=0;i<smooth.size();i++)
  {
    if (smooth[i]>0)
	   _behavior_contact_3.push_back(0);
    else
	   _behavior_contact_3.push_back(1);
  }

  smooth.clear();
  smooth.resize(_behavior_contact_4.size());
  for (int i=0;i<_behavior_contact_4.size();i++)
  {
    int k=0;
    for (int j=-length;j<=length;j++)
    {
      if (i+j>=0 && i+j<_behavior_contact_4.size())
      {
        smooth[i]+=_behavior_contact_4[i+j];
        k++;
      }
    }
    smooth[i]/=(float)k;
  }
  _behavior_contact_4.clear();
  for (int i=0;i<smooth.size();i++)
  {
    if (smooth[i]>0)
	   _behavior_contact_4.push_back(0);
    else
	   _behavior_contact_4.push_back(1);
  }

  smooth.clear();
  smooth.resize(_behavior_contact_5.size());
  for (int i=0;i<_behavior_contact_5.size();i++)
  {
    int k=0;
    for (int j=-length;j<=length;j++)
    {
	    if (i+j>=0 && i+j<_behavior_contact_5.size())
      {
	      smooth[i]+=_behavior_contact_5[i+j];
	      k++;
      }
    }
    smooth[i]/=(float)k;
  }
  _behavior_contact_5.clear();
  for (int i=0;i<smooth.size();i++)
  {
    if (smooth[i]>0)
	   _behavior_contact_5.push_back(0);
    else
	   _behavior_contact_5.push_back(1);
  }
}

void RobotHexa ::write_contact(std::string const name)
{


  std::ofstream workingFile(name.c_str());

  if (workingFile)
  {
    for (int i =0;i<_behavior_contact_0.size();i++)
    {
      workingFile<<_behavior_contact_0[i]<<" "<<_behavior_contact_1[i]<<" "<<_behavior_contact_2[i]<<" "<<_behavior_contact_3[i]<<" "<<_behavior_contact_4[i]<<" "<<_behavior_contact_5[i]<<std::endl;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Impossible to open the file.");
  }


}



void RobotHexa :: initRosNode(  int argc ,char** argv,boost::shared_ptr<ros::NodeHandle> node_p)
{
  //  ros::init(argc, argv, "AlgoTransf",ros::init_options::NoSigintHandler);
  _node_p = node_p;
  //    _chatter_pub  = _node_p->advertise<rgbdslam::SferesCmd>("sferes_cmd", 1);

  // create publisher to reset UKF filter (robot_localization)
  _reset_filter_pub = _node_p->advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 1000);
}
/*void RobotHexa ::send_ros_start(int nbrun,int nbtrans)
  {
  rgbdslam::SferesCmd msg;
  msg.msg = "start";
  msg.run_number=nbrun;
  msg.transfer_number=nbtrans;


  _chatter_pub.publish(msg);
  }
  void RobotHexa ::send_ros_stop(int nbrun,int nbtrans)
  {
  rgbdslam::SferesCmd msg;
  msg.msg = "stop";
  msg.run_number=nbrun;
  msg.transfer_number=nbtrans;

  _chatter_pub.publish(msg);
  }*/


void RobotHexa:: getSlamInfo()
{
  this->_request_time=ros::Time::now();
  msg_recv=false;
  while(msg_recv ==false)
    ros::spinOnce();

}


void RobotHexa :: transfer(ControllerDuty& controller, float duration,int transfer_number)
{
  ROS_DEBUG_STREAM("Entering transfer.");

  _behavior_contact_0.clear();
  _behavior_contact_1.clear();
  _behavior_contact_2.clear();
  _behavior_contact_3.clear();
  _behavior_contact_4.clear();
  _behavior_contact_5.clear();
  std::vector<int>pos=controller.get_pos_dyna(0,_correction);

  try {
    _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
    _controller.recv(READ_DURATION, _status);

    _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, pos));
    _controller.recv(READ_DURATION, _status);
  }
  catch (dynamixel::Error e) {
    ROS_ERROR_STREAM("WARNING: error (dynamixel) -- init transfer" << e.msg() << " " << __FILE__<<" "<<__LINE__);
  }


  //std::cout<<"Waiting key " <<std::endl;
  //sleep();
  //std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
  // Init timers //////////////////////////
  struct timeval timev_init;  // Initial absolute time (static)
  struct timeval timev_prev;  // Previous tick absolute time
  struct timeval timev_cur;   // Current absolute time
  struct timeval timev_diff;  // Current tick position (curent - previous)
  struct timeval timev_duration;  // Duration of the movement (current - initial)
  unsigned int sampling_interval_us = 30000;//30000*4;
  float t=0;
  // Ticks loop ///////////////////////////
  bool first=true;

  // reset odometry
  ros::ServiceClient client = _node_p->serviceClient<std_srvs::Empty>("/reset_odom");
  std_srvs::Empty srv;
  if (client.call(srv)) {
    ROS_INFO_STREAM("reset_odom sent");
  } else {
    ROS_INFO_STREAM("Failed to reset odometry");
  }

  // reset UKF filter (robot_localization)
  // by publishing a PoseWithCovarianceStamped message
  geometry_msgs::PoseWithCovarianceStamped pose_with_cov_st;

  // set message's header
  pose_with_cov_st.header.stamp = ros::Time::now();
  pose_with_cov_st.header.frame_id = "/odom";

  // set position
  pose_with_cov_st.pose.pose.position.x = 0;
  pose_with_cov_st.pose.pose.position.y = 0;
  pose_with_cov_st.pose.pose.position.z = 0;

  // set orientation
  pose_with_cov_st.pose.pose.orientation.x = 0;
  pose_with_cov_st.pose.pose.orientation.y = 0;
  pose_with_cov_st.pose.pose.orientation.z = 0;
  pose_with_cov_st.pose.pose.orientation.w = 1;

  // publish message to reset UKF filter
  _reset_filter_pub.publish(pose_with_cov_st);
  ROS_INFO_STREAM("Message to reset UKF filter sent");

  _sub=_node_p->subscribe("vo",1,&RobotHexa::posCallback,this);
  ROS_INFO_STREAM("------------------------------------- First getSlamInfo() -----------------");
  usleep(1e6);
  getSlamInfo(); // TODO : commented in the attempt to fix the experiment
  //    send_ros_start(1,transfer_number);

  ROS_INFO_STREAM("------------------------------------- First getSlamInfo() OK -----------------");

  usleep(0.5e6);
  timerclear(&timev_init);
  gettimeofday(&timev_init, NULL);

  timev_prev = timev_init;


  int index=0;
  while (true)
  {
    gettimeofday(&timev_cur, NULL);
    timersub(&timev_cur, &timev_prev, &timev_diff);

    timersub(&timev_cur, &timev_init, &timev_duration);

    if (timev_duration.tv_sec >= duration)//*/index>=duration*1000000/sampling_interval_us)
    {
      ROS_INFO_STREAM("time duration "<<timev_duration.tv_sec<<"."<<timev_duration.tv_usec);


  	  usleep(0.5e6);
  	  //send_ros_stop(1,transfer_number);
      ROS_INFO_STREAM(__FILE__<<"  "<<__LINE__);


      ROS_INFO_STREAM("------------------------------------- SECOND getSlamInfo() -----------------");
      usleep(1e6);
      ROS_INFO_STREAM("sleep done");

  	  getSlamInfo();
      ROS_INFO_STREAM("------------------------------------- SECOND getSlamInfo() ok -----------------");

      ROS_INFO_STREAM(__FILE__<<"  "<<__LINE__);

  	  _sub.shutdown();
      ROS_INFO_STREAM(__FILE__<<"  "<<__LINE__);

  	//  contactSmoothing(2);
      ROS_INFO_STREAM(__FILE__<<"  "<<__LINE__);

  	  break;
    }
    if (first)
    {
      first=false;
      try {
        _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, controller.get_pos_dyna(timev_duration.tv_sec+timev_duration.tv_usec/(float)1000000,_correction)));
        _controller.recv(READ_DURATION, _status);
      }
      catch (dynamixel::Error e) {
        ROS_ERROR_STREAM("WARNING: error (dynamixel) -- init " << e.msg() << " " << __FILE__<<" "<<__LINE__);
      }
  //    _read_contacts();


#ifdef IMU
      std::vector<float> vect;

      _imu.recv(READ_DURATION,vect);

      _imu_angles.push_back(vect);
#endif
  	  index++;
  	  //_controller.send(dynamixel::ax12::SetSpeeds(_wheels_ids, controller.get_directions_dyna(),controller.get_speeds_dyna()));
  	  //_controller.recv(READ_DURATION, _status);


    }



    // On fait un step de sampling_interval_us (on suppose qu'une step ne dépasse pas 1 sec)
    if (timev_diff.tv_usec >= sampling_interval_us)
    {
      //std::cout<<timev_diff.tv_usec<<std::endl;
      try {
        _controller.send(dynamixel::ax12::SetPositions(_actuators_ids, controller.get_pos_dyna(timev_duration.tv_sec+timev_duration.tv_usec/(float)1000000,_correction)));
        _controller.recv(READ_DURATION, _status);
      }
      catch (dynamixel::Error e) {
        ROS_ERROR_STREAM("WARNING: error (dynamixel) -- step: " << e.msg());
      }

	//    _read_contacts();
#ifdef IMU
	    std::vector<float> vect;

  	  _imu.recv(READ_DURATION,vect);
  	  _imu_angles.push_back(vect);
#endif
  	  timev_prev = timev_cur;
  	  t+=0.030;//0.30 * 4;//0.030;
  	  index++;
    }

  }

}
