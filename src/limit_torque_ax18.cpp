#include <iostream>
#include <dynamixel/dynamixel.hpp>

#define READ_DURATION 0.005f

int main(int argc, char **argv)
{
  int torque_limit=1023;
  if(argc==2)
    torque_limit=atoi(argv[1]);
  dynamixel::Usb2Dynamixel controller;
  controller.open_serial("/dev/ttyACM0",B1000000);
  dynamixel::Status status;
  controller.scan_ax12s();
  int torque_H=0;
  int torque_L=torque_limit;
  if(torque_limit>255)
    { 
      torque_H=torque_limit>>8;
      torque_L=torque_limit%(torque_H<<8);
    }  
  std::cout<<"setting max torques to "<<torque_limit<<"("<<torque_H<<" "<< torque_L<<")"<<std::endl;
  try{
    for(int i=1; i<=6;i++)//ax18
      {
	controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl::max_torque_lo,torque_L,torque_H));       
	controller.recv(READ_DURATION, status);

	controller.send(dynamixel::ax12::WriteData(i,dynamixel::ax12::ctrl::torque_limit_lo,torque_L,torque_H));       
	controller.recv(READ_DURATION, status);


       
      }
  }
  catch (dynamixel::Error e)
    {
      std::cerr << "error (dynamixel): " << e.msg() << std::endl;
    }  
  
}
