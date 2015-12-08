// Package-specific includes
#include <hexa_control/Transfert.h>

// ROS includes
#include <ros/ros.h>

// Standard includes
#include <cstdlib>
#include <fstream>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hexapod_client", ros::init_options::NoSigintHandler);
  if (argc!= 2 && argc !=3 && argc !=4)
  {
    ROS_INFO_STREAM("usage: hexapod_client FILE (DURATION)  (STARTING BEHAVIOR)");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<hexa_control::Transfert>("Transfert");
  hexa_control::Transfert srv;

  //  std::ifstream monFlux("/home/antoinecully/fuerte_workspace/sandbox/hexa_control/results.dat");  //Ouverture d'un fichier en lecture
  std::ifstream monFlux(argv[1]); // Open the transfer file for reading

  std::vector<std::vector<float> > controllers;
  if(monFlux)
  {
    // Old broken way to parse the file, since the format evolved
    // Yet, the example files should be removed beforhand.
    // while(!monFlux.eof())
    // {
    //   std::vector<float> controller;
    //   for(int i =0;i<43;i++)
    //   {
    //     if(monFlux.eof())
    //       break;
    //     float data;
    //     monFlux>>data;
    //     //	      ROS_INFO("%d",data);
    //     if(i>=7)
    //       controller.push_back(data);
    //     //  std::cout<<data<<" ";
    //   }
    //   if(controller.size()==36)
    //     controllers.push_back(controller);
    //   //	  std::cout<<std::endl;
    // }
    monFlux.ignore(100, '\n');
    bool even_line = true;
    while(!monFlux.eof())
    {
      if(even_line)
      {
        monFlux.ignore(100, '\n');
        even_line = !even_line;
        std::cout << "ignored line" << std::endl;
      }
      else
      {
        std::vector<float> controller;
        for(int i=0; i<36; i++)
        {
          if(monFlux.eof())
            break;
          float controller_parameter;
          monFlux >> controller_parameter;
          controller.push_back(controller_parameter);
        //   std::cout << controller_parameter << " ";
        }
        even_line = !even_line;
        if (controller.size() == 36)
            controllers.push_back(controller);
        monFlux.ignore(1, '\n');
        // std::cout << "\n";
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM("Impossible d'ouvrir le fichier en lecture.");
    return 0;
  }

  /*  for(int i=0; i<controllers.size();i++)
    {
      for(int j=0; j<controllers[i].size();j++)
	std::cout<<controllers[i][j]<<" ";
      std::cout<<std::endl;
    }
  */




  /*  int amp;
  if(argc=3)
    amp=atoll(argv[2]);
  else
    amp=1;

  srv.request.params[0]=amp;
  srv.request.params[1]=0;
  srv.request.params[2]=1;
  srv.request.params[3]=1;

  srv.request.params[4]=amp;
  srv.request.params[5]=2;
  srv.request.params[6]=1;
  srv.request.params[7]=3;

  srv.request.params[8]=amp;
  srv.request.params[9]=0;
  srv.request.params[10]=1;
  srv.request.params[11]=1;

  srv.request.params[12]=amp;
  srv.request.params[13]=0;
  srv.request.params[14]=1;
  srv.request.params[15]=3;

  srv.request.params[16]=amp;
  srv.request.params[17]=2;
  srv.request.params[18]=1;
  srv.request.params[19]=1;

  srv.request.params[20]=amp;
  srv.request.params[21]=0;
  srv.request.params[22]=1;
  srv.request.params[23]=3;

  for(int i=0;i<24;i++)
    srv.request.params[i]=    srv.request.params[i]/4;
  */
  int start=0;
  int duration=3;
  if(argc>3)
    start=atoll(argv[3]);

  if(argc>2)
    duration=atoll(argv[2]);

  if(start<0 || start>=controllers.size())
  {
    ROS_ERROR_STREAM("BAD PARAMETER VALUE ");
    return 0;
  }
  if(duration<=0)
  {
    srv.request.duration=duration;
    ROS_INFO_STREAM("relax...");
    if (client.call(srv))
    {
      ROS_INFO_STREAM("executed");
    }
    else
  	{
  	  ROS_ERROR_STREAM("Failed to call service");
  	  //return 1;
  	}
    return 0;
  }

  for(int num_controller=start;num_controller<controllers.size();num_controller++)
  {
    for(int i=0;i<36;i++)
  	{
  	  srv.request.params[i]=   controllers[num_controller][i];
  	  //	  std::cout<<srv.request.params[i]<< " ";
  	}

    srv.request.duration=duration;

    ROS_INFO_STREAM("execution of the "<<num_controller<<" th controller during "<< srv.request.duration<<" seconds");

    if (client.call(srv))
  	{
  	  ROS_INFO_STREAM("executed");
  	}
    else
  	{
  	  ROS_ERROR_STREAM("Failed to call service");
  	  //return 1;
  	}
    srv.request.duration=-1;
    if (client.call(srv))
  	{
  	  ROS_INFO_STREAM("executed");
  	}
    else
  	{
  	  ROS_ERROR_STREAM("Failed to call service");
  	  //return 1;
  	}
  	std::cin.clear();
  	std::cout<<"wait for key :"<<std::endl;
  	int ok;
  	std::cin>> ok;
  	std::cin.clear();
  	std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

  }

  srv.request.duration=-2;//relax
  ROS_INFO_STREAM("relax...");
  if (client.call(srv))
	{
	  ROS_INFO_STREAM("executed");
	}
  else
	{
	  ROS_ERROR_STREAM("Failed to call service");
	  //return 1;
	}
  return 0;
}
