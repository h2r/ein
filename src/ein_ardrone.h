#ifndef _EIN_ARDRONE_H_
#define _EIN_ARDRONE_H_

#include <ros/ros.h>

#include<sys/socket.h>
#include<arpa/inet.h>
#include <sys/poll.h>

#include "word.h"



class EinArDrone {
 public:
  ros::Subscriber truePoseSubscriber;

  ros::Publisher resetPublisher;
  ros::Publisher landPublisher;
  ros::Publisher takeoffPublisher;

  ros::Publisher posePublisher;

  ros::ServiceClient cameraSwitchService;


};




#endif /* _EIN_ARDRONE_H_*/
