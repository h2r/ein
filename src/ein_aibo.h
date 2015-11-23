#ifndef _EIN_AIBO_H_
#define _EIN_AIBO_H_

#include <ros/ros.h>

#include<sys/socket.h>
#include<arpa/inet.h> 
#include <sys/poll.h>

class EinAiboJoints {

};

class EinAiboConfig {

  public:
  ros::Publisher aibo_snout_pub;

  Mat snoutImage;

  EinAiboJoints targetJoints;
  EinAiboJoints trueJoints;

  // sockets for AIBO
  int aibo_socket_desc;
  struct sockaddr_in aibo_server;
  const static int aibo_sock_buf_size = 1024*1024*3;
  char aibo_sock_buf[aibo_sock_buf_size];
  int aibo_sock_buf_valid_bytes = 0;
};





#endif /* _EIN_AIBO_H_*/
