#include "ein_words.h"
#include "ein.h"

#include "qtgui/einwindow.h"

#include "ein_aibo.h"

/*

Using MS_URBI pink memory stick image downloaded from 

https://github.com/Diamg/AiboRosPackages/tree/master/MS_URBI

cloned from

https://github.com/Diamg/AiboRosPackages.git

Looked briefly for the source since we can cross compile for Aibo.
The image is an URBI server for the dog. 

Wrapped some URBI in Back. 

XXX be careful, some of these actions can cause Aibo to hit the ground
harder than I would allow...

///////////// Start Original forward to motion.u ///////////////////////

Contributors: * Laboratory of Robotics in Versailles (original walk)
                  (if you have other walks, contact us)
              * Matthieu Nottale (Fourier extraction)
              * Assif Mirza (incremental turn)
              * Frederic Lettelier (adaptation of the inc. turn for walk)

Functions:
  robot.walk(duration) : walk for duration milliseconds. 
                         => Walk backward if duration is negative
  robot.stopwalk()     : interrupt the walk
  robot.turn(duration) : turn couterclockwise for duration milliseconds.
                         => Turn clockwise if duration is negative
  robot.stopturn()     : interrupt the turn

  robot.initial()   : initial position sitting down (strech and lay)
  robot.stretch()   : stretching like in the morning...
  robot.lay()       : laying (sitting down)
  robot.sit()       : sit on the back
  robot.beg()       : stand up with knees bent
  robot.stand()     : stand up

Variables:
  robot.walkspeed      : speed of walk, the smaller the faster: 
                         period of a step. Defaults to 1s
  robot.turnspeed      : speed of turn, the smaller the faster:
                         period of a step. Defaults to 1s

///////////// End   Original forward to motion.u ///////////////////////

///////////// Start Original forward to swtrans.u ///////////////////////
##############################################################
#                  AiboUrbi Postures                         #
#                                                            #
#                                                            #
#  Postures of the robot are set with                        #
#                                        robot.StandUp()     #
#                                        robot.SitDown()     #
#                                        robot.LayDown()     #
#                                                            #
##############################################################
///////////// End   Original forward to swtrans.u ///////////////////////

We open a socket for Ein to talk to the URBI server over WiFi.  WiFi must be WirelessB. 

You can send URBI directly or use the Back wrappers. 

////////////////////
///// To use WiFi you need to set up a MS_URBI_TEST/OPEN-R/SYSTEM/CONF/WLANCONF.TXT
///////
#
# WLAN
#
HOSTNAME=peregrin
ESSID=tinesWorld
WEPENABLE=1
WEPKEY=0xAD4278E4A4
# 0=ad-hoc ; 1=infrastructure ; 2=inf || ad
APMODE=1
CHANNEL=8

#
# IP network
#
USE_DHCP=1
#ETHER_IP=192.168.0.2
#ETHER_NETMASK=255.255.255.0
#IP_GATEWAY=192.168.0.1
#DNS_SERVER_1=192.168.0.1
#DNS_DEFDNAME=example.net

#
# SSDP - for UPnP
#
SSDP_ENABLE=1
///////

*/

void sendOnDogSocket(std::shared_ptr<MachineState> ms, int member, string message) {
  // append return
  message = message;

  if( send(ms->pack[member].aibo_socket_desc, message.c_str(), message.size(), 0) < 0) {
    cout << "send failed..." << endl;
    return;
  } else {
    cout << "sent: " << endl << message << endl;
  }
}

void flushDogBuffer(std::shared_ptr<MachineState> ms, int member) {
  int p_flush_wait_milliseconds = 250;

  int read_size = 1;
  while (read_size > 0) {
    // return if not ready to read.
    struct pollfd fd;
    int ret;

    fd.fd = ms->pack[member].aibo_socket_desc;  
    fd.events = POLLIN;
    ret = poll(&fd, 1, p_flush_wait_milliseconds);  
    switch (ret) {
      case -1:
	// Error
	cout << "file descriptor error during poll in flush..." << endl;
	read_size = -1;
	break;
      case 0:
	// Timeout 
	cout << "timeout during poll in flush..." << endl;
	read_size = 0;
	break;
      default:
	// read from dog
	read_size = read(ms->pack[member].aibo_socket_desc, ms->pack[member].aibo_sock_buf, ms->pack[member].aibo_sock_buf_size);
	if (read_size <= 0) {
	  cout << "oops, read failed, closing socket..." << endl;
	  close(ms->pack[member].aibo_socket_desc);
	  return;
	} else {
	  cout << "read " << read_size << " bytes during poll in flush..." << endl;
	  break;
	}
    }
  }
}

int getBytesFromDog(std::shared_ptr<MachineState> ms, int member, int bytesToGet, int timeout) {
  int read_size = 1;
  ms->pack[member].aibo_sock_buf_valid_bytes = 0;
  int sbStart = ms->pack[member].aibo_sock_buf_valid_bytes;

  while ( (read_size > 0) && (sbStart < bytesToGet) ){
    // return if not ready to read.
    struct pollfd fd;
    int ret;

    fd.fd = ms->pack[member].aibo_socket_desc;  
    fd.events = POLLIN;
    ret = poll(&fd, 1, timeout);  
    switch (ret) {
      case -1:
	// Error
	cout << "file descriptor error during poll in gbfd..." << endl;
	read_size = -1;
	break;
      case 0:
	// Timeout 
	cout << "timeout during poll in gbfd..." << endl;
	read_size = 0;
	break;
      default:
	// read from dog
	sbStart = ms->pack[member].aibo_sock_buf_valid_bytes;
	read_size = read(ms->pack[member].aibo_socket_desc, &(ms->pack[member].aibo_sock_buf[sbStart]), ms->pack[member].aibo_sock_buf_size-sbStart);
	if (read_size <= 0) {
	  cout << "oops, read failed, closing socket..." << endl;
	  close(ms->pack[member].aibo_socket_desc);
	  return ms->pack[member].aibo_sock_buf_valid_bytes;
	} else {
	  ms->pack[member].aibo_sock_buf_valid_bytes += read_size;
	}
	cout << "read " << read_size << " bytes during poll in gbfd..." << endl;
	break;
    }
  }
  return ms->pack[member].aibo_sock_buf_valid_bytes;
}

// XXX REQUIRE_FOCUSED_DOG macro

namespace ein_words {

WORD(DogFormPack)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("2 dogSetPackSize \
		       0 dogSetFocusedMember \"192.168.1.127\" 54000 socketOpen \
		       1 dogSetFocusedMember \"192.168.1.128\" 54000 socketOpen");
}
END_WORD
REGISTER_WORD(DogFormPack)

WORD(DogSetPackSize)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int toSet = 0;
  GET_INT_ARG(ms, toSet);

  if (toSet >= 0) {
    cout << "dogSetPackSize setting pack size: " << toSet << endl;
    ms->pack.resize(toSet);
  } else {
    cout << "dogSetPackSize: invalid size..." << endl;
  }
}
END_WORD
REGISTER_WORD(DogSetPackSize)

WORD(DogDispersePack)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pack.resize(2);
  for (int m = 0; m < ms->pack.size(); m++) {
    stringstream ss;
    ss << m << " dogSetFocusedMember socketClose";
    ms->evaluateProgram(ss.str());
  }
}
END_WORD
REGISTER_WORD(DogDispersePack)

WORD(DogSetFocusedMember)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int toSet = 0;
  GET_INT_ARG(ms, toSet);
  if ((toSet >= 0) && (toSet < ms->pack.size())) {
    ms->focusedMember = toSet;
    cout << "dogSetFocusedMember focusedMember: " << ms->focusedMember << endl;
  } else {
    cout << "dogSetFocusedMember invalid selection: " << toSet << endl;
  }
}
END_WORD
REGISTER_WORD(DogSetFocusedMember)

WORD(DogFocusedMember)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord(make_shared<IntegerWord>(ms->focusedMember));
}
END_WORD
REGISTER_WORD(DogFocusedMember)

WORD(SocketOpen)
virtual void execute(std::shared_ptr<MachineState> ms) {
  signal(SIGCHLD,SIG_IGN);
  string t_ip;
  int t_port = 0;

  GET_INT_ARG(ms, t_port);
  GET_STRING_ARG(ms, t_ip);

  cout << "opening socket to IP, port: " << t_ip << " " << t_port << endl;

  // destroy old socket
  shutdown(ms->pack[ms->focusedMember].aibo_socket_desc, 2);
  // create socket
  ms->pack[ms->focusedMember].aibo_socket_desc = socket(AF_INET, SOCK_STREAM, 0);
  if (ms->pack[ms->focusedMember].aibo_socket_desc == -1) {
    cout <<("could not create socket");
    return;
  }

  cout << "created socket..." << endl;
       
  ms->pack[ms->focusedMember].aibo_server.sin_addr.s_addr = inet_addr(t_ip.c_str());
  ms->pack[ms->focusedMember].aibo_server.sin_family = AF_INET;
  ms->pack[ms->focusedMember].aibo_server.sin_port = htons(t_port);

  //Connect to remote aibo_server
  if (connect(ms->pack[ms->focusedMember].aibo_socket_desc , (struct sockaddr *)&ms->pack[ms->focusedMember].aibo_server , sizeof(ms->pack[ms->focusedMember].aibo_server)) < 0) {
    cout << "connect error" << endl;
    return;
  }
   
  cout << "connected!" << endl;
}
END_WORD
REGISTER_WORD(SocketOpen)

WORD(SocketClose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // destroy old socket
  shutdown(ms->pack[ms->focusedMember].aibo_socket_desc, 2);
}
END_WORD
REGISTER_WORD(SocketClose)

WORD(SocketSend)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // send to dog
  string message;
  GET_STRING_ARG(ms, message);

  sendOnDogSocket(ms, ms->focusedMember, message);
}
END_WORD
REGISTER_WORD(SocketSend)

WORD(SocketRead)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // return if not ready to read.
  // 1 millisecond timeout
  struct pollfd fd;
  int ret;

  fd.fd = ms->pack[ms->focusedMember].aibo_socket_desc;  
  fd.events = POLLIN;
  ret = poll(&fd, 1, 1);  
  switch (ret) {
    case -1:
      // Error
      cout << "file descriptor error during poll..." << endl;
      return;
    case 0:
      // Timeout 
      cout << "timeout during poll..." << endl;
      return;
    default:
      break;
  }

  // read from dog
  int read_size = read(ms->pack[ms->focusedMember].aibo_socket_desc, ms->pack[ms->focusedMember].aibo_sock_buf, ms->pack[ms->focusedMember].aibo_sock_buf_size);
  if (read_size <= 0) {
    cout << "oops, read failed, closing socket..." << endl;
    close(ms->pack[ms->focusedMember].aibo_socket_desc);
    return;
  }

  ms->pack[ms->focusedMember].aibo_sock_buf[read_size] = '\0';

  string message(ms->pack[ms->focusedMember].aibo_sock_buf);

  cout << "socketRead read " << read_size << " bytes and contained: " << endl << message << endl;
}
END_WORD
REGISTER_WORD(SocketRead)


WORD(DogWhistle)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"192.168.1.127\" 54000 socketOpen");
}
END_WORD
REGISTER_WORD(DogWhistle)

WORD(DogMotorsOn)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"motors on;\" socketSend");
}
END_WORD
REGISTER_WORD(DogMotorsOn)

//// start swtrans.u wrappers

WORD(DogLayDown)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.LayDown();\" socketSend");
}
END_WORD
REGISTER_WORD(DogLayDown)

WORD(DogStandUp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.StandUp();\" socketSend");
}
END_WORD
REGISTER_WORD(DogStandUp)

WORD(DogSitDown)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.SitDown();\" socketSend");
}
END_WORD
REGISTER_WORD(DogSitDown)

//// end swtrans.u wrappers


/*
  robot.walk(duration) : walk for duration milliseconds. 
                         => Walk backward if duration is negative
  robot.stopwalk()     : interrupt the walk
  robot.turn(duration) : turn couterclockwise for duration milliseconds.
                         => Turn clockwise if duration is negative
  robot.stopturn()     : interrupt the turn

  robot.initial()   : initial position sitting down (strech and lay)
  robot.stretch()   : stretching like in the morning...
  robot.lay()       : laying (sitting down)
  robot.sit()       : sit on the back
  robot.beg()       : stand up with knees bent
  robot.stand()     : stand up

Variables:
  robot.walkspeed      : speed of walk, the smaller the faster: 
                         period of a step. Defaults to 1s
  robot.turnspeed      : speed of turn, the smaller the faster:
                         period of a step. Defaults to 1s

*/

//// start motion.u wrappers

WORD(DogWalkSeconds)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // specify in seconds
  double duration = 0.0;
  GET_NUMERIC_ARG(ms, duration);
  
  stringstream ss;
  ss << "\"robot.walk(" << duration*1000 << ");\" socketSend";
  ms->evaluateProgram(ss.str());
}
END_WORD
REGISTER_WORD(DogWalkSeconds)

WORD(DogTurnSeconds)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // specify in seconds
  double duration = 0.0;
  GET_NUMERIC_ARG(ms, duration);
  
  stringstream ss;
  ss << "\"robot.turn(" << duration*1000 << ");\" socketSend";
  ms->evaluateProgram(ss.str());
}
END_WORD
REGISTER_WORD(DogTurnSeconds)

WORD(DogWalkMeters)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // specify in meters measured with robot.walkspeed = 1s
  // this is not meant to be very accurate
  double meters = 0.0;
  GET_NUMERIC_ARG(ms, meters);
  
  stringstream ss;
  ss << "\"robot.walk(" << meters*12*1000 << ");\" socketSend";
  ms->evaluateProgram(ss.str());
}
END_WORD
REGISTER_WORD(DogWalkMeters)

WORD(DogTurnRadians)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // specify in meters measured with robot.turnspeed = 1s
  double radians = 0.0;
  GET_NUMERIC_ARG(ms, radians);
  
  stringstream ss;
  // ten seconds turns it all the way around
  ss << "\"robot.turn(" << radians / (2 * 3.1415926) * 10 * 1000 << ");\" socketSend";
  ms->evaluateProgram(ss.str());
}
END_WORD
REGISTER_WORD(DogTurnRadians)

// this might not work...
WORD(DogStop)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.stopturn(), robot.stopwalk();\" socketSend");
}
END_WORD
REGISTER_WORD(DogStop)

WORD(DogInitial)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.initial();\" socketSend");
}
END_WORD
REGISTER_WORD(DogInitial)

WORD(DogStretch)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.stretch();\" socketSend");
}
END_WORD
REGISTER_WORD(DogStretch)

WORD(DogLay)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.lay();\" socketSend");
}
END_WORD
REGISTER_WORD(DogLay)

WORD(DogSit)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.sit();\" socketSend");
}
END_WORD
REGISTER_WORD(DogSit)

WORD(DogBeg)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.beg();\" socketSend");
}
END_WORD
REGISTER_WORD(DogBeg)

WORD(DogStand)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.stand();\" socketSend");
}
END_WORD
REGISTER_WORD(DogStand)

WORD(DogSetTurnSpeed)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // specify the period in seconds
  double seconds = 0.0;
  GET_NUMERIC_ARG(ms, seconds);
  
  stringstream ss;
  ss << "\"robot.turnspeed = " << seconds  << ";\" socketSend";
  ms->evaluateProgram(ss.str());
}
END_WORD
REGISTER_WORD(DogSetTurnSpeed)

WORD(DogSetWalkSpeed)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // specify the period in seconds
  double seconds = 0.0;
  GET_NUMERIC_ARG(ms, seconds);
  
  stringstream ss;
  ss << "\"robot.walkspeed = " << seconds  << ";\" socketSend";
  ms->evaluateProgram(ss.str());
}
END_WORD
REGISTER_WORD(DogSetWalkSpeed)

//// end motion.u wrappers

WORD(DogFormatImageDefault)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"camera.format = 0; camera.reconstruct = 0; camera.resolution = 0;\" socketSend 0.5 waitForSeconds");
}
END_WORD
REGISTER_WORD(DogFormatImageDefault)

WORD(DogGetImage)
virtual void execute(std::shared_ptr<MachineState> ms) {

  int this_dog = ms->focusedMember;

  // read until there is nothing
  flushDogBuffer(ms, this_dog); 

  // request the image
  string image_request("camera.val;");
  sendOnDogSocket(ms, this_dog, image_request);

  // get specs, verify format, ready image
  int rows = 160;
  int cols = 208;
  ms->pack[this_dog].snoutImage = Mat(rows, cols, CV_8UC3);

  int yCbCr208x160MessageLength = 99875;
  int chosenFormatLength = yCbCr208x160MessageLength;

  int dogGottenBytes = getBytesFromDog(ms, this_dog, chosenFormatLength, 250);
  if (dogGottenBytes == chosenFormatLength) {
    cout << "dogGetImage: got " << dogGottenBytes << endl;
    // start immediately before the good stuff
    int dataIdx = chosenFormatLength - (rows * cols * 3);
    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < cols; x++) {
	ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[0] = ms->pack[this_dog].aibo_sock_buf[dataIdx];
	//cout << int( ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[0] ) << " ";
	dataIdx++;
	ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[1] = ms->pack[this_dog].aibo_sock_buf[dataIdx];
	//cout << int( ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[1] ) << " ";
	dataIdx++;
	ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[2] = ms->pack[this_dog].aibo_sock_buf[dataIdx];
	//cout << int( ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[2] ) << " ";
	dataIdx++;
      }
    }

    cvtColor(ms->pack[this_dog].snoutImage, ms->pack[this_dog].snoutImage, CV_YCrCb2RGB);

    ms->config.dogSnoutViewWindow->updateImage(ms->pack[this_dog].snoutImage);
  } else {
    ROS_ERROR_STREAM("dogGetImage: Failed to get dog image... got " << dogGottenBytes << " bytes." << endl);
  }
}
END_WORD
REGISTER_WORD(DogGetImage)




/*

WORD(Dog)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.();\" socketSend");
}
END_WORD
REGISTER_WORD(Dog)


WORD(Dog)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\";\" socketSend");
}
END_WORD
REGISTER_WORD(Dog)

*/



}
