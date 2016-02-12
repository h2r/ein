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
ESSID=arachna
WEPENABLE=1
WEPKEY=0xDEADBEEF00
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

#define DOG_READ_VAR(x) \
  nextCoreMessage = findStringInDogBuffer(ms, this_dog, string("] "), nextCoreMessage); \
  r = stod(&(ms->pack[this_dog].aibo_sock_buf[nextCoreMessage]), &idx);  \
  x = r; \
  cout << "dgbi got: " << r << " for " << #x << endl; 

void dogReconnect(std::shared_ptr<MachineState> ms) {
  ms->pack[ms->focusedMember].aibo_socket_did_connect = 0;
  ms->pack[ms->focusedMember].dog_needs_reinit = 1;
  close(ms->pack[ms->focusedMember].aibo_socket_desc);
  ms->evaluateProgram("dogFocusedIP 54000 socketOpen ( endStackCollapseNoop dogSocketDidConnect not ) ( dogFocusedIP 54000 socketOpen ) while 15.0 waitForSeconds dogMotorsOn 1.0 waitForSeconds");
}

void sendOnDogSocket(std::shared_ptr<MachineState> ms, int member, string message) {
  int p_send_poll_timeout = 5000;
  struct pollfd fd = { ms->pack[member].aibo_socket_desc, POLLOUT, 0 };
  if (poll(&fd, 1, p_send_poll_timeout) != 1) {
    ROS_ERROR_STREAM("poll says unavailable for sending after " << p_send_poll_timeout << " ms" << endl);
    dogReconnect(ms);
    return;
  } else {
    if( send(ms->pack[member].aibo_socket_desc, message.c_str(), message.size(), 0) != message.size() ) {
      ROS_ERROR_STREAM("send failed..." << endl);
      dogReconnect(ms);
      return;
    } else {
      cout << "sent: " << endl << message << endl;
      //cout << "sent: " << message.size() << endl;
    }
  }
}

void sendOnDogSocket(std::shared_ptr<MachineState> ms, int member, char *buf, int size) {
  int p_send_poll_timeout = 5000;
  struct pollfd fd = { ms->pack[member].aibo_socket_desc, POLLOUT, 0 };
  if (poll(&fd, 1, p_send_poll_timeout) != 1) {
    ROS_ERROR_STREAM("poll says unavailable for sending after " << p_send_poll_timeout << " ms" << endl);
    dogReconnect(ms);
    return;
  } else {
    if( send(ms->pack[member].aibo_socket_desc, buf, size, 0) != size ) {
      ROS_ERROR_STREAM("send failed..." << endl);
      dogReconnect(ms);
      return;
    } else {
      cout << "sent: " << endl << size << " bytes." << endl;
      //cout << "sent: " << message.size() << endl;
    }
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
	  dogReconnect(ms);
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
	  dogReconnect(ms);
	  return ms->pack[member].aibo_sock_buf_valid_bytes;
	} else {
	  ms->pack[member].aibo_sock_buf_valid_bytes += read_size;
	}
	sbStart = ms->pack[member].aibo_sock_buf_valid_bytes;
	cout << "read " << read_size << " bytes during poll in gbfd..." << endl;
	break;
    }
  }
  return ms->pack[member].aibo_sock_buf_valid_bytes;
}

// returns the byte after the searched string
int readBytesFromDogUntilString(std::shared_ptr<MachineState> ms, int member, int timeout, string toFind) {
  int read_size = 1;
  ms->pack[member].aibo_sock_buf_valid_bytes = 0;
  int sbStart = ms->pack[member].aibo_sock_buf_valid_bytes;

  int searchedBytes = 0;

  while ( (read_size > 0) ){

    for (; searchedBytes < (ms->pack[member].aibo_sock_buf_valid_bytes - int(toFind.size()) + 1); searchedBytes++) {
      //cout << searchedBytes << " " << ms->pack[member].aibo_sock_buf[searchedBytes] << " " << ms->pack[member].aibo_sock_buf_valid_bytes << " " << toFind.size() << " " << (ms->pack[member].aibo_sock_buf_valid_bytes - int(toFind.size()) + 1) << endl;
      if (toFind.compare(0, toFind.size(), &(ms->pack[member].aibo_sock_buf[searchedBytes]), toFind.size()) == 0) {
	searchedBytes += toFind.size();
	cout << "readBytesFromDogUntilString found " << toFind << " at " << searchedBytes << endl;
	return searchedBytes;
      } else {
      }
    }

    // return if not ready to read.
    struct pollfd fd;
    int ret;

    fd.fd = ms->pack[member].aibo_socket_desc;  
    fd.events = POLLIN;
    ret = poll(&fd, 1, timeout);  
    switch (ret) {
      case -1:
	// Error
	cout << "file descriptor error during poll in rbfdus..." << endl;
	read_size = -1;
	break;
      case 0:
	// Timeout 
	cout << "timeout during poll in rbfdus..." << endl;
	read_size = 0;
	break;
      default:
	// read from dog
	sbStart = ms->pack[member].aibo_sock_buf_valid_bytes;
	read_size = read(ms->pack[member].aibo_socket_desc, &(ms->pack[member].aibo_sock_buf[sbStart]), ms->pack[member].aibo_sock_buf_size-sbStart);
	if (read_size <= 0) {
	  cout << "oops, read failed, closing socket..." << endl;
	  dogReconnect(ms);
	  return -1;
	} else {
	  ms->pack[member].aibo_sock_buf_valid_bytes += read_size;
	}
	cout << "read " << read_size << " bytes during poll in rbfdus..." << endl;
	break;
    }
  }
  return -1;
}

// returns the byte after the searched string
int findStringInDogBuffer(std::shared_ptr<MachineState> ms, int member, string toFind, int start) {
  int searchedBytes = 0;
  for (searchedBytes = start; searchedBytes < (ms->pack[member].aibo_sock_buf_valid_bytes - int(toFind.size()) + 1); searchedBytes++) {
    if (toFind.compare(0, toFind.size(), &(ms->pack[member].aibo_sock_buf[searchedBytes]), toFind.size()) == 0) {
      searchedBytes += toFind.size();
      cout << "findStringInDogBuffer found " << toFind << " at " << searchedBytes << endl;
      return searchedBytes;
    } else {
    }
  }

  cout << "findStringInDogBuffer failed to find " << toFind << " at " << searchedBytes << endl;
  string searchedText(ms->pack[member].aibo_sock_buf, searchedBytes);
  cout << "  searched: " << endl << searchedText << endl;

  return -1;
}



// XXX REQUIRE_FOCUSED_DOG macro

namespace ein_words {

WORD(DogSendPack)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string message;
  GET_STRING_ARG(ms, message);

  for (int m = 0; m < ms->pack.size(); m++) {
    sendOnDogSocket(ms, m, message);
  }
}
END_WORD
REGISTER_WORD(DogSendPack)

WORD(DogDoPack)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> w1;
  GET_WORD_ARG(ms, Word, w1);

  for (int m = 0; m < ms->pack.size(); m++) {
    ms->pushWord(w1);
    ms->pushWord("dogIncrementFocusedMember");
  }
}
END_WORD
REGISTER_WORD(DogDoPack)

WORD(DogStop)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  string message("stopall;");
  sendOnDogSocket(ms, this_dog, message);
}
END_WORD
REGISTER_WORD(DogStop)

WORD(DogIncrementFocusedMember)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->focusedMember = (ms->focusedMember + 1) % ms->pack.size();
}
END_WORD
REGISTER_WORD(DogIncrementFocusedMember)

WORD(DogDecrementPackMember)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->focusedMember = (ms->focusedMember - 1) % ms->pack.size();
}
END_WORD
REGISTER_WORD(DogDecrementPackMember)


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

WORD(DogBark)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("socketSend");
  ms->pushWord(make_shared<StringWord>("speaker.play(\"bark.wav\");"));
}
END_WORD
REGISTER_WORD(DogBark)

WORD(DogFocusedIP)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord(make_shared<StringWord>(ms->pack[ms->focusedMember].ip_string));
}
END_WORD
REGISTER_WORD(DogFocusedIP)

WORD(DogNeedsReinit)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord(make_shared<IntegerWord>(ms->pack[ms->focusedMember].dog_needs_reinit));
}
END_WORD
REGISTER_WORD(DogNeedsReinit)

WORD(DogSignalReinitDone)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pack[ms->focusedMember].dog_needs_reinit = 0;
}
END_WORD
REGISTER_WORD(DogSignalReinitDone)

WORD(SocketOpen)
virtual void execute(std::shared_ptr<MachineState> ms) {
  signal(SIGCHLD,SIG_IGN);
  string t_ip;
  int t_port = 0;

  GET_INT_ARG(ms, t_port);
  GET_STRING_ARG(ms, t_ip);

  cout << "opening socket to IP, port: " << t_ip << " " << t_port << endl;

  // destroy old socket
  close(ms->pack[ms->focusedMember].aibo_socket_desc);
  // create socket
  ms->pack[ms->focusedMember].aibo_socket_desc = socket(AF_INET, SOCK_STREAM, 0);
  if (ms->pack[ms->focusedMember].aibo_socket_desc == -1) {
    cout <<("could not create socket");
    return;
  }

  cout << "created socket..." << endl;
       
  ms->pack[ms->focusedMember].ip_string = t_ip;
  ms->pack[ms->focusedMember].aibo_server.sin_addr.s_addr = inet_addr(t_ip.c_str());
  ms->pack[ms->focusedMember].aibo_server.sin_family = AF_INET;
  ms->pack[ms->focusedMember].aibo_server.sin_port = htons(t_port);

  //Connect to remote aibo_server
  if (connect(ms->pack[ms->focusedMember].aibo_socket_desc , (struct sockaddr *)&ms->pack[ms->focusedMember].aibo_server , sizeof(ms->pack[ms->focusedMember].aibo_server)) < 0) {
    cout << "connect error" << endl;
    ms->pack[ms->focusedMember].aibo_socket_did_connect = 0;
    ms->pack[ms->focusedMember].dog_needs_reinit = 1;
    close(ms->pack[ms->focusedMember].aibo_socket_desc);
    return;
  } else {
    ms->pack[ms->focusedMember].aibo_socket_did_connect = 1;
  }
   
  cout << "connected!" << endl;
}
END_WORD
REGISTER_WORD(SocketOpen)


WORD(DogSocketDidConnect)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<IntegerWord> word = std::make_shared<IntegerWord>(ms->pack[ms->focusedMember].aibo_socket_did_connect);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(DogSocketDidConnect)

WORD(SocketClose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // destroy old socket
  close(ms->pack[ms->focusedMember].aibo_socket_desc);
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
    dogReconnect(ms);
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

WORD(DogMotorsOff)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"motors off;\" socketSend");
}
END_WORD
REGISTER_WORD(DogMotorsOff)

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
WORD(DogStopWalkTurn)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("\"robot.stopturn(), robot.stopwalk();\" socketSend");
}
END_WORD
REGISTER_WORD(DogStopWalkTurn)

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
  int this_dog = ms->focusedMember;
  ros::NodeHandle n("~");
  stringstream ss;
  ss << "dog_" << ms->focusedMember << "_snout"; 
  ms->pack[this_dog].aibo_snout_pub = n.advertise<sensor_msgs::Image>(ss.str(),10);

  ms->evaluateProgram("\"camera.format = 0; camera.reconstruct = 0; camera.resolution = 0;\" socketSend 0.5 waitForSeconds");
}
END_WORD
REGISTER_WORD(DogFormatImageDefault)

// XXX this should be redone to detect and handle format using new dog read methods
WORD(DogGetImage)
virtual void execute(std::shared_ptr<MachineState> ms) {

  int this_dog = ms->focusedMember;

  int p_get_image_timeout = 1000;

  // read until there is nothing
  flushDogBuffer(ms, this_dog); 

  // request the image
  string image_request("camera.val;");
  sendOnDogSocket(ms, this_dog, image_request);

  //usleep(400*1000);

  // get specs, verify format, ready image
  int rows = 160;
  int cols = 208;
  ms->pack[this_dog].snoutImage = Mat(rows, cols, CV_8UC3);

  int yCbCr208x160MessageLength = 99875;
  int chosenFormatLength = yCbCr208x160MessageLength;

  int dogGottenBytes = getBytesFromDog(ms, this_dog, chosenFormatLength, p_get_image_timeout);
  if (dogGottenBytes >= chosenFormatLength) {
    cout << "dogGetImage: got " << dogGottenBytes << endl;

    if (dogGottenBytes > chosenFormatLength) {
      cout << "dogGetImage: got " << dogGottenBytes << " bytes, so the message index is probably more than eight digits. This dog must be tired." << endl;
    } else {}

    // start immediately before the good stuff
    int dataIdx = dogGottenBytes - (rows * cols * 3);
    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < cols; x++) {
	// dog is in YCbCr and opencv uses YCrCb
	ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[0] = ms->pack[this_dog].aibo_sock_buf[dataIdx];
	//cout << int( ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[0] ) << " ";
	dataIdx++;
	ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[2] = ms->pack[this_dog].aibo_sock_buf[dataIdx];
	//cout << int( ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[1] ) << " ";
	dataIdx++;
	ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[1] = ms->pack[this_dog].aibo_sock_buf[dataIdx];
	//cout << int( ms->pack[this_dog].snoutImage.at<Vec3b>(y,x)[2] ) << " ";
	dataIdx++;
      }
    }

    // convert YCrCb explicity to BGR
    cvtColor(ms->pack[this_dog].snoutImage, ms->pack[this_dog].snoutImage, CV_YCrCb2BGR);

    // for now this is done for display purposes only
    Size p_toBecome(640, 400);
    cv::resize(ms->pack[this_dog].snoutImage, ms->pack[this_dog].snoutCamImage, p_toBecome);
    ms->config.dogSnoutViewWindow->updateImage(ms->pack[this_dog].snoutCamImage);
  } else {
    ROS_ERROR_STREAM("dogGetImage: Failed to get dog image... got " << dogGottenBytes << " bytes." << endl);
  }
}
END_WORD
REGISTER_WORD(DogGetImage)

WORD(DogPublishSnout)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;
  {
    sensor_msgs::Image msg;
    Mat topub = ms->pack[this_dog].snoutImage;

    msg.header.stamp = ros::Time::now();
    msg.width = topub.cols;
    msg.height = topub.rows;
    msg.step = topub.cols * topub.elemSize();
    msg.is_bigendian = false;
    msg.encoding = sensor_msgs::image_encodings::RGB8;
    msg.data.assign(topub.data, topub.data + size_t(topub.rows * msg.step));

    ms->pack[this_dog].aibo_snout_pub.publish(msg);
  }
}
END_WORD
REGISTER_WORD(DogPublishSnout)

WORD(DogGetSensoryMotorStates)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  // read until there is nothing
  flushDogBuffer(ms, this_dog); 

  // request the info 
  string request("dgsms +begin: legLF1.val; legLF2.val; legLF3.val; legLH1.val; legLH2.val; legLH3.val; legRH1.val; legRH2.val; legRH3.val; legRF1.val; legRF2.val; legRF3.val; neck.val; headPan.val; headTilt.val; tailPan.val; tailTilt.val; mouth.val; legLF1.PGain; legLF2.PGain; legLF3.PGain; legLH1.PGain; legLH2.PGain; legLH3.PGain; legRH1.PGain; legRH2.PGain; legRH3.PGain; legRF1.PGain; legRF2.PGain; legRF3.PGain; neck.PGain; headPan.PGain; headTilt.PGain; tailPan.PGain; tailTilt.PGain; mouth.PGain; legLF1.IGain; legLF2.IGain; legLF3.IGain; legLH1.IGain; legLH2.IGain; legLH3.IGain; legRH1.IGain; legRH2.IGain; legRH3.IGain; legRF1.IGain; legRF2.IGain; legRF3.IGain; neck.IGain; headPan.IGain; headTilt.IGain; tailPan.IGain; tailTilt.IGain; mouth.IGain; legLF1.DGain; legLF2.DGain; legLF3.DGain; legLH1.DGain; legLH2.DGain; legLH3.DGain; legRH1.DGain; legRH2.DGain; legRH3.DGain; legRF1.DGain; legRF2.DGain; legRF3.DGain; neck.DGain; headPan.DGain; headTilt.DGain; tailPan.DGain; tailTilt.DGain; mouth.DGain; distanceNear.val; distance.val; distanceChest.val; accelX.val; accelY.val; accelZ.val; pawLF.val; pawLH.val; pawRF.val; pawRH.val; chinSensor.val; headSensor.val; backSensorR.val; backSensorM.val; dgsms +end: backSensorF.val;");

  sendOnDogSocket(ms, this_dog, request);

  // read until all info is in
  readBytesFromDogUntilString(ms, this_dog, 5000, string(":dgsms] *** end\n"));

  int startCoreMessage = findStringInDogBuffer(ms, this_dog, string(":dgsms] *** begin\n"), 0);
  cout << ms->pack[this_dog].aibo_sock_buf[startCoreMessage] << endl;

  size_t idx;
  int nextCoreMessage = startCoreMessage; 
  double r = 0.0;

  DOG_READ_VAR(ms->pack[this_dog].truePose.legLF1);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legLF2);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legLF3);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legLH1);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legLH2);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legLH3);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legRH1);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legRH2);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legRH3);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legRF1);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legRF2);
  DOG_READ_VAR(ms->pack[this_dog].truePose.legRF3);
  DOG_READ_VAR(ms->pack[this_dog].truePose.neck);
  DOG_READ_VAR(ms->pack[this_dog].truePose.headPan);
  DOG_READ_VAR(ms->pack[this_dog].truePose.headTilt);
  DOG_READ_VAR(ms->pack[this_dog].truePose.tailPan);
  DOG_READ_VAR(ms->pack[this_dog].truePose.tailTilt);
  DOG_READ_VAR(ms->pack[this_dog].truePose.mouth);

  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legLF1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legLF2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legLF3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legLH1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legLH2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legLH3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legRH1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legRH2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legRH3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legRF1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legRF2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].legRF3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].neck);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].headPan);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].headTilt);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].tailPan);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].tailTilt);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[0].mouth);

  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legLF1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legLF2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legLF3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legLH1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legLH2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legLH3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legRH1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legRH2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legRH3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legRF1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legRF2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].legRF3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].neck);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].headPan);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].headTilt);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].tailPan);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].tailTilt);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[1].mouth);

  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legLF1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legLF2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legLF3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legLH1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legLH2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legLH3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legRH1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legRH2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legRH3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legRF1);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legRF2);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].legRF3);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].neck);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].headPan);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].headTilt);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].tailPan);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].tailTilt);
  DOG_READ_VAR(ms->pack[this_dog].trueGain[2].mouth);

  DOG_READ_VAR(ms->pack[this_dog].trueSensors.distanceNearSnout);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.distanceFarSnout);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.distanceChest);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.accelerometer[0]);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.accelerometer[1]);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.accelerometer[2]);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.pawLF);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.pawLH);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.pawRF);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.pawRH);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.chinSensor);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.headTouch);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.backTouchR);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.backTouchM);
  DOG_READ_VAR(ms->pack[this_dog].trueSensors.backTouchF);

  cout << "dogGetSensoryMotorStates: finished" << endl;
}
END_WORD
REGISTER_WORD(DogGetSensoryMotorStates)

WORD(DogGetIndicators)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  // read until there is nothing
  flushDogBuffer(ms, this_dog); 

  // request the info 
  string request("dgbi +begin:  ledBFC.val; ledBFW.val; ledBMC.val; ledBMW.val; ledBRC.val; ledBRW.val; ledF1.val; ledF2.val; ledF3.val; ledF4.val; ledF5.val; ledF6.val; ledF7.val; ledF8.val; ledF9.val; ledF10.val; ledF11.val; ledF12.val; ledF13.val; ledF14.val; ledHC.val; modeR.val; modeG.val; modeB.val; earL.val; dgbi +end: earR.val;");
  sendOnDogSocket(ms, this_dog, request);

  // read until all info is in
  readBytesFromDogUntilString(ms, this_dog, 250, string(":dgbi] *** end\n"));

  int startCoreMessage = findStringInDogBuffer(ms, this_dog, string(":dgbi] *** begin\n"), 0);
  cout << ms->pack[this_dog].aibo_sock_buf[startCoreMessage] << endl;

  size_t idx;
  int nextCoreMessage = startCoreMessage; 
  double r = 0.0;

  //cout << ms->pack[this_dog].aibo_sock_buf[nextCoreMessage] << endl;
  //cout << "dgbi got: " << r << endl;

  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledBFC);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledBFW);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledBMC);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledBMW);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledBRC);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledBRW);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF1);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF2);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF3);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF4);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF5);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF6);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF7);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF8);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF9);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF10);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF11);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF12);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF13);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledF14);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.ledHC);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.modeR);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.modeG);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.modeB);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.earL);
  DOG_READ_VAR(ms->pack[this_dog].trueIndicators.earR);

  cout << "dogGetIndicators: finished" << endl;
  // parse the info
}
END_WORD
REGISTER_WORD(DogGetIndicators)

#define DOG_SEND_JOINT_VAR(x) << " " << #x << ".val = " << ms->pack[this_dog].intendedPose.x << ","
#define DOG_SEND_JOINT_PGAIN_VAR(x) << " " << #x << ".PGain = " << ms->pack[this_dog].intendedGain[0].x << ","
#define DOG_SEND_JOINT_IGAIN_VAR(x) << " " << #x << ".IGain = " << ms->pack[this_dog].intendedGain[1].x << ","
#define DOG_SEND_JOINT_DGAIN_VAR(x) << " " << #x << ".DGain = " << ms->pack[this_dog].intendedGain[2].x << ","
#define DOG_SEND_INDICATOR_VAR(x) << " " << #x << ".val = " << ms->pack[this_dog].intendedIndicators.x << ","

WORD(DogSendMotorState)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  stringstream ss;
/*
  ss <<
  "legLF1.val = " << ms->pack[this_dog].intendedPose.legLF1 << "; " <<
  "legLF2.val = " << ms->pack[this_dog].intendedPose.legLF2 << "; " <<
  "legLF3.val = " << ms->pack[this_dog].intendedPose.legLF3 << "; " <<
  "legLH1.val = " << ms->pack[this_dog].intendedPose.legLH1 << "; " <<
  "legLH2.val = " << ms->pack[this_dog].intendedPose.legLH2 << "; " <<
  "legLH3.val = " << ms->pack[this_dog].intendedPose.legLH3 << "; " <<
  "legRH1.val = " << ms->pack[this_dog].intendedPose.legRH1 << "; " <<
  "legRH2.val = " << ms->pack[this_dog].intendedPose.legRH2 << "; " <<
  "legRH3.val = " << ms->pack[this_dog].intendedPose.legRH3 << "; " <<
  "legRF1.val = " << ms->pack[this_dog].intendedPose.legRF1 << "; " <<
  "legRF2.val = " << ms->pack[this_dog].intendedPose.legRF2  << "; " <<
  "legRF3.val = " << ms->pack[this_dog].intendedPose.legRF3  << "; " <<
  "neck.val = " << ms->pack[this_dog].intendedPose.neck << "; " <<
  "headPan.val = " << ms->pack[this_dog].intendedPose.headPan << "; " <<
  "headTilt.val = " << ms->pack[this_dog].intendedPose.headTilt << "; " <<
  "tailPan.val = " << ms->pack[this_dog].intendedPose.tailPan << "; " <<
  "tailTilt.val = " << ms->pack[this_dog].intendedPose.tailTilt << "; " <<
  "mouth.val = " << ms->pack[this_dog].intendedPose.mouth << "; "
*/
  ss  
    DOG_SEND_JOINT_VAR(legLF1)
    DOG_SEND_JOINT_VAR(legLF2)
    DOG_SEND_JOINT_VAR(legLF3)
    DOG_SEND_JOINT_VAR(legLH1)
    DOG_SEND_JOINT_VAR(legLH2)
    DOG_SEND_JOINT_VAR(legLH3)
    DOG_SEND_JOINT_VAR(legRH1)
    DOG_SEND_JOINT_VAR(legRH2)
    DOG_SEND_JOINT_VAR(legRH3)
    DOG_SEND_JOINT_VAR(legRF1)
    DOG_SEND_JOINT_VAR(legRF2)
    DOG_SEND_JOINT_VAR(legRF3)
    DOG_SEND_JOINT_VAR(neck)
    DOG_SEND_JOINT_VAR(headPan)
    DOG_SEND_JOINT_VAR(headTilt)
    DOG_SEND_JOINT_VAR(tailPan)
    DOG_SEND_JOINT_VAR(tailTilt)
    DOG_SEND_JOINT_VAR(mouth) 
    DOG_SEND_JOINT_PGAIN_VAR(legLF1)
    DOG_SEND_JOINT_PGAIN_VAR(legLF2)
    DOG_SEND_JOINT_PGAIN_VAR(legLF3)
    DOG_SEND_JOINT_PGAIN_VAR(legLH1)
    DOG_SEND_JOINT_PGAIN_VAR(legLH2)
    DOG_SEND_JOINT_PGAIN_VAR(legLH3)
    DOG_SEND_JOINT_PGAIN_VAR(legRH1)
    DOG_SEND_JOINT_PGAIN_VAR(legRH2)
    DOG_SEND_JOINT_PGAIN_VAR(legRH3)
    DOG_SEND_JOINT_PGAIN_VAR(legRF1)
    DOG_SEND_JOINT_PGAIN_VAR(legRF2)
    DOG_SEND_JOINT_PGAIN_VAR(legRF3)
    DOG_SEND_JOINT_PGAIN_VAR(neck)
    DOG_SEND_JOINT_PGAIN_VAR(headPan)
    DOG_SEND_JOINT_PGAIN_VAR(headTilt)
    DOG_SEND_JOINT_PGAIN_VAR(tailPan)
    DOG_SEND_JOINT_PGAIN_VAR(tailTilt)
    DOG_SEND_JOINT_PGAIN_VAR(mouth) 
    DOG_SEND_JOINT_IGAIN_VAR(legLF1)
    DOG_SEND_JOINT_IGAIN_VAR(legLF2)
    DOG_SEND_JOINT_IGAIN_VAR(legLF3)
    DOG_SEND_JOINT_IGAIN_VAR(legLH1)
    DOG_SEND_JOINT_IGAIN_VAR(legLH2)
    DOG_SEND_JOINT_IGAIN_VAR(legLH3)
    DOG_SEND_JOINT_IGAIN_VAR(legRH1)
    DOG_SEND_JOINT_IGAIN_VAR(legRH2)
    DOG_SEND_JOINT_IGAIN_VAR(legRH3)
    DOG_SEND_JOINT_IGAIN_VAR(legRF1)
    DOG_SEND_JOINT_IGAIN_VAR(legRF2)
    DOG_SEND_JOINT_IGAIN_VAR(legRF3)
    DOG_SEND_JOINT_IGAIN_VAR(neck)
    DOG_SEND_JOINT_IGAIN_VAR(headPan)
    DOG_SEND_JOINT_IGAIN_VAR(headTilt)
    DOG_SEND_JOINT_IGAIN_VAR(tailPan)
    DOG_SEND_JOINT_IGAIN_VAR(tailTilt)
    DOG_SEND_JOINT_IGAIN_VAR(mouth) 
    DOG_SEND_JOINT_DGAIN_VAR(legLF1)
    DOG_SEND_JOINT_DGAIN_VAR(legLF2)
    DOG_SEND_JOINT_DGAIN_VAR(legLF3)
    DOG_SEND_JOINT_DGAIN_VAR(legLH1)
    DOG_SEND_JOINT_DGAIN_VAR(legLH2)
    DOG_SEND_JOINT_DGAIN_VAR(legLH3)
    DOG_SEND_JOINT_DGAIN_VAR(legRH1)
    DOG_SEND_JOINT_DGAIN_VAR(legRH2)
    DOG_SEND_JOINT_DGAIN_VAR(legRH3)
    DOG_SEND_JOINT_DGAIN_VAR(legRF1)
    DOG_SEND_JOINT_DGAIN_VAR(legRF2)
    DOG_SEND_JOINT_DGAIN_VAR(legRF3)
    DOG_SEND_JOINT_DGAIN_VAR(neck)
    DOG_SEND_JOINT_DGAIN_VAR(headPan)
    DOG_SEND_JOINT_DGAIN_VAR(headTilt)
    DOG_SEND_JOINT_DGAIN_VAR(tailPan)
    DOG_SEND_JOINT_DGAIN_VAR(tailTilt)
    DOG_SEND_JOINT_DGAIN_VAR(mouth) 
  ;

  string message = ss.str();
  sendOnDogSocket(ms, this_dog, message);
}
END_WORD
REGISTER_WORD(DogSendMotorState)

WORD(DogSendIndicators)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  stringstream ss;
  ss 
    DOG_SEND_INDICATOR_VAR(ledBFC)
    DOG_SEND_INDICATOR_VAR(ledBFW)
    DOG_SEND_INDICATOR_VAR(ledBMC)
    DOG_SEND_INDICATOR_VAR(ledBMW)
    DOG_SEND_INDICATOR_VAR(ledBRC)
    DOG_SEND_INDICATOR_VAR(ledBRW)
    DOG_SEND_INDICATOR_VAR(ledF1)
    DOG_SEND_INDICATOR_VAR(ledF2)
    DOG_SEND_INDICATOR_VAR(ledF3)
    DOG_SEND_INDICATOR_VAR(ledF4)
    DOG_SEND_INDICATOR_VAR(ledF5)
    DOG_SEND_INDICATOR_VAR(ledF6)
    DOG_SEND_INDICATOR_VAR(ledF7)
    DOG_SEND_INDICATOR_VAR(ledF8)
    DOG_SEND_INDICATOR_VAR(ledF9)
    DOG_SEND_INDICATOR_VAR(ledF10)
    DOG_SEND_INDICATOR_VAR(ledF11)
    DOG_SEND_INDICATOR_VAR(ledF12)
    DOG_SEND_INDICATOR_VAR(ledF13)
    DOG_SEND_INDICATOR_VAR(ledF14)
    DOG_SEND_INDICATOR_VAR(ledHC)
    DOG_SEND_INDICATOR_VAR(modeR)
    DOG_SEND_INDICATOR_VAR(modeG)
    DOG_SEND_INDICATOR_VAR(modeB)
    DOG_SEND_INDICATOR_VAR(earL)
    DOG_SEND_INDICATOR_VAR(earR)
  ;

  string message = ss.str();
  sendOnDogSocket(ms, this_dog, message);
}
END_WORD
REGISTER_WORD(DogSendIndicators)

WORD(DogStay)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;
  ms->pack[this_dog].intendedPose = ms->pack[this_dog].truePose;
  ms->pack[this_dog].intendedGain[0] = ms->pack[this_dog].trueGain[0];
  ms->pack[this_dog].intendedGain[1] = ms->pack[this_dog].trueGain[1];
  ms->pack[this_dog].intendedGain[2] = ms->pack[this_dog].trueGain[2];
}
END_WORD
REGISTER_WORD(DogStay)

WORD(DogPushIntendedPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  shared_ptr<AiboPoseWord> word = std::make_shared<AiboPoseWord>(ms->pack[this_dog].intendedPose);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(DogPushIntendedPose)

WORD(DogPushIntendedGain)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int t_gain= 0;
  GET_INT_ARG(ms, t_gain);

  int this_dog = ms->focusedMember;

  shared_ptr<AiboPoseWord> word = std::make_shared<AiboPoseWord>(ms->pack[this_dog].intendedGain[t_gain]);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(DogPushIntendedGain)

WORD(DogPushTrueGain)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int t_gain= 0;
  GET_INT_ARG(ms, t_gain);

  int this_dog = ms->focusedMember;

  shared_ptr<AiboPoseWord> word = std::make_shared<AiboPoseWord>(ms->pack[this_dog].trueGain[t_gain]);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(DogPushTrueGain)

WORD(DogCreatePose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  EinAiboJoints toPush;

  GET_NUMERIC_ARG(ms, toPush.mouth);
  GET_NUMERIC_ARG(ms, toPush.tailTilt);
  GET_NUMERIC_ARG(ms, toPush.tailPan);
  GET_NUMERIC_ARG(ms, toPush.headTilt);
  GET_NUMERIC_ARG(ms, toPush.headPan);
  GET_NUMERIC_ARG(ms, toPush.neck);
  GET_NUMERIC_ARG(ms, toPush.legRF3);
  GET_NUMERIC_ARG(ms, toPush.legRF2);
  GET_NUMERIC_ARG(ms, toPush.legRF1);
  GET_NUMERIC_ARG(ms, toPush.legRH3);
  GET_NUMERIC_ARG(ms, toPush.legRH2);
  GET_NUMERIC_ARG(ms, toPush.legRH1);
  GET_NUMERIC_ARG(ms, toPush.legLH3);
  GET_NUMERIC_ARG(ms, toPush.legLH2);
  GET_NUMERIC_ARG(ms, toPush.legLH1);
  GET_NUMERIC_ARG(ms, toPush.legLF3);
  GET_NUMERIC_ARG(ms, toPush.legLF2);
  GET_NUMERIC_ARG(ms, toPush.legLF1);

  shared_ptr<AiboPoseWord> word = std::make_shared<AiboPoseWord>(toPush);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(DogCreatePose)

WORD(DogSetIntendedPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  shared_ptr<AiboPoseWord> word ;
  GET_WORD_ARG(ms, AiboPoseWord, word);

  ms->pack[this_dog].intendedPose = word->value();
}
END_WORD
REGISTER_WORD(DogSetIntendedPose)

WORD(DogSetIntendedGain)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int t_gain = 0;
  GET_INT_ARG(ms, t_gain);

  shared_ptr<AiboPoseWord> word ;
  GET_WORD_ARG(ms, AiboPoseWord, word);

  int this_dog = ms->focusedMember;

  ms->pack[this_dog].trueGain[t_gain] = word->value();
}
END_WORD
REGISTER_WORD(DogSetIntendedGain)


#define AIBO_POSE_ACCESSOR(J, C, D) \
WORD(AiboPose ## J) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  shared_ptr<AiboPoseWord> word ; \
  GET_WORD_ARG(ms, AiboPoseWord, word); \
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(C); \
  ms->pushData(vword); \
} \
END_WORD \
REGISTER_WORD(AiboPose ## J) \
\
WORD(SetAiboPose ## J) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  double value; \
  GET_NUMERIC_ARG(ms, value); \
   \
  shared_ptr<AiboPoseWord> word; \
  GET_WORD_ARG(ms, AiboPoseWord, word); \
   \
  EinAiboJoints newPose = word->value(); \
  D = value; \
 \
  shared_ptr<AiboPoseWord> newWord = make_shared<AiboPoseWord>(newPose); \
 \
  ms->pushData(newWord); \
} \
END_WORD \
REGISTER_WORD(SetAiboPose ## J) \
/*
*/


AIBO_POSE_ACCESSOR(LegLF1  , word->value().legLF1  , newPose.legLF1  )                  
AIBO_POSE_ACCESSOR(LegLF2  , word->value().legLF2  , newPose.legLF2  )                  
AIBO_POSE_ACCESSOR(LegLF3  , word->value().legLF3  , newPose.legLF3  )                  
AIBO_POSE_ACCESSOR(LegLH1  , word->value().legLH1  , newPose.legLH1  )                  
AIBO_POSE_ACCESSOR(LegLH2  , word->value().legLH2  , newPose.legLH2  )                  
AIBO_POSE_ACCESSOR(LegLH3  , word->value().legLH3  , newPose.legLH3  )                  
AIBO_POSE_ACCESSOR(LegRH1  , word->value().legRH1  , newPose.legRH1  )                  
AIBO_POSE_ACCESSOR(LegRH2  , word->value().legRH2  , newPose.legRH2  )                  
AIBO_POSE_ACCESSOR(LegRH3  , word->value().legRH3  , newPose.legRH3  )                  
AIBO_POSE_ACCESSOR(LegRF1  , word->value().legRF1  , newPose.legRF1  )                  
AIBO_POSE_ACCESSOR(LegRF2  , word->value().legRF2  , newPose.legRF2  )                  
AIBO_POSE_ACCESSOR(LegRF3  , word->value().legRF3  , newPose.legRF3  )                  
AIBO_POSE_ACCESSOR(Neck    , word->value().neck    , newPose.neck    )                  
AIBO_POSE_ACCESSOR(HeadPan , word->value().headPan , newPose.headPan )                  
AIBO_POSE_ACCESSOR(HeadTilt, word->value().headTilt, newPose.headTilt)                  
AIBO_POSE_ACCESSOR(TailPan , word->value().tailPan , newPose.tailPan )                  
AIBO_POSE_ACCESSOR(TailTilt, word->value().tailTilt, newPose.tailTilt)                  
AIBO_POSE_ACCESSOR(Mouth   , word->value().mouth   , newPose.mouth   )                  

#define AIBO_POSE_DELTAS(J, C, D) \
WORD(Dog ## J ## Up) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
  C += D; \
} \
END_WORD \
REGISTER_WORD(Dog ## J ## Up) \
\
WORD(Dog ## J ## Down) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
  C -= D; \
} \
END_WORD \
REGISTER_WORD(Dog ## J ## Down) \
\
WORD(Dog ## J ## By) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
\
  double amount = 0.0; \
  GET_NUMERIC_ARG(ms, amount); \
  C += amount; \
} \
END_WORD \
REGISTER_WORD(Dog ## J ## By) \
WORD(Dog ## J ## To) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
\
  double amount = 0.0; \
  GET_NUMERIC_ARG(ms, amount); \
  C = amount; \
} \
END_WORD \
REGISTER_WORD(Dog ## J ## To) \

#define AIBO_POSE_DELTAS_GAIN(J, C, D) \
WORD(Dog ## J ## Up) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
  C += D; \
  C = min( max( 0.0, C ), 18.0 ); \
} \
END_WORD \
REGISTER_WORD(Dog ## J ## Up) \
\
WORD(Dog ## J ## Down) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
  C -= D; \
  C = min( max( 0.0, C ), 18.0 ); \
} \
END_WORD \
REGISTER_WORD(Dog ## J ## Down) \
\
WORD(Dog ## J ## By) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
\
  double amount = 0.0; \
  GET_NUMERIC_ARG(ms, amount); \
  C += amount; \
  C = min( max( 0.0, C ), 18.0 ); \
} \
END_WORD \
REGISTER_WORD(Dog ## J ## By) \
\
WORD(Dog ## J ## To) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
\
  double amount = 0.0; \
  GET_NUMERIC_ARG(ms, amount); \
  C = amount; \
  C = min( max( 0.0, C ), 18.0 ); \
} \
END_WORD \
REGISTER_WORD(Dog ## J ## To) \


AIBO_POSE_DELTAS(LegLF1, ms->pack[this_dog].intendedPose.legLF1, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegLF2, ms->pack[this_dog].intendedPose.legLF2, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegLF3, ms->pack[this_dog].intendedPose.legLF3, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegLH1, ms->pack[this_dog].intendedPose.legLH1, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegLH2, ms->pack[this_dog].intendedPose.legLH2, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegLH3, ms->pack[this_dog].intendedPose.legLH3, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegRH1, ms->pack[this_dog].intendedPose.legRH1, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegRH2, ms->pack[this_dog].intendedPose.legRH2, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegRH3, ms->pack[this_dog].intendedPose.legRH3, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegRF1, ms->pack[this_dog].intendedPose.legRF1, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegRF2, ms->pack[this_dog].intendedPose.legRF2, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(LegRF3, ms->pack[this_dog].intendedPose.legRF3, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(Neck, ms->pack[this_dog].intendedPose. neck, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(HeadPan, ms->pack[this_dog].intendedPose.headPan, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(HeadTilt, ms->pack[this_dog].intendedPose.headTilt, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(TailPan, ms->pack[this_dog].intendedPose.tailPan, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(TailTilt, ms->pack[this_dog].intendedPose.tailTilt, ms->pack[this_dog].dogPoseGridSize)
AIBO_POSE_DELTAS(Mouth, ms->pack[this_dog].intendedPose.mouth, ms->pack[this_dog].dogPoseGridSize)

AIBO_POSE_DELTAS_GAIN(LegLF1PGain, ms->pack[this_dog].intendedGain[0].legLF1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLF2PGain, ms->pack[this_dog].intendedGain[0].legLF2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLF3PGain, ms->pack[this_dog].intendedGain[0].legLF3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLH1PGain, ms->pack[this_dog].intendedGain[0].legLH1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLH2PGain, ms->pack[this_dog].intendedGain[0].legLH2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLH3PGain, ms->pack[this_dog].intendedGain[0].legLH3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRH1PGain, ms->pack[this_dog].intendedGain[0].legRH1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRH2PGain, ms->pack[this_dog].intendedGain[0].legRH2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRH3PGain, ms->pack[this_dog].intendedGain[0].legRH3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRF1PGain, ms->pack[this_dog].intendedGain[0].legRF1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRF2PGain, ms->pack[this_dog].intendedGain[0].legRF2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRF3PGain, ms->pack[this_dog].intendedGain[0].legRF3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(NeckPGain, ms->pack[this_dog].intendedGain[0]. neck, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(HeadPanPGain, ms->pack[this_dog].intendedGain[0].headPan, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(HeadTiltPGain, ms->pack[this_dog].intendedGain[0].headTilt, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(TailPanPGain, ms->pack[this_dog].intendedGain[0].tailPan, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(TailTiltPGain, ms->pack[this_dog].intendedGain[0].tailTilt, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(MouthPGain, ms->pack[this_dog].intendedGain[0].mouth, ms->pack[this_dog].dogGainGridSize)

AIBO_POSE_DELTAS_GAIN(LegLF1IGain, ms->pack[this_dog].intendedGain[1].legLF1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLF2IGain, ms->pack[this_dog].intendedGain[1].legLF2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLF3IGain, ms->pack[this_dog].intendedGain[1].legLF3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLH1IGain, ms->pack[this_dog].intendedGain[1].legLH1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLH2IGain, ms->pack[this_dog].intendedGain[1].legLH2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLH3IGain, ms->pack[this_dog].intendedGain[1].legLH3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRH1IGain, ms->pack[this_dog].intendedGain[1].legRH1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRH2IGain, ms->pack[this_dog].intendedGain[1].legRH2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRH3IGain, ms->pack[this_dog].intendedGain[1].legRH3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRF1IGain, ms->pack[this_dog].intendedGain[1].legRF1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRF2IGain, ms->pack[this_dog].intendedGain[1].legRF2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRF3IGain, ms->pack[this_dog].intendedGain[1].legRF3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(NeckIGain, ms->pack[this_dog].intendedGain[1]. neck, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(HeadPanIGain, ms->pack[this_dog].intendedGain[1].headPan, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(HeadTiltIGain, ms->pack[this_dog].intendedGain[1].headTilt, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(TailPanIGain, ms->pack[this_dog].intendedGain[1].tailPan, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(TailTiltIGain, ms->pack[this_dog].intendedGain[1].tailTilt, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(MouthIGain, ms->pack[this_dog].intendedGain[1].mouth, ms->pack[this_dog].dogGainGridSize)

AIBO_POSE_DELTAS_GAIN(LegLF1DGain, ms->pack[this_dog].intendedGain[2].legLF1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLF2DGain, ms->pack[this_dog].intendedGain[2].legLF2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLF3DGain, ms->pack[this_dog].intendedGain[2].legLF3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLH1DGain, ms->pack[this_dog].intendedGain[2].legLH1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLH2DGain, ms->pack[this_dog].intendedGain[2].legLH2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegLH3DGain, ms->pack[this_dog].intendedGain[2].legLH3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRH1DGain, ms->pack[this_dog].intendedGain[2].legRH1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRH2DGain, ms->pack[this_dog].intendedGain[2].legRH2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRH3DGain, ms->pack[this_dog].intendedGain[2].legRH3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRF1DGain, ms->pack[this_dog].intendedGain[2].legRF1, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRF2DGain, ms->pack[this_dog].intendedGain[2].legRF2, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(LegRF3DGain, ms->pack[this_dog].intendedGain[2].legRF3, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(NeckDGain, ms->pack[this_dog].intendedGain[2]. neck, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(HeadPanDGain, ms->pack[this_dog].intendedGain[2].headPan, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(HeadTiltDGain, ms->pack[this_dog].intendedGain[2].headTilt, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(TailPanDGain, ms->pack[this_dog].intendedGain[2].tailPan, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(TailTiltDGain, ms->pack[this_dog].intendedGain[2].tailTilt, ms->pack[this_dog].dogGainGridSize)
AIBO_POSE_DELTAS_GAIN(MouthDGain, ms->pack[this_dog].intendedGain[2].mouth, ms->pack[this_dog].dogGainGridSize)

WORD(DogSetPoseGridSize)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  double newgrid = 0.0;
  GET_NUMERIC_ARG(ms, newgrid);
  ms->pack[this_dog].dogPoseGridSize = newgrid;
  cout << "dogSetPoseGridSize: " << ms->pack[this_dog].dogPoseGridSize << endl;
}
END_WORD
REGISTER_WORD(DogSetPoseGridSize)

WORD(DogSetGainGridSize)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  double newgrid = 0.0;
  GET_NUMERIC_ARG(ms, newgrid);
  ms->pack[this_dog].dogGainGridSize = newgrid;
  cout << "dogSetGainGridSize: " << ms->pack[this_dog].dogGainGridSize << endl;
}
END_WORD
REGISTER_WORD(DogSetGainGridSize)

#define AIBO_INDICATOR_ACCESSORS(J, C) \
WORD(Dog ## J) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(C); \
  ms->pushData(vword); \
} \
END_WORD \
REGISTER_WORD(Dog ## J) \
WORD(DogSet ## J) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
  double value; \
  GET_NUMERIC_ARG(ms, value); \
   \
  C = value; \
} \
END_WORD \
REGISTER_WORD(DogSet ## J) \

AIBO_INDICATOR_ACCESSORS(IntendedLedBFC, ms->pack[this_dog].intendedIndicators.ledBFC);
AIBO_INDICATOR_ACCESSORS(IntendedLedBFW, ms->pack[this_dog].intendedIndicators.ledBFW);
AIBO_INDICATOR_ACCESSORS(IntendedLedBMC, ms->pack[this_dog].intendedIndicators.ledBMC);
AIBO_INDICATOR_ACCESSORS(IntendedLedBMW, ms->pack[this_dog].intendedIndicators.ledBMW);
AIBO_INDICATOR_ACCESSORS(IntendedLedBRC, ms->pack[this_dog].intendedIndicators.ledBRC);
AIBO_INDICATOR_ACCESSORS(IntendedLedBRW, ms->pack[this_dog].intendedIndicators.ledBRW);
AIBO_INDICATOR_ACCESSORS(IntendedLedF1, ms->pack[this_dog].intendedIndicators.ledF1);
AIBO_INDICATOR_ACCESSORS(IntendedLedF2, ms->pack[this_dog].intendedIndicators.ledF2);
AIBO_INDICATOR_ACCESSORS(IntendedLedF3, ms->pack[this_dog].intendedIndicators.ledF3);
AIBO_INDICATOR_ACCESSORS(IntendedLedF4, ms->pack[this_dog].intendedIndicators.ledF4);
AIBO_INDICATOR_ACCESSORS(IntendedLedF5, ms->pack[this_dog].intendedIndicators.ledF5);
AIBO_INDICATOR_ACCESSORS(IntendedLedF6, ms->pack[this_dog].intendedIndicators.ledF6);
AIBO_INDICATOR_ACCESSORS(IntendedLedF7, ms->pack[this_dog].intendedIndicators.ledF7);
AIBO_INDICATOR_ACCESSORS(IntendedLedF8, ms->pack[this_dog].intendedIndicators.ledF8);
AIBO_INDICATOR_ACCESSORS(IntendedLedF9, ms->pack[this_dog].intendedIndicators.ledF9);
AIBO_INDICATOR_ACCESSORS(IntendedLedF10, ms->pack[this_dog].intendedIndicators.ledF10);
AIBO_INDICATOR_ACCESSORS(IntendedLedF11, ms->pack[this_dog].intendedIndicators.ledF11);
AIBO_INDICATOR_ACCESSORS(IntendedLedF12, ms->pack[this_dog].intendedIndicators.ledF12);
AIBO_INDICATOR_ACCESSORS(IntendedLedF13, ms->pack[this_dog].intendedIndicators.ledF13);
AIBO_INDICATOR_ACCESSORS(IntendedLedF14, ms->pack[this_dog].intendedIndicators.ledF14);
AIBO_INDICATOR_ACCESSORS(IntendedLedHC, ms->pack[this_dog].intendedIndicators.ledHC);
AIBO_INDICATOR_ACCESSORS(IntendedModeR, ms->pack[this_dog].intendedIndicators.modeR);
AIBO_INDICATOR_ACCESSORS(IntendedModeG, ms->pack[this_dog].intendedIndicators.modeG);
AIBO_INDICATOR_ACCESSORS(IntendedModeB, ms->pack[this_dog].intendedIndicators.modeB);
AIBO_INDICATOR_ACCESSORS(IntendedEarL, ms->pack[this_dog].intendedIndicators.earL);
AIBO_INDICATOR_ACCESSORS(IntendedEarR, ms->pack[this_dog].intendedIndicators.earR);

#define AIBO_SENSOR_ACCESSORS(J, C) \
WORD(Dog ## J) \
virtual void execute(std::shared_ptr<MachineState> ms) { \
  int this_dog = ms->focusedMember; \
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(C); \
  ms->pushData(vword); \
} \
END_WORD \
REGISTER_WORD(Dog ## J) \

AIBO_SENSOR_ACCESSORS(TrueLedBFC, ms->pack[this_dog].trueIndicators.ledBFC);
AIBO_SENSOR_ACCESSORS(TrueLedBFW, ms->pack[this_dog].trueIndicators.ledBFW);
AIBO_SENSOR_ACCESSORS(TrueLedBMC, ms->pack[this_dog].trueIndicators.ledBMC);
AIBO_SENSOR_ACCESSORS(TrueLedBMW, ms->pack[this_dog].trueIndicators.ledBMW);
AIBO_SENSOR_ACCESSORS(TrueLedBRC, ms->pack[this_dog].trueIndicators.ledBRC);
AIBO_SENSOR_ACCESSORS(TrueLedBRW, ms->pack[this_dog].trueIndicators.ledBRW);
AIBO_SENSOR_ACCESSORS(TrueLedF1, ms->pack[this_dog].trueIndicators.ledF1);
AIBO_SENSOR_ACCESSORS(TrueLedF2, ms->pack[this_dog].trueIndicators.ledF2);
AIBO_SENSOR_ACCESSORS(TrueLedF3, ms->pack[this_dog].trueIndicators.ledF3);
AIBO_SENSOR_ACCESSORS(TrueLedF4, ms->pack[this_dog].trueIndicators.ledF4);
AIBO_SENSOR_ACCESSORS(TrueLedF5, ms->pack[this_dog].trueIndicators.ledF5);
AIBO_SENSOR_ACCESSORS(TrueLedF6, ms->pack[this_dog].trueIndicators.ledF6);
AIBO_SENSOR_ACCESSORS(TrueLedF7, ms->pack[this_dog].trueIndicators.ledF7);
AIBO_SENSOR_ACCESSORS(TrueLedF8, ms->pack[this_dog].trueIndicators.ledF8);
AIBO_SENSOR_ACCESSORS(TrueLedF9, ms->pack[this_dog].trueIndicators.ledF9);
AIBO_SENSOR_ACCESSORS(TrueLedF10, ms->pack[this_dog].trueIndicators.ledF10);
AIBO_SENSOR_ACCESSORS(TrueLedF11, ms->pack[this_dog].trueIndicators.ledF11);
AIBO_SENSOR_ACCESSORS(TrueLedF12, ms->pack[this_dog].trueIndicators.ledF12);
AIBO_SENSOR_ACCESSORS(TrueLedF13, ms->pack[this_dog].trueIndicators.ledF13);
AIBO_SENSOR_ACCESSORS(TrueLedF14, ms->pack[this_dog].trueIndicators.ledF14);
AIBO_SENSOR_ACCESSORS(TrueLedHC, ms->pack[this_dog].trueIndicators.ledHC);
AIBO_SENSOR_ACCESSORS(TrueModeR, ms->pack[this_dog].trueIndicators.modeR);
AIBO_SENSOR_ACCESSORS(TrueModeG, ms->pack[this_dog].trueIndicators.modeG);
AIBO_SENSOR_ACCESSORS(TrueModeB, ms->pack[this_dog].trueIndicators.modeB);
AIBO_SENSOR_ACCESSORS(TrueEarL, ms->pack[this_dog].trueIndicators.earL);
AIBO_SENSOR_ACCESSORS(TrueEarR, ms->pack[this_dog].trueIndicators.earR);

AIBO_SENSOR_ACCESSORS(DistanceNearSnout, ms->pack[this_dog].trueSensors.distanceNearSnout);
AIBO_SENSOR_ACCESSORS(DistanceFarSnout, ms->pack[this_dog].trueSensors.distanceFarSnout);
AIBO_SENSOR_ACCESSORS(DistanceChest, ms->pack[this_dog].trueSensors.distanceChest);
AIBO_SENSOR_ACCESSORS(PawLF, ms->pack[this_dog].trueSensors.pawLF);
AIBO_SENSOR_ACCESSORS(PawLH, ms->pack[this_dog].trueSensors.pawLH);
AIBO_SENSOR_ACCESSORS(PawRF, ms->pack[this_dog].trueSensors.pawRF);
AIBO_SENSOR_ACCESSORS(PawRH, ms->pack[this_dog].trueSensors.pawRH);
AIBO_SENSOR_ACCESSORS(ChinSensor, ms->pack[this_dog].trueSensors.chinSensor);
AIBO_SENSOR_ACCESSORS(HeadTouch, ms->pack[this_dog].trueSensors.headTouch ); 
AIBO_SENSOR_ACCESSORS(BackTouchR, ms->pack[this_dog].trueSensors.backTouchR);
AIBO_SENSOR_ACCESSORS(BackTouchM, ms->pack[this_dog].trueSensors.backTouchM);
AIBO_SENSOR_ACCESSORS(BackTouchF, ms->pack[this_dog].trueSensors.backTouchF);

AIBO_SENSOR_ACCESSORS(AccelerometerX, ms->pack[this_dog].trueSensors.accelerometer[0]);
AIBO_SENSOR_ACCESSORS(AccelerometerY, ms->pack[this_dog].trueSensors.accelerometer[1]);
AIBO_SENSOR_ACCESSORS(AccelerometerZ, ms->pack[this_dog].trueSensors.accelerometer[2]);

WORD(DogReplaceWristImageWithSnoutImage)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;
  if (!isSketchyMat(ms->pack[this_dog].snoutCamImage)) {
    if ( (ms->pack[this_dog].snoutCamImage.rows == 400) && 
	 (ms->pack[this_dog].snoutCamImage.cols== 600) ) {
      ms->config.wristCamImage = ms->pack[this_dog].snoutCamImage.clone();
    } else {
      Mat tobo;
      Size p_toBecome(640, 400);
      cv::resize(ms->pack[this_dog].snoutCamImage, tobo, p_toBecome);
      ms->config.wristCamImage = tobo;
    }
  } else {
    ROS_ERROR_STREAM("dogReplaceWristImageWithSnoutImage: snoutCamImage was sketchy..." << ms->pack[this_dog].snoutCamImage.size() << endl);
  }
}
END_WORD
REGISTER_WORD(DogReplaceWristImageWithSnoutImage)

WORD(DogSendToneSin)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  double d = 0.0;
  GET_NUMERIC_ARG(ms, d);

  double f = 0.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  int tone_samples = ceil(16000*d);
  int tone_length = 2*tone_samples;

  stringstream ss;
  ss << "speaker.val = BIN " << tone_length << " raw 1 16000 16 1;";
  string header = ss.str();

  int buf_size = tone_length + header.size();
  char * buf = new char[buf_size];
  sprintf(buf, "%s", header.c_str());

  int16_t * currentSample = (int16_t*)(buf + header.size());

  for (int i = 0; i < tone_samples; i++) {
    int16_t intended = int16_t(v * sin(i * f * 2.0 * M_PI / 16000) );
    currentSample[i] = intended;
    cout << int(currentSample[i]) << " " << intended << endl;;
  }

  sendOnDogSocket(ms, this_dog, buf, buf_size);

  delete buf;
}
END_WORD
REGISTER_WORD(DogSendToneSin)

double squareWave(double t) {
  t = t - floor(t / (2.0*M_PI))*2.0*M_PI;
  if (t > 0) {
    if (t > M_PI) {
      return -1.0;
    } else {
      return 1.0;
    }
  } else {
    if (t > -M_PI) {
      return -1.0;
    } else {
      return 1.0;
    }
  }
}

double sawtoothWave(double t) {
  t = t - floor(t / (2.0*M_PI))*2.0*M_PI;
  if (t > 0) {
    return ( t/M_PI ) - 1.0;
  } else {
    double t2 = t + ( 2.0*M_PI );
    return ( t2/M_PI ) - 1.0;
  }
}

double triangleWave(double t) {
  t = t - floor(t / (2.0*M_PI))*2.0*M_PI;
  if (t > 0) {
    double t2 = max( double(M_PI-t), double(t-M_PI) );
    return ( 2*t2/M_PI ) - 1.0;
  } else {
    double t2 = max( double(-M_PI-t), double(t-(-M_PI)) );
    return ( 2*t2/M_PI ) - 1.0;
  }
}

WORD(DogSendToneSquare)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  double d = 0.0;
  GET_NUMERIC_ARG(ms, d);

  double f = 0.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  int tone_samples = ceil(16000*d);
  int tone_length = 2*tone_samples;

  stringstream ss;
  ss << "speaker.val = BIN " << tone_length << " raw 1 16000 16 1;";
  string header = ss.str();

  int buf_size = tone_length + header.size();
  char * buf = new char[buf_size];
  sprintf(buf, "%s", header.c_str());

  int16_t * currentSample = (int16_t*)(buf + header.size());

  for (int i = 0; i < tone_samples; i++) {
    int16_t intended = int16_t(v * squareWave(i * f * 2.0 * M_PI / 16000) );
    currentSample[i] = intended;
    cout << int(currentSample[i]) << " " << intended << endl;;
  }

  sendOnDogSocket(ms, this_dog, buf, buf_size);

  delete buf;
}
END_WORD
REGISTER_WORD(DogSendToneSquare)

WORD(DogMorse)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  //double dmp = ms->pack[this_dog].dog_morse_period;
  double dmp = 0.1;
  GET_NUMERIC_ARG(ms, dmp);

  double f = 400.0;
  GET_NUMERIC_ARG(ms, f);

  double _v = 128.0;
  GET_NUMERIC_ARG(ms, _v);

  string message;
  GET_STRING_ARG(ms, message);

  stringstream binarizedss;

  for (int l = 0; l < message.size(); l++) {
    if (message[l] == ' ') {
      binarizedss << "0000";
    } else if ( (message[l] == 'a') || (message[l] == 'A') ) {
      binarizedss << "10111";
    } else if ( (message[l] == 'b') || (message[l] == 'B') ) {
      binarizedss << "111010101";
    } else if ( (message[l] == 'c') || (message[l] == 'C') ) {
      binarizedss << "11101011101";
    } else if ( (message[l] == 'd') || (message[l] == 'D') ) {
      binarizedss << "1110101";
    } else if ( (message[l] == 'e') || (message[l] == 'E') ) {
      binarizedss << "1";
    } else if ( (message[l] == 'f') || (message[l] == 'F') ) {
      binarizedss << "101011101";
    } else if ( (message[l] == 'g') || (message[l] == 'G') ) {
      binarizedss << "111011101";
    } else if ( (message[l] == 'h') || (message[l] == 'H') ) {
      binarizedss << "1010101";
    } else if ( (message[l] == 'i') || (message[l] == 'I') ) {
      binarizedss << "101";
    } else if ( (message[l] == 'j') || (message[l] == 'J') ) {
      binarizedss << "1011101110111";
    } else if ( (message[l] == 'k') || (message[l] == 'K') ) {
      binarizedss << "1110111";
    } else if ( (message[l] == 'l') || (message[l] == 'L') ) {
      binarizedss << "101110101";
    } else if ( (message[l] == 'm') || (message[l] == 'M') ) {
      binarizedss << "1110111";
    } else if ( (message[l] == 'n') || (message[l] == 'N') ) {
      binarizedss << "11101";
    } else if ( (message[l] == 'o') || (message[l] == 'O') ) {
      binarizedss << "11101110111";
    } else if ( (message[l] == 'p') || (message[l] == 'P') ) {
      binarizedss << "10111011101";
    } else if ( (message[l] == 'q') || (message[l] == 'Q') ) {
      binarizedss << "1110111010111";
    } else if ( (message[l] == 'r') || (message[l] == 'R') ) {
      binarizedss << "1011101";
    } else if ( (message[l] == 's') || (message[l] == 'S') ) {
      binarizedss << "10101";
    } else if ( (message[l] == 't') || (message[l] == 'T') ) {
      binarizedss << "111";
    } else if ( (message[l] == 'u') || (message[l] == 'U') ) {
      binarizedss << "1010111";
    } else if ( (message[l] == 'v') || (message[l] == 'V') ) {
      binarizedss << "101010111";
    } else if ( (message[l] == 'w') || (message[l] == 'W') ) {
      binarizedss << "101110111";
    } else if ( (message[l] == 'x') || (message[l] == 'X') ) {
      binarizedss << "11101010111";
    } else if ( (message[l] == 'y') || (message[l] == 'Y') ) {
      binarizedss << "1110101110111";
    } else if ( (message[l] == 'z') || (message[l] == 'Z') ) {
      binarizedss << "11101110101";
    } else if ( (message[l] == '0') ) {
      binarizedss << "1110111011101110111";
    } else if ( (message[l] == '1') ) {
      binarizedss << "10111011101110111";
    } else if ( (message[l] == '2') ) {
      binarizedss << "101011101110111";
    } else if ( (message[l] == '3') ) {
      binarizedss << "1010101110111";
    } else if ( (message[l] == '4') ) {
      binarizedss << "10101010111";
    } else if ( (message[l] == '5') ) {
      binarizedss << "101010101";
    } else if ( (message[l] == '6') ) {
      binarizedss << "11101010101";
    } else if ( (message[l] == '7') ) {
      binarizedss << "1110111010101";
    } else if ( (message[l] == '8') ) {
      binarizedss << "111011101110101";
    } else if ( (message[l] == '9') ) {
      binarizedss << "11101110111011101";
    } else {
    }
    binarizedss << "000";
  }

  string binarized = binarizedss.str();

  double samples_per_dmp = 16000 * dmp;
  double d = dmp * binarized.size();

  int tone_samples = ceil(16000*d);
  int tone_length = 2*tone_samples;

  stringstream ss;
  ss << "speaker.val = BIN " << tone_length << " raw 1 16000 16 1;";
  string header = ss.str();

  int buf_size = tone_length + header.size();
  char * buf = new char[buf_size];
  sprintf(buf, "%s", header.c_str());

  int16_t * currentSample = (int16_t*)(buf + header.size());

  double v = _v;

  for (int i = 0; i < tone_samples; i++) {
    int this_slot = floor(i / samples_per_dmp);
    this_slot = max(0, min((int)this_slot, (int)binarized.size()-1));

    if (binarized[this_slot] == '0') {
      v = 0;
    } else if (binarized[this_slot] == '1') {
      v = _v;
    } else {
      ROS_ERROR_STREAM("Oops, dogMorse encountered an error during encoding." << endl);
    }

    int16_t intended = int16_t(v * sin(i * f * 2.0 * M_PI / 16000) );
    currentSample[i] = intended;
    //cout << int(currentSample[i]) << " " << intended << endl;;
  }

  sendOnDogSocket(ms, this_dog, buf, buf_size);
}
END_WORD
REGISTER_WORD(DogMorse)

WORD(DogChirpSin)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  double sigma = 1.0;
  GET_NUMERIC_ARG(ms, sigma);

  double d = 0.0;
  GET_NUMERIC_ARG(ms, d);

  double f = 0.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  int tone_samples = ceil(16000*d);
  int tone_length = 2*tone_samples;

  stringstream ss;
  ss << "speaker.val = BIN " << tone_length << " raw 1 16000 16 1;";
  string header = ss.str();

  int buf_size = tone_length + header.size();
  char * buf = new char[buf_size];
  sprintf(buf, "%s", header.c_str());

  int16_t * currentSample = (int16_t*)(buf + header.size());

  for (int i = 0; i < tone_samples; i++) {
    int16_t intended = int16_t(  v * sin(i * f * 2.0 * M_PI / 16000) * exp( -pow((tone_samples/2.0)-i,2)/(2.0*pow(sigma,2.0)) )  );
    currentSample[i] = intended;
    cout << int(currentSample[i]) << " " << intended << endl;;
  }

  sendOnDogSocket(ms, this_dog, buf, buf_size);

  delete buf;
}
END_WORD
REGISTER_WORD(DogChirpSin)

WORD(DogChirpSquare)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  double sigma = 1.0;
  GET_NUMERIC_ARG(ms, sigma);

  double d = 0.0;
  GET_NUMERIC_ARG(ms, d);

  double f = 0.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  int tone_samples = ceil(16000*d);
  int tone_length = 2*tone_samples;

  stringstream ss;
  ss << "speaker.val = BIN " << tone_length << " raw 1 16000 16 1;";
  string header = ss.str();

  int buf_size = tone_length + header.size();
  char * buf = new char[buf_size];
  sprintf(buf, "%s", header.c_str());

  int16_t * currentSample = (int16_t*)(buf + header.size());

  for (int i = 0; i < tone_samples; i++) {
    int16_t intended = int16_t(  v * squareWave(i * f * 2.0 * M_PI / 16000) * exp( -pow((tone_samples/2.0)-i,2)/(2.0*pow(sigma,2.0)) )  );
    currentSample[i] = intended;
    cout << int(currentSample[i]) << " " << intended << endl;;
  }

  sendOnDogSocket(ms, this_dog, buf, buf_size);

  delete buf;
}
END_WORD
REGISTER_WORD(DogChirpSquare)


WORD(DogWarbleSin)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  double warb_width_seconds = 1.0;
  GET_NUMERIC_ARG(ms, warb_width_seconds);

  double sigma = 1.0;
  GET_NUMERIC_ARG(ms, sigma);

  double d = 0.0;
  GET_NUMERIC_ARG(ms, d);

  double f = 0.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  int tone_samples = ceil(16000*d);
  int tone_length = 2*tone_samples;

  int warb_width_samples = ceil(warb_width_seconds * 16000);

  stringstream ss;
  ss << "speaker.val = BIN " << tone_length << " raw 1 16000 16 1;";
  string header = ss.str();

  int buf_size = tone_length + header.size();
  char * buf = new char[buf_size];
  sprintf(buf, "%s", header.c_str());

  int16_t * currentSample = (int16_t*)(buf + header.size());

  for (int i = 0; i < tone_samples; i++) {
    int sample_idx = i % warb_width_samples;
    int16_t intended = int16_t(  v * sin(i * f * 2.0 * M_PI / 16000) * exp( -pow((warb_width_samples/2.0)-sample_idx,2)/(2.0*pow(sigma,2.0)) )  );
    currentSample[i] = intended;
    cout << int(currentSample[i]) << " " << intended << endl;;
  }

  sendOnDogSocket(ms, this_dog, buf, buf_size);

  delete buf;
}
END_WORD
REGISTER_WORD(DogWarbleSin)

WORD(DogWarbleSquare)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  double warb_width_seconds = 1.0;
  GET_NUMERIC_ARG(ms, warb_width_seconds);

  double sigma = 1.0;
  GET_NUMERIC_ARG(ms, sigma);

  double d = 0.0;
  GET_NUMERIC_ARG(ms, d);

  double f = 0.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  int tone_samples = ceil(16000*d);
  int tone_length = 2*tone_samples;

  int warb_width_samples = ceil(warb_width_seconds * 16000);

  stringstream ss;
  ss << "speaker.val = BIN " << tone_length << " raw 1 16000 16 1;";
  string header = ss.str();

  int buf_size = tone_length + header.size();
  char * buf = new char[buf_size];
  sprintf(buf, "%s", header.c_str());

  int16_t * currentSample = (int16_t*)(buf + header.size());

  for (int i = 0; i < tone_samples; i++) {
    int sample_idx = i % warb_width_samples;
    int16_t intended = int16_t(  v * squareWave(i * f * 2.0 * M_PI / 16000) * exp( -pow((warb_width_samples/2.0)-sample_idx,2)/(2.0*pow(sigma,2.0)) )  );
    currentSample[i] = intended;
    cout << int(currentSample[i]) << " " << intended << endl;;
  }

  sendOnDogSocket(ms, this_dog, buf, buf_size);

  delete buf;
}
END_WORD
REGISTER_WORD(DogWarbleSquare)


WORD(DogVoiceInit)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  double seconds = 0;
  GET_NUMERIC_ARG(ms, seconds);

  if ( ms->pack[this_dog].voice_buffer != NULL ) {
    delete ms->pack[this_dog].voice_buffer;
  } else {}

  ms->pack[this_dog].voice_buffer_size = 16000*seconds;
  ms->pack[this_dog].voice_buffer = new double[ms->pack[this_dog].voice_buffer_size];

  cout << "dogVoiceInit made buffer of " << ms->pack[this_dog].voice_buffer_size << " samples." << endl;
}
END_WORD
REGISTER_WORD(DogVoiceInit)

WORD(DogVoiceClear)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;

  if (ms->pack[this_dog].voice_buffer != NULL) {
    for (int i = 0; i < ms->pack[this_dog].voice_buffer_size; i++) {
      ms->pack[this_dog].voice_buffer[i] = 0.0;
    }
  }
  cout << "dogVoiceClear: clearing." << endl;
}
END_WORD
REGISTER_WORD(DogVoiceClear)

WORD(DogVoiceTrackTone)
virtual void execute(std::shared_ptr<MachineState> ms) {
// volume frequency start end dogVoiceTrackTone
  int this_dog = ms->focusedMember;

  double e = 0.0;
  GET_NUMERIC_ARG(ms, e);

  double s = 0.0;
  GET_NUMERIC_ARG(ms, s);

  double f = 0.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  int tone_samples_start = max( 0, min( int(ceil(16000*s)), ms->pack[this_dog].voice_buffer_size-1 ) );
  int tone_samples_end = max( 0, min( int(ceil(16000*e)), ms->pack[this_dog].voice_buffer_size-1 ) );

  for (int i = tone_samples_start; i <= tone_samples_end; i++) {
    ms->pack[this_dog].voice_buffer[i] += v * sin(i * f * 2.0 * M_PI / 16000);
  }
}
END_WORD
REGISTER_WORD(DogVoiceTrackTone)

WORD(DogVoiceTrackWarble)
virtual void execute(std::shared_ptr<MachineState> ms) {
// volume frequency start end window_sigma repetition_period dogVoiceTrackWarble
  int this_dog = ms->focusedMember;

  double warb_width_seconds = 1.0;
  GET_NUMERIC_ARG(ms, warb_width_seconds);

  double sigma = 1.0;
  GET_NUMERIC_ARG(ms, sigma);

  double e = 0.0;
  GET_NUMERIC_ARG(ms, e);

  double s = 0.0;
  GET_NUMERIC_ARG(ms, s);

  double f = 0.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  int tone_samples_start = max( 0, min( int(ceil(16000*s)), ms->pack[this_dog].voice_buffer_size-1 ) );
  int tone_samples_end = max( 0, min( int(ceil(16000*e)), ms->pack[this_dog].voice_buffer_size-1 ) );

  int warb_width_samples = ceil(warb_width_seconds * 16000);

  for (int i = tone_samples_start; i <= tone_samples_end; i++) {
    int sample_idx = i % warb_width_samples;
    ms->pack[this_dog].voice_buffer[i] += v * sin(i * f * 2.0 * M_PI / 16000) * exp( -pow((warb_width_samples/2.0)-sample_idx,2)/(2.0*pow(sigma,2.0)) );
  }
}
END_WORD
REGISTER_WORD(DogVoiceTrackWarble)

WORD(DogVoiceTrackWarbleNotes)
virtual void execute(std::shared_ptr<MachineState> ms) {
// volume a4_frequency start end window_sigma repetition_period advance_fraction "A3# B2 C4b R R" dogVoiceTrackWarbleNotes
  int this_dog = ms->focusedMember;

  string notes;
  GET_STRING_ARG(ms, notes);

  double a = 0.0;
  GET_NUMERIC_ARG(ms, a);

  double warb_width_seconds = 1.0;
  GET_NUMERIC_ARG(ms, warb_width_seconds);

  double sigma = 1.0;
  GET_NUMERIC_ARG(ms, sigma);

  double e = 0.0;
  GET_NUMERIC_ARG(ms, e);

  double s = 0.0;
  GET_NUMERIC_ARG(ms, s);

  double f = 440.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  vector<double> note_fs;
  for (int c = 0; c < notes.size(); c++) {
    double this_f = f;

    if (notes[c] == 'R' || notes[c] == 'r') {
      this_f *= 0;
    } else if (notes[c] == 'a' || notes[c] == 'A') {
    } else if (notes[c] == 'b' || notes[c] == 'B') {
      this_f *= pow(2.0, 2.0/12.0);
    } else if (notes[c] == 'c' || notes[c] == 'C') {
      this_f *= pow(2.0, -9.0/12.0);
    } else if (notes[c] == 'd' || notes[c] == 'D') {
      this_f *= pow(2.0, -7/12.0);
    } else if (notes[c] == 'e' || notes[c] == 'E') {
      this_f *= pow(2.0, -5.0/12.0);
    } else if (notes[c] == 'f' || notes[c] == 'F') {
      this_f *= pow(2.0, -4.0/12.0);
    } else if (notes[c] == 'g' || notes[c] == 'G') {
      this_f *= pow(2.0, -2.0/12.0);
    } else if (notes[c] == ' ') {
      continue;
    } else {
    }

    c++;
    if ( !(c < notes.size()) ) {
      note_fs.push_back(this_f);
      cout << "pushing " << this_f << " ran out first" << endl;
      break;
    }

    if (notes[c] == ' ') {
      note_fs.push_back(this_f);
      cout << "pushing " << this_f << " hit space first" << endl;
      continue;
    } else if (notes[c] == '0') {
      this_f *= pow(2.0,-4);
    } else if (notes[c] == '1') {
      this_f *= pow(2.0,-3);
    } else if (notes[c] == '2') {
      this_f *= pow(2.0,-2);
    } else if (notes[c] == '3') {
      this_f *= pow(2.0,-1);
    } else if (notes[c] == '4') {
    } else if (notes[c] == '5') {
      this_f *= 2;
    } else if (notes[c] == '6') {
      this_f *= 4;
    } else if (notes[c] == '7') {
      this_f *= 8;
    } else if (notes[c] == '8') {
      this_f *= 16;
    } else if (notes[c] == '9') {
      this_f *= 32;
    } else if (notes[c] == '#') {
      this_f *= pow(2.0, 1.0/12.0);
    } else if (notes[c] == 'b') {
      this_f *= pow(2.0, -1.0/12.0);
    } else {
    }

    c++;
    if ( !(c < notes.size()) ) {
      note_fs.push_back(this_f);
      cout << "pushing " << this_f << " ran out second" << endl;
      break;
    }

    if (notes[c] == ' ') {
      note_fs.push_back(this_f);
      cout << "pushing " << this_f << " hit space second" << endl;
      continue;
    } else if (notes[c] == '0') {
      this_f *= pow(2.0,-4);
    } else if (notes[c] == '1') {
      this_f *= pow(2.0,-3);
    } else if (notes[c] == '2') {
      this_f *= pow(2.0,-2);
    } else if (notes[c] == '3') {
      this_f *= pow(2.0,-1);
    } else if (notes[c] == '4') {
    } else if (notes[c] == '5') {
      this_f *= 2;
    } else if (notes[c] == '6') {
      this_f *= 4;
    } else if (notes[c] == '7') {
      this_f *= 8;
    } else if (notes[c] == '8') {
      this_f *= 16;
    } else if (notes[c] == '9') {
      this_f *= 32;
    } else if (notes[c] == '#') {
      this_f *= pow(2.0, 1.0/12.0);
    } else if (notes[c] == 'b') {
      this_f *= pow(2.0, -1.0/12.0);
    } else {
    }

    note_fs.push_back(this_f);
    cout << "pushing " << this_f << " finished" << endl;
  }

  if (note_fs.size() == 0) {
    double this_f = f;
    note_fs.push_back(this_f);
    cout << "pushing " << this_f << " by default" << endl;
  } else {}


  int tone_samples_start = max( 0, min( int(ceil(16000*s)), ms->pack[this_dog].voice_buffer_size-1 ) );
  int tone_samples_end = max( 0, min( int(ceil(16000*e)), ms->pack[this_dog].voice_buffer_size-1 ) );

  int warb_width_samples = ceil(warb_width_seconds * 16000);

  int advance_samples = a * double(warb_width_samples);

  cout << advance_samples << " adv samp " << endl;

  for (int i = tone_samples_start; i <= tone_samples_end; i++) {
    int advanced_i = (i + advance_samples);
    int sample_idx = advanced_i % warb_width_samples;
    int freq_idx = ( i / warb_width_samples ) % note_fs.size();
    int this_f = note_fs[ freq_idx ];

    //int sample_arg = min(warb_width_samples-1-sample_idx, sample_idx);
    //int sample_arg = sample_idx;
    //int16_t intended = int16_t(  v * sin(advanced_i * f * 2.0 * M_PI / 16000) * exp( -pow(sample_arg,2)/(2.0*pow(sigma,2.0)) )  );


//cout << "this_f synth " << this_f << endl;
    ms->pack[this_dog].voice_buffer[i] += v * sin(i * this_f * 2.0 * M_PI / 16000) * exp( -pow((warb_width_samples/2.0)-sample_idx,2)/(2.0*pow(sigma,2.0)) );
    //ms->pack[this_dog].voice_buffer[i] += v * sin(i * this_f * 2.0 * M_PI / 16000) * exp( -pow(sample_arg,2)/(2.0*pow(sigma,2.0)) );
  }
}
END_WORD
REGISTER_WORD(DogVoiceTrackWarbleNotes)

WORD(DogVoiceTimeTrackWarbleSinNotes)
virtual void execute(std::shared_ptr<MachineState> ms) {
// volume a4_frequency start end window_sigma repetition_period advance_fraction "A3# B2 C4b R R" dogVoiceTrackWarbleNotes
  int this_dog = ms->focusedMember;

  string notes;
  GET_STRING_ARG(ms, notes);

  double a = 0.0;
  GET_NUMERIC_ARG(ms, a);

  double warb_width_seconds = 1.0;
  GET_NUMERIC_ARG(ms, warb_width_seconds);

  double sigma = 1.0;
  GET_NUMERIC_ARG(ms, sigma);

  double e = 0.0;
  GET_NUMERIC_ARG(ms, e);

  double s = 0.0;
  GET_NUMERIC_ARG(ms, s);

  double f = 440.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  vector<double> note_fs;
  vector<double> note_ws;
  for (int c = 0; c < notes.size(); c++) {
    double this_f = f;
    double this_w = warb_width_seconds;

    int parsed = 0;

    while ( (c < notes.size()) ) {
      cout << notes[c] << " note " << endl;
      if (notes[c] == ' ') {
	cout << "breaking " << this_f << " hit space" << endl;
	break;
      } else if (notes[c] == 'R' || notes[c] == 'r') {
	this_f *= 0;
      } else if (notes[c] == 'a' || notes[c] == 'A') {
      } else if (notes[c] == 'b' || notes[c] == 'B') {
	this_f *= pow(2.0, 2.0/12.0);
      } else if (notes[c] == 'c' || notes[c] == 'C') {
	this_f *= pow(2.0, -9.0/12.0);
      } else if (notes[c] == 'd' || notes[c] == 'D') {
	this_f *= pow(2.0, -7/12.0);
      } else if (notes[c] == 'e' || notes[c] == 'E') {
	this_f *= pow(2.0, -5.0/12.0);
      } else if (notes[c] == 'f' || notes[c] == 'F') {
	this_f *= pow(2.0, -4.0/12.0);
      } else if (notes[c] == 'g' || notes[c] == 'G') {
	this_f *= pow(2.0, -2.0/12.0);
      } else if (notes[c] == '0') {
	this_f *= pow(2.0,-4);
      } else if (notes[c] == '1') {
	this_f *= pow(2.0,-3);
      } else if (notes[c] == '2') {
	this_f *= pow(2.0,-2);
      } else if (notes[c] == '3') {
	this_f *= pow(2.0,-1);
      } else if (notes[c] == '4') {
      } else if (notes[c] == '5') {
	this_f *= 2;
      } else if (notes[c] == '6') {
	this_f *= 4;
      } else if (notes[c] == '7') {
	this_f *= 8;
      } else if (notes[c] == '8') {
	this_f *= 16;
      } else if (notes[c] == '9') {
	this_f *= 32;
      } else if (notes[c] == '#') {
	this_f *= pow(2.0, 1.0/12.0);
      } else if (notes[c] == 'p') {
	this_f *= pow(2.0, -1.0/12.0);
      } else if (notes[c] == 'h') {
	this_w *= 0.5;
      } else if (notes[c] == 'H') {
	this_w *= 2;
      } else if (notes[c] == 't') {
	this_w *= 1.0/3.0;
      } else if (notes[c] == 'T') {
	this_w *= 3.0;
      } else {
      }
      parsed++;
      c++;
    }

    if (parsed > 0) {
      note_fs.push_back(this_f);
      note_ws.push_back(this_w);
      cout << "pushing f w: " << this_f << " " << this_w << " finished" << endl;
    } else {
      cout << "did not parse non-whitespace, not pushing" << endl;
    }
  }

  if (note_fs.size() == 0) {
    double this_f = f;
    note_fs.push_back(this_f);
    cout << "pushing " << this_f << " by default" << endl;
  } else {}


  int tone_samples_start = max( 0, min( int(ceil(16000*s)), ms->pack[this_dog].voice_buffer_size-1 ) );
  int tone_samples_end = max( 0, min( int(ceil(16000*e)), ms->pack[this_dog].voice_buffer_size-1 ) );


  int current_note = 0;
  int num_notes = note_fs.size();
  int cn_sample_offset = 0;
  int played_samples = 0;

  for (int i = tone_samples_start; i <= tone_samples_end; i++) {
    int freq_idx = current_note;
    double this_f = note_fs[ freq_idx ];
    double this_w = note_ws[ freq_idx ];

    int warb_width_samples = ceil(this_w * 16000);
    int advance_samples = a * double(warb_width_samples);

    int advanced_i = (i + advance_samples);
    int sample_idx = advanced_i - cn_sample_offset;

    //int sample_arg = min(warb_width_samples-1-sample_idx, sample_idx);
    //int sample_arg = sample_idx;
    //int16_t intended = int16_t(  v * sin(advanced_i * f * 2.0 * M_PI / 16000) * exp( -pow(sample_arg,2)/(2.0*pow(sigma,2.0)) )  );


//cout << "this_f synth " << this_f << endl;
    ms->pack[this_dog].voice_buffer[i] += v * sin(i * this_f * 2.0 * M_PI / 16000) * exp( -pow((warb_width_samples/2.0)-sample_idx,2)/(2.0*pow(sigma,2.0)) );
    //ms->pack[this_dog].voice_buffer[i] += v * sin(i * this_f * 2.0 * M_PI / 16000) * exp( -pow(sample_arg,2)/(2.0*pow(sigma,2.0)) );
  
    played_samples++;
    if (played_samples - cn_sample_offset >= warb_width_samples) {
      current_note++;
      cn_sample_offset = cn_sample_offset + warb_width_samples;
      cout << advance_samples << " adv samp, " << played_samples << " " << current_note << endl;
    } else {}
  
    current_note = current_note % num_notes;
  }
}
END_WORD
REGISTER_WORD(DogVoiceTimeTrackWarbleSinNotes)

WORD(DogVoiceTimeTrackWarbleSquareNotes)
virtual void execute(std::shared_ptr<MachineState> ms) {
// volume a4_frequency start end window_sigma repetition_period advance_fraction "A3# B2 C4b R R" dogVoiceTrackWarbleNotes
  int this_dog = ms->focusedMember;

  string notes;
  GET_STRING_ARG(ms, notes);

  double a = 0.0;
  GET_NUMERIC_ARG(ms, a);

  double warb_width_seconds = 1.0;
  GET_NUMERIC_ARG(ms, warb_width_seconds);

  double sigma = 1.0;
  GET_NUMERIC_ARG(ms, sigma);

  double e = 0.0;
  GET_NUMERIC_ARG(ms, e);

  double s = 0.0;
  GET_NUMERIC_ARG(ms, s);

  double f = 440.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  vector<double> note_fs;
  vector<double> note_ws;
  for (int c = 0; c < notes.size(); c++) {
    double this_f = f;
    double this_w = warb_width_seconds;

    int parsed = 0;

    while ( (c < notes.size()) ) {
      cout << notes[c] << " note " << endl;
      if (notes[c] == ' ') {
	cout << "breaking " << this_f << " hit space" << endl;
	break;
      } else if (notes[c] == 'R' || notes[c] == 'r') {
	this_f *= 0;
      } else if (notes[c] == 'a' || notes[c] == 'A') {
      } else if (notes[c] == 'b' || notes[c] == 'B') {
	this_f *= pow(2.0, 2.0/12.0);
      } else if (notes[c] == 'c' || notes[c] == 'C') {
	this_f *= pow(2.0, -9.0/12.0);
      } else if (notes[c] == 'd' || notes[c] == 'D') {
	this_f *= pow(2.0, -7/12.0);
      } else if (notes[c] == 'e' || notes[c] == 'E') {
	this_f *= pow(2.0, -5.0/12.0);
      } else if (notes[c] == 'f' || notes[c] == 'F') {
	this_f *= pow(2.0, -4.0/12.0);
      } else if (notes[c] == 'g' || notes[c] == 'G') {
	this_f *= pow(2.0, -2.0/12.0);
      } else if (notes[c] == '0') {
	this_f *= pow(2.0,-4);
      } else if (notes[c] == '1') {
	this_f *= pow(2.0,-3);
      } else if (notes[c] == '2') {
	this_f *= pow(2.0,-2);
      } else if (notes[c] == '3') {
	this_f *= pow(2.0,-1);
      } else if (notes[c] == '4') {
      } else if (notes[c] == '5') {
	this_f *= 2;
      } else if (notes[c] == '6') {
	this_f *= 4;
      } else if (notes[c] == '7') {
	this_f *= 8;
      } else if (notes[c] == '8') {
	this_f *= 16;
      } else if (notes[c] == '9') {
	this_f *= 32;
      } else if (notes[c] == '#') {
	this_f *= pow(2.0, 1.0/12.0);
      } else if (notes[c] == 'p') {
	this_f *= pow(2.0, -1.0/12.0);
      } else if (notes[c] == 'h') {
	this_w *= 0.5;
      } else if (notes[c] == 'H') {
	this_w *= 2;
      } else if (notes[c] == 't') {
	this_w *= 1.0/3.0;
      } else if (notes[c] == 'T') {
	this_w *= 3.0;
      } else {
      }
      parsed++;
      c++;
    }

    if (parsed > 0) {
      note_fs.push_back(this_f);
      note_ws.push_back(this_w);
      cout << "pushing f w: " << this_f << " " << this_w << " finished" << endl;
    } else {
      cout << "did not parse non-whitespace, not pushing" << endl;
    }
  }

  if (note_fs.size() == 0) {
    double this_f = f;
    note_fs.push_back(this_f);
    cout << "pushing " << this_f << " by default" << endl;
  } else {}


  int tone_samples_start = max( 0, min( int(ceil(16000*s)), ms->pack[this_dog].voice_buffer_size-1 ) );
  int tone_samples_end = max( 0, min( int(ceil(16000*e)), ms->pack[this_dog].voice_buffer_size-1 ) );


  int current_note = 0;
  int num_notes = note_fs.size();
  int cn_sample_offset = 0;
  int played_samples = 0;

  for (int i = tone_samples_start; i <= tone_samples_end; i++) {
    int freq_idx = current_note;
    double this_f = note_fs[ freq_idx ];
    double this_w = note_ws[ freq_idx ];

    int warb_width_samples = ceil(this_w * 16000);
    int advance_samples = a * double(warb_width_samples);

    int advanced_i = (i + advance_samples);
    int sample_idx = advanced_i - cn_sample_offset;

    //int sample_arg = min(warb_width_samples-1-sample_idx, sample_idx);
    //int sample_arg = sample_idx;
    //int16_t intended = int16_t(  v * sin(advanced_i * f * 2.0 * M_PI / 16000) * exp( -pow(sample_arg,2)/(2.0*pow(sigma,2.0)) )  );


//cout << "this_f synth " << this_f << endl;
    ms->pack[this_dog].voice_buffer[i] += v * squareWave(i * this_f * 2.0 * M_PI / 16000) * exp( -pow((warb_width_samples/2.0)-sample_idx,2)/(2.0*pow(sigma,2.0)) );
    //ms->pack[this_dog].voice_buffer[i] += v * sin(i * this_f * 2.0 * M_PI / 16000) * exp( -pow(sample_arg,2)/(2.0*pow(sigma,2.0)) );
  
    played_samples++;
    if (played_samples - cn_sample_offset >= warb_width_samples) {
      current_note++;
      cn_sample_offset = cn_sample_offset + warb_width_samples;
      cout << advance_samples << " adv samp, " << played_samples << " " << current_note << endl;
    } else {}
  
    current_note = current_note % num_notes;
  }
}
END_WORD
REGISTER_WORD(DogVoiceTimeTrackWarbleSquareNotes)

WORD(DogVoiceTetraTrackWarbleNotes)
virtual void execute(std::shared_ptr<MachineState> ms) {
// volume a4_frequency start end window_sigma repetition_period advance_fraction "A3# B2 C4b R R" dogVoiceTrackWarbleNotes
  int this_dog = ms->focusedMember;

  string notes;
  GET_STRING_ARG(ms, notes);

  string wave;
  GET_STRING_ARG(ms, wave);

  // XXX set wave
  double (*waveFunction)(double) = &sin;
  if (wave.compare("sin") == 0) {
    waveFunction = &sin;
  } else if (wave.compare("square") == 0) {
    waveFunction = &squareWave;
  } else if (wave.compare("triangle") == 0) {
    waveFunction = &triangleWave;
  } else if (wave.compare("sawtooth") == 0) {
    waveFunction = &sawtoothWave;
  } else {
    cout << "Unrecognized wave, using sin" << endl;
    waveFunction = &sin;
  }

  double a = 0.0;
  GET_NUMERIC_ARG(ms, a);

  double warb_width_seconds = 1.0;
  GET_NUMERIC_ARG(ms, warb_width_seconds);

  double sigma = 1.0;
  GET_NUMERIC_ARG(ms, sigma);

  double e = 0.0;
  GET_NUMERIC_ARG(ms, e);

  double s = 0.0;
  GET_NUMERIC_ARG(ms, s);

  double f = 440.0;
  GET_NUMERIC_ARG(ms, f);

  double v = 0.0;
  GET_NUMERIC_ARG(ms, v);

  vector<double> note_fs;
  vector<double> note_ws;
  vector<double> note_vs;
  for (int c = 0; c < notes.size(); c++) {
    double this_f = f;
    double this_w = warb_width_seconds;
    double this_v = v;

    int parsed = 0;

    while ( (c < notes.size()) ) {
      cout << notes[c] << " note " << endl;
      if (notes[c] == ' ') {
	cout << "breaking " << this_f << " hit space" << endl;
	break;
      } else if (notes[c] == 'R' || notes[c] == 'r') {
	this_f *= 0;
	this_v *= 0;
      } else if (notes[c] == 'a' || notes[c] == 'A') {
      } else if (notes[c] == 'b' || notes[c] == 'B') {
	this_f *= pow(2.0, 2.0/12.0);
      } else if (notes[c] == 'c' || notes[c] == 'C') {
	this_f *= pow(2.0, -9.0/12.0);
      } else if (notes[c] == 'd' || notes[c] == 'D') {
	this_f *= pow(2.0, -7/12.0);
      } else if (notes[c] == 'e' || notes[c] == 'E') {
	this_f *= pow(2.0, -5.0/12.0);
      } else if (notes[c] == 'f' || notes[c] == 'F') {
	this_f *= pow(2.0, -4.0/12.0);
      } else if (notes[c] == 'g' || notes[c] == 'G') {
	this_f *= pow(2.0, -2.0/12.0);
      } else if (notes[c] == '0') {
	this_f *= pow(2.0,-4);
      } else if (notes[c] == '1') {
	this_f *= pow(2.0,-3);
      } else if (notes[c] == '2') {
	this_f *= pow(2.0,-2);
      } else if (notes[c] == '3') {
	this_f *= pow(2.0,-1);
      } else if (notes[c] == '4') {
      } else if (notes[c] == '5') {
	this_f *= 2;
      } else if (notes[c] == '6') {
	this_f *= 4;
      } else if (notes[c] == '7') {
	this_f *= 8;
      } else if (notes[c] == '8') {
	this_f *= 16;
      } else if (notes[c] == '9') {
	this_f *= 32;
      } else if (notes[c] == '#') {
	this_f *= pow(2.0, 1.0/12.0);
      } else if (notes[c] == 'p') {
	this_f *= pow(2.0, -1.0/12.0);
      } else if (notes[c] == 'h') {
	this_w *= 0.5;
      } else if (notes[c] == 'H') {
	this_w *= 2;
      } else if (notes[c] == 't') {
	this_w *= 1.0/3.0;
      } else if (notes[c] == 'T') {
	this_w *= 3.0;
      } else {
      }
      parsed++;
      c++;
    }

    if (parsed > 0) {
      note_fs.push_back(this_f);
      note_ws.push_back(this_w);
      note_vs.push_back(this_v);
      cout << "pushing f w: " << this_f << " " << this_w << " finished" << endl;
    } else {
      cout << "did not parse non-whitespace, not pushing" << endl;
    }
  }

  if (note_fs.size() == 0) {
    double this_f = f;
    double this_w = warb_width_seconds;
    double this_v = v;
    note_fs.push_back(this_f);
    note_ws.push_back(this_w);
    note_vs.push_back(this_v);
    cout << "pushing " << this_f << " by default" << endl;
  } else {}


  int tone_samples_start = max( 0, min( int(ceil(16000*s)), ms->pack[this_dog].voice_buffer_size-1 ) );
  int tone_samples_end = max( 0, min( int(ceil(16000*e)), ms->pack[this_dog].voice_buffer_size-1 ) );


  int current_note = 0;
  int num_notes = note_fs.size();
  int cn_sample_offset = 0;
  int played_samples = 0;

  for (int i = tone_samples_start; i <= tone_samples_end; i++) {
    int freq_idx = current_note;
    double this_f = note_fs[ freq_idx ];
    double this_w = note_ws[ freq_idx ];
    double this_v = note_vs[ freq_idx ];

    int warb_width_samples = ceil(this_w * 16000);
    int advance_samples = a * double(warb_width_samples);

    int advanced_i = (i + advance_samples);
    int sample_idx = advanced_i - cn_sample_offset;

    //ms->pack[this_dog].voice_buffer[i] += v * squareWave(i * this_f * 2.0 * M_PI / 16000) * exp( -pow((warb_width_samples/2.0)-sample_idx,2)/(2.0*pow(sigma,2.0)) );
    ms->pack[this_dog].voice_buffer[i] += this_v * (*waveFunction)(i * this_f * 2.0 * M_PI / 16000) * exp( -pow((warb_width_samples/2.0)-sample_idx,2)/(2.0*pow(sigma,2.0)) );
  
    played_samples++;
    if (played_samples - cn_sample_offset >= warb_width_samples) {
      current_note++;
      cn_sample_offset = cn_sample_offset + warb_width_samples;
      cout << advance_samples << " adv samp, " << played_samples << " " << current_note << endl;
    } else {}
  
    current_note = current_note % num_notes;
  }
}
END_WORD
REGISTER_WORD(DogVoiceTetraTrackWarbleNotes)

WORD(DogVoiceSing)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;
  int tone_samples = ms->pack[this_dog].voice_buffer_size;
  int tone_length = 2*tone_samples;

  stringstream ss;
  ss << "speaker.val = BIN " << tone_length << " raw 1 16000 16 1;";
  string header = ss.str();

  int buf_size = tone_length + header.size();
  char * buf = new char[buf_size];
  sprintf(buf, "%s", header.c_str());

  int16_t * currentSample = (int16_t*)(buf + header.size());

  for (int i = 0; i < ms->pack[this_dog].voice_buffer_size; i++) {
    // clip
    currentSample[i] = int16_t( max( double(INT16_MIN), min( ms->pack[this_dog].voice_buffer[i], double(INT16_MAX) ) ) );
    //cout << int(currentSample[i]) << " " << intended << endl;;
  }

  sendOnDogSocket(ms, this_dog, buf, buf_size);

  delete buf;
}
END_WORD
REGISTER_WORD(DogVoiceSing)

void write_little_endian(unsigned int word, int num_bytes, uchar *out) {
  if (num_bytes > 0) {
    uchar buf;
    while( num_bytes > 0 ) {   
      buf = word & 0xff;
      *out = buf;
      out++;
      num_bytes--;
      word >>= 8;
    }
  } else {}
}

WORD(DogVoiceToPCM)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;
  int tone_samples = ms->pack[this_dog].voice_buffer_size;
  int tone_length = 2*tone_samples;

  string filename;
  GET_STRING_ARG(ms, filename);

  stringstream ss;
  //ss << "speaker.val = BIN " << tone_length << " raw 1 16000 16 1;";
  // 'RIFF'
  // 4:(overall size - 8 bytes):
  // 'WAVE'
  // 'fmt\0'
  // 4:length of above header:16 
  // 2:format type, PCM=1:1 
  // 2:channels:1 
  // 4:rate:16000 
  // 4:(sample rate * bits per sample * number of channels) / 8:
  // 2:(bits per sample * channel)/8:2 
  // 2:bits per sample:16   
  // 'data' 
  // 4:size of data:
  ss << "RIFF    WAVEfmt                     data    ";
  string header = ss.str();

  int buf_size = max(int(tone_length + header.size()), int(1 + header.size()));
  uchar * buf = new uchar[buf_size];
  sprintf((char*)buf, "%s", header.c_str());

  /*
  *(int*)(&buf[4]) = buf_size - 8;
  *(int*)(&buf[16]) = 16;
  *(uchar*)(&buf[20]) = 1; *(uchar*)(&buf[21]) = 0;
  *(uchar*)(&buf[22]) = 1; *(uchar*)(&buf[23]) = 0;
  *(int*)(&buf[24]) = 16000;
  *(int*)(&buf[28]) = 32000;
  *(uchar*)(&buf[33]) = 2; *(uchar*)(&buf[34]) = 0;
  *(uchar*)(&buf[35]) = 16; *(uchar*)(&buf[36]) = 0;
  *(int*)(&buf[40]) = tone_length;
  */

  write_little_endian(buf_size-8, 4, &(buf[4])); 
  write_little_endian(16, 4, &(buf[16])); 
  write_little_endian(1, 2, &(buf[20])); 
  write_little_endian(1, 2, &(buf[22])); 
  write_little_endian(16000, 4, &(buf[24])); 
  write_little_endian(32000, 4, &(buf[28])); 
  write_little_endian(2, 2, &(buf[32])); 
  write_little_endian(16, 2, &(buf[34])); 
  write_little_endian(tone_length, 4, &(buf[40])); 

  int16_t * currentSample = (int16_t*)(buf + header.size());

  for (int i = 0; i < ms->pack[this_dog].voice_buffer_size; i++) {
    // clip
    //currentSample[i] = int16_t( max( double(INT16_MIN), min( ms->pack[this_dog].voice_buffer[i], double(INT16_MAX) ) ) );
    int16_t val = int16_t( max( double(INT16_MIN), min( ms->pack[this_dog].voice_buffer[i], double(INT16_MAX) ) ) );
    write_little_endian(val, 2, (uchar*)&(currentSample[i]));
    //cout << int(currentSample[i]) << " " << intended << endl;;
  }

  //sendOnDogSocket(ms, this_dog, buf, buf_size);
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/wav/";
  mkdir(ss_dir.str().c_str(), 0777);

  stringstream ssf;
  ssf << ms->config.data_directory + "/wav/" + filename + ".wav";

  FILE *out = fopen(ssf.str().c_str(), "wb");
  fwrite(buf, 1, buf_size, out);
  fclose(out);

  delete buf;
}
END_WORD
REGISTER_WORD(DogVoiceToPCM)


WORD(DogWriteIntendedFromTrue)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int this_dog = ms->focusedMember;
  EinAiboConfig & dog = ms->pack[this_dog];

  dog.intendedPose = dog.truePose;
  memcpy(dog.intendedGain, dog.trueGain, 3 * sizeof(EinAiboJoints));
  dog.intendedIndicators = dog.trueIndicators;
}
END_WORD
REGISTER_WORD(DogWriteIntendedFromTrue)



/*

WORD(DogVoice)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(DogVoice)

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
 

  // convert dog image 
  // loop over relevant pixels
  // fill out gaussian map

  // convert dog image
  // loop over relevant pixels
  // lay down density at parts probably not background





}
