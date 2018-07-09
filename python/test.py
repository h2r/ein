#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
import socket
import sys

HOST = ''
PORT = 12346
SIZE = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((HOST, PORT))

def talker():
    pub = rospy.Publisher('/ein/right/forth_commands', String, queue_size=0)
    rospy.init_node('commands', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    start_str = "\"aibo\" import"
    rospy.loginfo(start_str)
    pub.publish(start_str)
    time.sleep(3)
    summon_str = "dogSummonPluto"
    rospy.loginfo(summon_str)
    pub.publish(summon_str)
    time.sleep(3)
    while True:
        (data, addr) = s.recvfrom(SIZE)
        print(data)
        #sys.exit()
        if not rospy.is_shutdown():
            execute_str = data
            rospy.loginfo(execute_str)
            pub.publish(execute_str)
            rate.sleep()
            time.sleep(2.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

