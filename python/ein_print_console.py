#!/usr/bin/env python
import rospy

from ein.msg import EinConsole
import os
class EinPrint:
    def __init__(self, topic):

        self.state_subscriber = rospy.Subscriber(topic, 
                                                 EinConsole, self.einconsole_callback)
        self.state = None
        self.call_stack = []
        self.data_stack = []
        

    def einconsole_callback(self, msg):
        print str(msg.msg)

    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.2)

def hangup(signal, stackframe):
    import signal
    import os
    os.kill(os.getpid(), signal.SIGTERM)


def main():
    import sys
    import signal

    if (len(sys.argv) != 2):
        print "usage:  ein_print_console.py left|right"
        return

    arm = sys.argv[1]
    signal.signal(signal.SIGHUP, hangup)
    rospy.init_node("ein_print_console_%s" % arm, anonymous=True)

    rows, cols = os.popen('stty size', 'r').read().split()
    rows = int(rows)
    print "".rjust(rows, "\n")

    client = EinPrint("/ein/%s/console" % arm)
    client.spin()


    
if __name__=='__main__':
    main()
