#!/usr/bin/env python
import rospy
import math
from ein.msg import EinState
import os
class EinPrint:
    def __init__(self, topic):

        self.state_subscriber = rospy.Subscriber(topic, 
                                                 EinState, self.state_callback)
        self.state = None
        self.call_stack = []
        self.data_stack = []
        

    def state_callback(self, msg):
        state = str(msg.state_string)

        rows, cols = os.popen('stty size', 'r').read().split()
        rows = int(rows)
        cols = int(cols)

        state_lines = state.split("\n")
        num_lines = sum([max(1, int(math.ceil(len(w) / float(cols)))) for w in state_lines])
        #print "num_lines", num_lines
        lines_to_add = max(0, rows - num_lines - 1)
        state +="".rjust(lines_to_add, "\n")
        #print "num_lines: ", num_lines, "rows", rows, "lines to add", lines_to_add
        print state

    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.2)


def main():
    import sys
    if (len(sys.argv) != 2):
        print "usage:  ein_print_state.py left|right"
        return

    arm = sys.argv[1]

    rospy.init_node("ein_print_state_%s" % arm, anonymous=True)


    client = EinPrint("/ein/%s/state" % arm)
    client.spin()


    
if __name__=='__main__':
    main()
