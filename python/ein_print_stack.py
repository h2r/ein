#!/usr/bin/env python
import rospy

from ein.msg import EinState
import os


class EinStack:
    def __init__(self, state_topic, stack_to_use):
        print "state topic: ", state_topic

        self.state_subscriber = rospy.Subscriber(state_topic, 
                                                 EinState, self.state_callback)
        self.state = None
        self.call_stack = []
        self.data_stack = []
        self.stack_to_use = stack_to_use

    def state_callback(self, msg):
        #print "received state."
        self.state = msg
        self.call_stack = self.state.call_stack
        self.data_stack = self.state.data_stack

    def printStack(self, stack):
        rows, cols = os.popen('stty size', 'r').read().split()
        rows = int(rows)
        print "rows", rows
        stack_to_print = stack[0:rows]
        string = ""
        if len(stack_to_print) != rows:
            string += "".rjust(rows - len(stack_to_print), "\n")
        for i, word in enumerate(stack_to_print):
            string += "%s: %s\n" % (str(len(stack_to_print) - i).rjust(3), word)
        print string
    def printCallStack(self):
        self.printStack(self.call_stack)
    def printDataStack(self):
        self.printStack(self.data_stack)

    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            if self.stack_to_use == "call":
                self.printCallStack()
            elif self.stack_to_use == "data":
                self.printDataStack()
            else:
                raise ValueError("Bad stack: %s" % self.stack_to_use)
def hangup(signal, stackframe):
    import signal
    import os
    os.kill(os.getpid(), signal.SIGTERM)

def main():
    import sys
    import signal
    if (len(sys.argv) != 3):
        print "usage:  ein_stack.py left|right  call|data"
        return

    arm = sys.argv[1]
    stack = sys.argv[2]

    if arm not in ("left", "right"):
        print "Arm must be left or right, not", arm
        return
    if stack not in ("call", "data"):
        print "Stack must be call or data, not", stack
        return

    rospy.init_node("ein_stack_%s" % arm, anonymous=True)

    signal.signal(signal.SIGHUP, hangup)
    client = EinStack("/ein/%s/state" % arm, stack)
    client.spin()


    
if __name__=='__main__':
    main()
