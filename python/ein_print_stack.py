#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from ein.msg import EinState
import os


class EinStack(Node):
    def __init__(self, state_topic, stack_to_use):
        super().__init__('EinStack')
        print("state topic: " + state_topic)

        self.state_subscriber = self.create_subscription(EinState,
                                                         state_topic,
                                                         self.state_callback, 10)
        self.state = None
        self.call_stack = []
        self.data_stack = []
        self.stack_to_use = stack_to_use

        self.timer = self.create_timer(0.2, self.printCallback)
        

    def state_callback(self, msg):
        self.state = msg
        self.call_stack = self.state.call_stack
        self.data_stack = self.state.data_stack

    def printStack(self, stack):
        rows, cols = os.popen('stty size', 'r').read().split()
        rows = int(rows)
        print("rows " + str(rows))
        stack_to_print = stack[0:rows]
        string = ""
        if len(stack_to_print) != rows:
            string += "".rjust(rows - len(stack_to_print), "\n")
        for i, word in enumerate(stack_to_print):
            string += "%s: %s\n" % (str(len(stack_to_print) - i).rjust(3), word)
        print(string)
    def printCallStack(self):
        self.printStack(self.call_stack)
    def printDataStack(self):
        self.printStack(self.data_stack)

    def printCallback(self):
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
    if (len(sys.argv) != 2):
        print("usage:  ein_stack.py call|data")
        return

    stack = sys.argv[1]

    if stack not in ("call", "data"):
        print("Stack must be call or data, not " + stack)
        return

    rclpy.init()

    #signal.signal(signal.SIGHUP, hangup)
    client = EinStack("/ein/left/state", stack)
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()

    
if __name__=='__main__':
    main()
