#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import math
from ein.msg import EinState
import os
class EinPrint(Node):

    def __init__(self, topic):
        super().__init__('EinPrint')
        self.state_subscriber = self.create_subscription(EinState, topic, 
                                                         self.state_callback, 10)
        self.state = None
        self.call_stack = []
        self.data_stack = []
        self.lastrun = self.get_clock().now()

    def state_callback(self, msg):
        if self.get_clock().now() - self.lastrun < rclpy.duration.Duration(seconds=0.25):
            return

        self.lastrun = self.get_clock().now()
            
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
        print(state)


def hangoup(signal, stackframe):
    import signal
    import os
    os.kill(os.getpid(), signal.SIGTERM)

def main():
    import sys
    import signal
    if (len(sys.argv) != 1):
        print("usage:  ein_print_state.py")
        return

    #signal.signal(signal.SIGHUP, hangup)

    rclpy.init()


    client = EinPrint("/ein/left/state")
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()

    
if __name__=='__main__':
    main()
