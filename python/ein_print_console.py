#!/usr/bin/env python

import rclpy
from rclpy.node import Node


from ein.msg import EinConsole
import os
class EinPrint(Node):
    def __init__(self, topic):
        super().__init__('EinPrint')
        print("topic: " + topic)
        self.state_subscriber = self.create_subscription(EinConsole, topic, 
                                                         self.einconsole_callback, 10)

    def einconsole_callback(self, msg):
        print("got msg")
        print(str(msg.msg))


def hangup(signal, stackframe):
    import signal
    import os
    os.kill(os.getpid(), signal.SIGTERM)


def main():
    import sys
    import signal

    if (len(sys.argv) != 1):
        print("usage:  ein_print_console.py")
        return


    #signal.signal(signal.SIGHUP, hangup)
    rclpy.init()

    rows, cols = os.popen('stty size', 'r').read().split()
    rows = int(rows)
    print("".rjust(rows, "\n"))

    client = EinPrint("/ein/left/console")
    print("spinning")
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


    
if __name__=='__main__':
    main()
