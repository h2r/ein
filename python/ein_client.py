#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import threading

from std_msgs.msg import String
import readline

from ein.msg import EinState
from ein.msg import EinConsole

import os

readline.parse_and_bind('tab: complete')
readline.parse_and_bind('set editing-mode emacs')



class SimpleCompleter(object):
    
    def __init__(self, options):
        self.options = sorted(options)
        return

    def complete(self, text, state):
        response = None
        if state == 0:
            # This is the first time for this text, so build a match list.
            if text:
                self.matches = [s 
                                for s in self.options
                                if s and s.startswith(text)]
            else:
                self.matches = self.options[:]
        
        # Return the state'th item from the match list,
        # if we have that many.
        try:
            response = self.matches[state]
        except IndexError:
            response = None
        return response


class EinClient(Node):
    def __init__(self, words, base_topic, print_console_messages, print_stacks):
        super().__init__('EinClient')

        print("base topic: ", base_topic)

        self.print_console_messages = print_console_messages
        self.print_stacks = print_stacks
        self.forth_command_publisher = self.create_publisher(String, "%s/forth_commands" % base_topic, 
                                                             10)

        self.state_subscriber = self.create_subscription(EinState, "%s/state" % base_topic, 
                                                         self.state_callback, 10)
        self.console_subscriber = self.create_subscription(EinConsole, "%s/console" % base_topic, 
                                                           self.einconsole_callback, 10)

        self.state = None
        self.call_stack = []
        self.data_stack = []
        self.console_messages = []
        
        save_history_hook()
        
    def einconsole_callback(self, msg):
        self.console_messages.append(msg.msg)
        self.console_messages = self.console_messages[-10:]

    def state_callback(self, msg):
        #print "received state."
        self.state = msg
        self.call_stack = self.state.call_stack
        self.data_stack = self.state.data_stack
        if len(msg.words) != 0:
            readline.set_completer(SimpleCompleter(msg.words).complete)



    def printCallStack(self):
        print("Call Stack: ")
        for word in reversed(self.call_stack):
            print(" ".rjust(15) + " " + word)

    def printDataStack(self):
        print("Data Stack: ")
        for word in reversed(self.data_stack):
            print(" ".rjust(15) + word)


    def ask(self):

        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()
        rate = self.create_rate(0.2)
        
        while rclpy.ok():
            rate.sleep()
            if self.print_console_messages:
                print("Console: ")
                print("\t" + "\n\t".join(self.console_messages))
            if self.print_stacks:
                self.printCallStack()
                self.printDataStack()
            try:
                line = raw_input('Prompt: ')
            except EOFError:
                break
    
            if line == 'stop':
                break
            #print 'ENTERED: "%s"' % line
            self.forth_command_publisher.publish(line);
            rclpy.spinOnce()

def save_history_hook():
    import os
    histfile = os.path.join(os.path.expanduser("~"), ".ein_client_history")
    try:
        readline.read_history_file(histfile)
    except IOError:
        pass
    import atexit
    atexit.register(readline.write_history_file, histfile)

def hangup(signal, stackframe):
    import signal
    import os
    os.kill(os.getpid(), signal.SIGTERM)



def main():
    import signal

    import argparse

    parser = argparse.ArgumentParser(description='Run the ein client.')
    parser.add_argument('--silence-console-messages', action='store_true',
                        help='Whether we should silence console messages.')
    parser.add_argument('--silence-stacks', action='store_true',
                        help='Whether we should print the stacks.')

    args = parser.parse_args()



    rclpy.init()
    #words = []
    #for wordline in open("ein_words.txt"):
    #    words.append(wordline.split(" ")[0])
        
    #print words


    client = EinClient([], "/ein/left", not args.silence_console_messages, not args.silence_stacks)
    rows, cols = os.popen('stty size', 'r').read().split()
    rows = int(rows)
    print("".rjust(rows, "\n"))

    signal.signal(signal.SIGHUP, hangup)
    client.ask()


    
if __name__=='__main__':
    main()
