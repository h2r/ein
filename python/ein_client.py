#!/usr/bin/env python
import rospy

import std_msgs
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


class EinClient:
    def __init__(self, words, base_topic, print_console_messages, print_stacks):
        print "base topic: ", base_topic

        self.print_console_messages = print_console_messages
        self.print_stacks = print_stacks
        self.forth_command_publisher = rospy.Publisher("%s/forth_commands" % base_topic, 
                                                       std_msgs.msg.String, queue_size=10)

        self.state_subscriber = rospy.Subscriber("%s/state" % base_topic, 
                                                 EinState, self.state_callback)
        self.console_subscriber = rospy.Subscriber("%s/console" % base_topic, 
                                                   EinConsole, self.einconsole_callback)

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
        print "Call Stack: "
        for word in reversed(self.call_stack):
            print " ".rjust(15), word

    def printDataStack(self):
        print "Data Stack: "
        for word in reversed(self.data_stack):
            print " ".rjust(15), word


    def ask(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            if self.print_console_messages:
                print "Console: "
                print "\t" + "\n\t".join(self.console_messages)
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

    parser.add_argument('arm', nargs=1, choices=("left", "right"),
                        help='Which arm to use.')

    args = parser.parse_args()
    arm = args.arm[0]



    rospy.init_node("ein_client_%s" % arm, anonymous=True)
    words = []
    for wordline in open("ein_words.txt"):
        words.append(wordline.split(" ")[0])
        
    #print words


    client = EinClient(words, "/ein/%s" % arm, not args.silence_console_messages, not args.silence_stacks)
    rows, cols = os.popen('stty size', 'r').read().split()
    rows = int(rows)
    print "".rjust(rows, "\n")

    signal.signal(signal.SIGHUP, hangup)
    client.ask()


    
if __name__=='__main__':
    main()
