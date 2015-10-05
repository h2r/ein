#!/usr/bin/env python
import rospy
import std_msgs
import roslib
#roslib.load_manifest("baxter_pick_and_place")
import readline

from ein.msg import EinState

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
    def __init__(self, words, publish_topic, state_topic):
        print "publish topic: ", publish_topic
        print "state topic: ", state_topic

        self.forth_command_publisher = rospy.Publisher(publish_topic, 
                                                       std_msgs.msg.String, queue_size=10)

        self.state_subscriber = rospy.Subscriber(state_topic, 
                                                 EinState, self.state_callback)
        self.state = None
        self.call_stack = []
        self.data_stack = []
        
        save_history_hook()

    def state_callback(self, msg):
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

        while True:
            rospy.sleep(0.2)
            self.printCallStack()
            self.printDataStack()
            try:
                line = raw_input('Prompt ("stop" to quit): ')
            except EOFError:
                break
    
            if line == 'stop':
                break
            print 'ENTERED: "%s"' % line
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

def main():
    import sys
    if (len(sys.argv) != 2):
        print "usage:  ein_client.py left|right"
        return

    arm = sys.argv[1]

    rospy.init_node("ein_client_%s" % arm, anonymous=True)
    words = []
    for wordline in open("ein_words.txt"):
        words.append(wordline.split(" ")[0])
        
    print words


    client = EinClient(words, 
                       "/ein/%s/forth_commands" % arm,
                       "/ein_%s/state" % arm,
                       )

    client.ask()


    
if __name__=='__main__':
    main()
