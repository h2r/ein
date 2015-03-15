#!/usr/bin/env python
import rospy
import std_msgs
import roslib
#roslib.load_manifest("baxter_pick_and_place")
import readline

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
    def __init__(self, words, topic):
        print "topic: ", topic

        self.forth_command_publisher = rospy.Publisher(topic, 
                                                       std_msgs.msg.String, queue_size=10)
        
        readline.set_completer(SimpleCompleter(words).complete)
        save_history_hook()


    def ask(self):

        while True:
            line = raw_input('Prompt ("stop" to quit): ')
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

    rospy.init_node("ein_client_%s" % arm)
    words = []
    for wordline in open("ein_words.txt"):
        words.append(wordline.split(" ")[0])
        
    print words


    client = EinClient(words, "/ein/%s/forth_commands" % arm)

    client.ask()


    
if __name__=='__main__':
    main()
