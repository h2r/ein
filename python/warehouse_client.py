#!/usr/bin/env python
import rospy
import time
import std_msgs
import roslib
#roslib.load_manifest("ein")
from object_recognition_msgs.msg import RecognizedObjectArray
import baxter_external_devices

class WarehouseClient:
    def __init__(self, givenTopic):
        self.command_publisher = rospy.Publisher("/fetch_commands", 
                                                 std_msgs.msg.String, queue_size=10)
        self.objects = []
        #topic = "/publish_detections_center/blue_labeled_objects" # node
        #topic = "/ar_objects" # ar tags
        #topic = "/ein_right/blue_memory_objects" # ein
        #topic = "/ein_left/blue_memory_objects" # ein
	topic = givenTopic
	print topic
        rospy.Subscriber(topic, RecognizedObjectArray, self.object_callback)
        self.last_object_callback = rospy.Time()
        rospy.Timer(rospy.Duration(2), self.timer)
        
    def timer(self, event):
        if (self.last_object_callback  - rospy.Time.now() > rospy.Duration(10)):
            self.objects = []

    def object_callback(self, msg):
        self.objects = []
        self.object_poses = dict()
        for o in msg.objects:
            self.objects.append(o.type.key)
        self.objects.sort()
        self.objects = tuple(self.objects)
        self.last_object_callback = rospy.Time()

    def ask(self):
        done = False
        printed_objects = None

        while not done and not rospy.is_shutdown():
            if printed_objects != self.objects:
                printed_objects = self.objects
                print "Which object?"
                for i, o in enumerate(self.objects):
                    print "%d.)" % (i + 1), o

            c = baxter_external_devices.getch()
            if c:
                if c in ['\x1b', '\x03']:
                    done = True
                else:
                    try:
                        idx = int(c)
                    except ValueError:
                        idx = 0
                    if idx != None and idx > 0 and idx <= len(self.objects):
                        print "Sending", self.objects[idx - 1]
                        self.command_publisher.publish(self.objects[idx - 1])
def main():
    import sys
    if (len(sys.argv) != 2):
        print "usage:  warehouse_client.py left|right"
        return
    arm = sys.argv[1]

    rospy.init_node("warehouse_client_%s" % arm)
    client = WarehouseClient("/ein_%s/blue_memory_objects" % arm)

    client.ask()


    
if __name__=='__main__':
    main()
