#!/usr/bin/env python
"""
Simple example showing how to use the SoundClient provided by libsoundplay,
in blocking, non-blocking, and explicit usage.
"""

import rospy
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
from std_msgs.msg import String

def callback(data):
    soundhandle = SoundClient(blocking=True)
    soundhandle.say(data.data)

if __name__ == '__main__':
    rospy.init_node('ros_soundclient', anonymous=False)
    rospy.Subscriber("/talk", String, callback)
    rospy.spin()
