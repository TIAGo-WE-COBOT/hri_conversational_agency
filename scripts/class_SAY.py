#!/usr/bin/env python

from say_cfg import VOICE, VOLUME
from sound_play.libsoundplay import SoundClient

import rospy
from std_msgs.msg import Bool, String

class SAY():
    def __init__(self):
        self.soundhandle = SoundClient(blocking=True)
        self.voice = VOICE
        self.volume = VOLUME
        #self.is_playing = Bool()
        self.is_playing = String()
        self.is_playing.data = "False"
        #self.play_pb = rospy.Publisher("check/play", Bool, queue_size=1)
        self.play_pb = rospy.Publisher("check/play", String, queue_size=1)

    def r_say(self, string):
        self.is_playing.data = "True"
        self.play_pb.publish(self.is_playing)
        print(self.is_playing)
        self.soundhandle.say(string, self.voice, self.volume)
        self.is_playing.data = "False"
        self.play_pb.publish(self.is_playing)
        print(self.is_playing)
