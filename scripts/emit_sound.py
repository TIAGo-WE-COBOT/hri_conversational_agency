#!/usr/bin/env python

VOICE = 'voice_pc_diphone'
VOLUME = 1

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class EmitSound():
    def __init__(self):
        self.soundhandle = SoundClient(blocking=True) #In this way the flux is stopped untill the sound in completely reproduced
        self.voice = VOICE
        self.volume = VOLUME
        self.is_playing = String()
        self.is_playing.data = "False"
        self.play_pb = rospy.Publisher("check/play", String, queue_size=1)

    def r_say(self, string):
        self.is_playing.data = "True"
        self.play_pb.publish(self.is_playing)
        print("The robot is talking...")
        self.soundhandle.say(string, self.voice, self.volume)
        self.is_playing.data = "False"
        self.play_pb.publish(self.is_playing)
        print("The robot ceased speaking.\n")
