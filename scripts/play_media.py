#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class MediaPlayer:
    def __init__(self):
        self.play_media_sub = rospy.Subscriber('play/media',
                                               String,
                                               self.play_media_cb)
        
        self.type_media = ''
        self.media = ''

    def play_media_cb(self, data):
        self.type_media = data.data.split('%')[0]
        self.media = data.data.split('%')[1]
        if(self.type_media == 'M' or self.type_media == 'AL'):
            print("Ripriduco audio:", self.media)
        elif(self.type_media == 'V'):
            print("Ripriduco video:", self.media)

if __name__ == "__main__":
    rospy.init_node('media_player')
    mp = MediaPlayer()
    rate = rospy.Rate(10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')
            