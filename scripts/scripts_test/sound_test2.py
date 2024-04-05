#!/usr/bin/env python


''' This script is a template to develop your own node implementing a conversational agent. One should override listen and talk methods to define custom behavior.
'''
from class_SAY import SAY
#from sound_play.msg import SoundRequest
#from sound_play.libsoundplay import SoundClient

import time
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray

class ChatBot():
    def __init__(self):
        self.state = "idle" #DEFINE all the states 

        # Initialize the OpenAI model to generate responses 
        self.r_sound = SAY()

        # self.h_req_sub = rospy.Subscriber('r_response',
        #                                 String,
        #                                 self.h_listen
        #                                 )

        # listen to the user input
        # When the user add a request it triggers the model to response to the user input
        # It has to suscribe to the topic used by ros_whisper
        self.r_req_sub = rospy.Subscriber('asr/string',
                                        String,
                                        self.r_talk
                                        )
        #self.s = rospy.Subscriber('/sound_play/status', GoalStatusArray)

        # publisher relative to the robot
        # self.r_res_pub = rospy.Publisher('r_response',
        #                                 String,
        #                                 queue_size=1
        #                                 )
        
    def idle(self):
        self.state = "talk"
        self.r_talk(self.state) 
    
    # Once the model gets an user input, it provides the response
    def r_talk(self, r):
        rospy.init_node('chatbot')
        rate = rospy.Rate(0.1)
        r_ans = "Ciao, come stai? Io sto molto bene, oggi voglio andare al parco" #Use this line to test the system without wasting tokens
        while not rospy.is_shutdown():
            self.r_sound.r_say(r_ans)
            rate.sleep()
            #rospy.spin()

if __name__ == "__main__":
    cb = ChatBot()
    try:
        r = "r"
        cb.r_talk(r)
    except KeyboardInterrupt:
        pass
            