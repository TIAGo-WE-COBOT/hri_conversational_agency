#!/usr/bin/env python

''' This script is a template to develop your own node implementing a conversational agent. One should override listen and talk methods to define custom behavior.
'''
from class_SAY import SAY
from conversational_agency.openai_utils.chatter import OpenAIChatter
#from sound_play.msg import SoundRequest
#from sound_play.libsoundplay import SoundClient

import time
import rospy
from std_msgs.msg import String

class ChatBot():
    def __init__(self):

        # Initialize the OpenAI model to generate responses 
        self.ai_chatter = OpenAIChatter()
        self.r_sound = SAY()

        # Listen to the model response
        # When the model provides a response it triggers the method to let the user add a request
        self.h_req_sub = rospy.Subscriber('r_response',
                                        String,
                                        self.h_listen
                                        )
        
        # publisher relative to the user
        self.h_res_pub = rospy.Publisher('h_response',
                                       String,
                                       queue_size=1
                                       )
        # listen to the user input
        # When the user add a request it triggers the model to response to the user input
        self.r_req_sub = rospy.Subscriber('h_response',
                                        String,
                                        self.r_listen
                                        )
        
        #publisher relative to the robot
        self.r_res_pub = rospy.Publisher('r_response',
                                       String,
                                       queue_size=1
                                       )

    def h_listen(self, msg): #msg not used
        self.h_talk()

    def r_listen(self, msg):
        self.r_talk(msg.data)

    #the system waits for the user to add an input
    def h_talk(self):
        self.string_h_prompt = "Inserire prompt: "
        print(self.string_h_prompt, end="")
        self.h_prompt = input()
        h_msg = String()

        h_msg.data = self.h_prompt
        self.h_res_pub.publish(h_msg)
    
    # Once the model gets an user input, it provides the response
    def r_talk(self, h_prompt):
        prompt = self.ai_chatter.generate_s_prompt()
        #r_ans = self.ai_chatter.generate_response(prompt, h_prompt) #string genereted by the model
        r_ans = "Ciao, come stai?" #Use this line to test the system without wasting tokens
        self.ai_chatter.log.log_input(h_prompt)
        self.ai_chatter.log.log_output(r_ans, self.ai_chatter.model)
        self.r_sound.r_say(r_ans)
        r_msg = String()
        r_msg.data = r_ans
        self.r_res_pub.publish(r_msg)
        self.state = "listen"

if __name__ == "__main__":
    rospy.init_node('chatbot')
    cb = ChatBot()
    rospy.on_shutdown(cb.ai_chatter.log.log_close)
    rate = rospy.Rate(10)
    try:
        cb.h_talk()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')
            