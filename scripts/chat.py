#!/usr/bin/env python

''' This script is a template to develop your own node implementing a conversational agent. One should override listen and talk methods to define custom behavior.
'''
from class_SAY import SAY
from timer import TIMER

from conversational_agency.openai_utils.chatter import OpenAIChatter

import rospy
from std_msgs.msg import String

class ChatBot():
    def __init__(self):
        self.state = "idle" #DEFINE all the states 

        # Initialize the OpenAI model to generate responses 
        self.ai_chatter = OpenAIChatter()
        self.r_sound = SAY()

        # Listen to the model response
        # When the model provides a response it triggers the method to let the user add a request
        self.h_req_sub = rospy.Subscriber('r_response',
                                        String,
                                        self.h_listen
                                        )

        # listen to the user input
        # When the user add a request it triggers the model to response to the user input
        # It has to suscribe to the topic used by ros_whisper
        self.r_req_sub = rospy.Subscriber('asr/string',
                                        String,
                                        self.r_listen
                                        )
        
        # publisher relative to the robot
        self.r_res_pub = rospy.Publisher('r_response',
                                        String,
                                        queue_size=1
                                        )
        self.timer = TIMER()

        self.n_interactions = 0

        self.idle_flag = False
        self.h_listen_flag = True
        self.r_listen_flag = False
        self.r_talk_flag = False
        
    def idle(self):
        self.state = "listen"
        self.idle_flag = True
        

    def h_listen(self, msg): 
        if(msg.data=="flag" and self.state=="listen"):
            msg.data = ""
        if(self.idle_flag):
            self.h_listen_flag = True
        

    def r_listen(self, msg): 
        if not msg.data: #wait 'till string received
            rospy.logwarn('Empty string received, just skipping.')
            return
        if self.state == "listen":
            self.state = "talk"
            if(self.idle_flag):
                self.r_listen_flag = True
            self.r_talk(msg.data)
    
    # Once the model gets an user input, it provides the response
    def r_talk(self, h_prompt):
        if not self.state == "talk":
            raise ValueError("The `r_talk` cb should not be entered when not in state `talk`. How did you get here?!")
        prompt = self.ai_chatter.generate_s_prompt()
        #r_ans = self.ai_chatter.generate_response(prompt, h_prompt) #string genereted by the model
        r_ans = "Ciao, come stai?" #Use this line to test the system without wasting tokens
        print(r_ans)
        if(self.idle_flag and self.r_listen_flag and self.h_listen_flag):
            self.n_interactions = self.n_interactions + 1
            self.h_listen_flag = False
            self.r_listen_flag = False
        self.ai_chatter.log.log_curr_interaction(self.n_interactions)
        self.ai_chatter.log.log_input(h_prompt)
        self.ai_chatter.log.log_output(r_ans, self.ai_chatter.model)
        self.r_sound.r_say(r_ans)
        r_msg = String()
        r_msg.data = "flag"
        self.r_res_pub.publish(r_msg)
        r_msg.data = ""
        self.state = "listen"
        

if __name__ == "__main__":
    rospy.init_node('chatbot')
    cb = ChatBot()

    def on_shutdown():
        cb.ai_chatter.log.log_n_interactions(cb.n_interactions)
        duration = cb.timer.elapsed_time()
        cb.ai_chatter.log.log_duration(duration)
        cb.ai_chatter.log.log_close

    rospy.on_shutdown(on_shutdown)

    rate = rospy.Rate(10)
    try:
        cb.idle()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')
            