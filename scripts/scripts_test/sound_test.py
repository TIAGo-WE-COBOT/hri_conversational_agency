#!/usr/bin/env python

# ''' This script is a template to develop your own node implementing a conversational agent. One should override listen and talk methods to define custom behavior.
# '''
# from class_SAY import SAY
# #from sound_play.msg import SoundRequest
# #from sound_play.libsoundplay import SoundClient

# import time
# import rospy
# from std_msgs.msg import String

# class ChatBot():
#     def __init__(self):
#         self.state = "idle" #DEFINE all the states 

#         # Initialize the OpenAI model to generate responses 
#         self.r_sound = SAY()

#         # Listen to the model response
#         # When the model provides a response it triggers the method to let the user add a request
#         self.h_req_sub = rospy.Subscriber('r_response',
#                                         String,
#                                         self.h_listen
#                                         )

#         # listen to the user input
#         # When the user add a request it triggers the model to response to the user input
#         # It has to suscribe to the topic used by ros_whisper
#         self.r_req_sub = rospy.Subscriber('asr/string',
#                                         String,
#                                         self.r_listen
#                                         )
        
#         # publisher relative to the robot
#         self.r_res_pub = rospy.Publisher('r_response',
#                                         String,
#                                         queue_size=1
#                                         )
        
#     # def is_playing(self):
#     #     print(self.r_sound._playing)
#     #     return
        
#     def idle(self):
#         self.state = "listen"

#     def h_listen(self, msg): 
#         if(msg.data=="flag" and self.state=="listen"):
#             msg.data = ""

#     def r_listen(self, msg):
#         print(msg.data)
#         while not msg.data: #wait 'till string received
#             pass
#         if self.state == "listen":
#             self.state = "talk"
#             self.r_talk(msg.data)   
    
#     # Once the model gets an user input, it provides the response
#     def r_talk(self, r):
#         # assert self.state == "talk" "The r_talk ..."
#         if not self.state == "talk":
#             raise ValueError("The `r_talk` cb should not be entered when not in state `talk`. How did you get here?!")
#         #r_ans = self.ai_chatter.generate_response(prompt, h_prompt) #string genereted by the model
#         r_ans = "Ciao, come stai?" #Use this line to test the system without wasting tokens
#         self.r_sound.r_say(r_ans)
#         r_msg = String()
#         r_msg.data = "flag"
#         self.r_res_pub.publish(r_msg)
#         r_msg.data = ""
#         self.state = "listen"

# if __name__ == "__main__":
#     rospy.init_node('chatbot')
#     cb = ChatBot()
#     #rospy.on_shutdown(cb.ai_chatter.log.log_close)
#     rate = rospy.Rate(10)
#     try:
#         #cb.r_sound.is_playing()
#         # cb.h_talk()
#         cb.idle()
#         rospy.spin()
#     except KeyboardInterrupt:
#         rospy.loginfo('Shutting down on user request.')


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
        #r_ans = "Ciao, come stai?"
        #self.r_sound.r_say(r_ans)
        self.r_talk(self.state) 
    
    # Once the model gets an user input, it provides the response
    def r_talk(self, r):
        rospy.init_node('chatbot')
        rate = rospy.Rate(0.1)
        r_ans = "Ciao, come stai? Io sto molto bene, oggi voglio andare al parco" #Use this line to test the system without wasting tokens
        while not rospy.is_shutdown():
            #self.r_sound.is_playing()
            self.r_sound.r_say(r_ans)
            #self.r_sound.is_playing()
            #rospy.loginfo("c")
            rate.sleep()
            #rospy.spin()

if __name__ == "__main__":
    #rospy.init_node('chatbot')
    cb = ChatBot()
    #rospy.on_shutdown(cb.ai_chatter.log.log_close)
    #rate = rospy.Rate(0.2)
    try:
        r = "r"
        cb.r_talk(r)
        #rospy.spin()
    except KeyboardInterrupt:
        pass
            