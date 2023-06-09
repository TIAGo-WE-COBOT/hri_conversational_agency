#!/usr/bin/env python

''' This script is a template to develop your own node implementing a conversational agent. One should override listen and talk methods to define custom behavior.
'''
from conversational_agency.openai_utils.chatter import OpenAIChatter

import rospy
from std_msgs.msg import String

class ChatBot():
    def __init__(self):
        # Initialize the OpenAI model to generate responses
        self.ai_chatter = OpenAIChatter()
        # Listen to user input
        self.req_sub = rospy.Subscriber('request',
                                        String, 
                                        self.listen
                                        )
        # Initialize the publisher to broadcast the model response
        self.res_pub = rospy.Publisher('response',
                                       String, 
                                       queue_size=1
                                       )
        self.text = ""
        self.is_speech_recognized = False
    
    def listen(self, msg):
        self.talk(msg.data)

    def talk(self, in_text):
        prompt = self.ai_chatter.generate_prompt(in_text)
        print(prompt)
        ans = self.ai_chatter.generate_response(prompt)
        self.tts_pub.publish(ans)
        self.text = ""
    
if __name__ == "__main__":
    rospy.init_node('chatbot')
    cb = ChatBot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')