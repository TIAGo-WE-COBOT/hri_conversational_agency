#!/usr/bin/env python

''' This script is a template to develop your own node implementing a conversational agent. One should override listen and talk methods to define custom behavior.
'''
from emit_sound import EmitSound
from timer import MyTimer
from conversational_agency.openai_utils.chatter import OpenAIChatter
from openpyxl import Workbook, load_workbook
import json
import random
import rospy
from std_msgs.msg import String

class ChatBot():
    def __init__(self):
        self.state = "init" #DEFINE all the states 

        # Initialize the OpenAI model to generate responses 
        self.ai_chatter = OpenAIChatter()
        self.r_sound = EmitSound()

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

        self.slide_col = 0
        self.init_flag = False
        self.idle_flag = False
        self.h_listen_flag = True
        self.r_listen_flag = False
        self.r_talk_flag = False
        self.id_flag = False

    def setup(self):
        if not self.init_flag and self.state == "init":
            print("PHASE: SETUP\n")
            try:
                id = int(input("Inserire l'ID del paziente: "))
                self.n_mod = random.randint(1, 3)

            except ValueError:
                print("Inserire un ID valido")
            
            work_space = load_workbook(filename = "BFI assessment module TRIAL2copy.xlsx", data_only=True) #switch with the real path of the excel file
            #work_space = load_workbook(filename = "test.xlsx", data_only=True)
            pers_data = work_space["Preprocessed_Data"]
            col = pers_data["B"]
            
            while not self.id_flag:
                if col[self.slide_col].value == id:
                    row = pers_data[self.slide_col + 1]
                    self.slide_col = 0
                    self.id_flag = True
                else:
                    self.slide_col += 1
                    self.id_flag = False

            #print(row[0].value) #row[n] is a cell object, to return the value in cell use row[n].value
            #row[0].value = row[0].value.rstrip(";").replace(";",", ").lower() #to format the string relative to the interests
            self.gender = row[7].value.capitalize()
            self.age = row[8].value
            self.education = row[9].value
            self.job = row[10].value
            self.interests = row[11].value.rstrip(";").rstrip().replace(";",", ").lower().capitalize()
            self.extraversion = row[57].value
            self.agreeableness = row[59].value
            self.conscientiousness = row[61].value
            self.neuroticism = row[63].value
            self.openness = row[65].value
            self.init_flag = True
            self.state = "idle"
            self.idle()
        
    def idle(self):
        if self.state == "idle" and not self.idle_flag:
            print("PHASE: IDLE\n")
            self.timer = MyTimer()
            self.state = "listen"
            self.idle_flag = True
        

    def h_listen(self, msg):
        print("PHASE: HUMAN_LISTEN\n") 
        if(msg.data=="flag" and self.state=="listen"):
            msg.data = ""
        if(self.idle_flag):
            self.h_listen_flag = True
        

    def r_listen(self, msg):
        print("PHASE: ROBOT_LISTEN...waiting for a trascription\n") 
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
        print("PHASE: ROBOT_TALK\n")
        if not self.state == "talk":
            raise ValueError("The `r_talk` cb should not be entered when not in state `talk`. How did you get here?!")
        
        #decidere se generare il prompt con il setup oppure con la generazione della risposta
        self.ai_chatter.generate_s_prompt(self.n_mod, self.gender, self.age, self.education, self.job, self.interests, self.extraversion, self.agreeableness, self.conscientiousness, self.neuroticism, self.openness)
        
        if(self.idle_flag and self.r_listen_flag and self.h_listen_flag):
            self.ai_chatter.n_interactions = self.ai_chatter.n_interactions + 1 #
            self.h_listen_flag = False
            self.r_listen_flag = False
        #r_ans = self.ai_chatter.generate_response(self.ai_chatter.n_interactions, self.ai_chatter.s_prompt, h_prompt) #string genereted by the model #
        r_ans = "Ciao, come stai?" #Use this line to test the system without wasting tokens
        #print(h_prompt, "\n")
        #print(r_ans)
        self.ai_chatter.conversational_hystory(h_prompt, r_ans)
        self.ai_chatter.log.log_curr_interaction(self.ai_chatter.n_interactions) #
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
        cb.ai_chatter.log.log_n_interactions(cb.ai_chatter.n_interactions) #
        duration = cb.timer.elapsed_time()
        cb.ai_chatter.log.log_duration(duration)
        cb.ai_chatter.log.log_tot_tokens(cb.ai_chatter.tot_tokens)
        cb.ai_chatter.log.log_close
        print(json.dumps(cb.ai_chatter.conversation, indent=2))

    rospy.on_shutdown(on_shutdown)
    rate = rospy.Rate(10)
    try:
        #cb.idle()
        cb.setup()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')
            
