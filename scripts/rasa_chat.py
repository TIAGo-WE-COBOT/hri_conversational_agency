#!/usr/bin/env python3

import requests
import rospy
from std_msgs.msg import String

class RasaChatBot:
    def __init__(self):
        self.rasa_server_url = 'http://localhost:5005/webhooks/rest/webhook'
        self.rasa_interaction = 0

        self.rasa_curr_mod = ""
        self.rasa_curr_media = ""
        self.rasa_curr_trial = 0
        self.rasa_media = ""

        self.rasa_conv_timer_sub = rospy.Subscriber('check/conv_timer',
                                                    String, 
                                                    self.rasa_end_timer_cb)
        self.last_int_flag = False
        self.media_ext_flag = False
        self.rasa_play_media_flag = False

    def rasa_end_timer_cb(self, rasa_end_chat):
        if(rasa_end_chat.data == "True"):
            self.last_int_flag = True #to be checked to see if the conversation is done
            print("Last interaction")

    def send_request(self, mess):
        if(not self.last_int_flag): #conversazione normale   
            resp = requests.post(self.rasa_server_url, json={"message": mess})
            if resp.status_code == 200: #requested completed
                rasa_response = resp.json()
                self.chatbot_resp = rasa_response[0]['text']
            else:
                print("Errore durante la richiesta al server Rasa")
                return []
        elif(self.last_int_flag):
            if(not self.media_ext_flag):
                if(self.rasa_curr_media == "M"):
                    self.rasa_media = 'musica'
                    print(self.rasa_media)
                elif(self.rasa_curr_media == "V"):
                    self.rasa_media = 'video'
                    print(self.rasa_media)
                elif(self.rasa_curr_media == "AL"):
                    self.rasa_media = 'audiolibro'
                    print(self.rasa_media)
                quest = "chiedi "+ self.rasa_media
                print(quest)
                resp = requests.post(self.rasa_server_url, json={"message": quest})
                if resp.status_code == 200: #requested completed
                    rasa_response = resp.json()
                    self.chatbot_resp = rasa_response[0]['text']
                    self.media_ext_flag = True
                else:
                    print("Errore durante la richiesta al server Rasa")
                    return []
            elif(self.media_ext_flag):                
                resp = requests.post(self.rasa_server_url, json={"message": mess})
                if resp.status_code == 200: #requested completed
                    rasa_response = resp.json()
                    self.chatbot_resp = rasa_response[0]['text']
                    self.rasa_play_media_flag = True
                else:
                    print("Errore durante la richiesta al server Rasa")
                    return []