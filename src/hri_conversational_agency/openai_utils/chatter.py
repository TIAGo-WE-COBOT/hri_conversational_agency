#!/usr/bin/env python3

from openai import OpenAI

from hri_conversational_agency.openai_utils.cfg import OPENAI_API_KEY, MODEL, MAX_TOKENS, TEMPERATURE, SEED, FREQUENCY_PENALTY, PRESENCE_PENALTY, SYSTEM_PROMPT_TEMPLATE, PERS_SYSTEM_PROMPT_TEMPLATE, PERS_SYSTEM_PROMPT_END_TEMPLATE, STD_SYSTEM_PROMPT_TEMPLATE, STD_SYSTEM_PROMPT_END_TEMPLATE, MEDIA_PROPOSAL_PROMPT
from hri_conversational_agency.openai_utils.logger import ChatLogger

import json

class OpenAIChatter():
    def __init__(self):
        self.log = ChatLogger()
        self.client = OpenAI()
        self.log.log_open()
        self.end_timer_flag = False
        self.play_media_flag = False

        self.curr_mod = ""
        self.curr_media = ""
        self.curr_trial = 0

        self.messages = []
        self.conversation = {}
        self.n_interactions = 0
        self.tot_tokens = 0
        self.model = MODEL
        self.max_tokens = MAX_TOKENS
        self.temperature = TEMPERATURE
        self.seed = SEED
        self.frequency_penalty = FREQUENCY_PENALTY
        self.presence_penalty = PRESENCE_PENALTY
        self.log.log_model_parameters(self.model, 
                                      self.max_tokens, 
                                      self.temperature, 
                                      self.seed, 
                                      self.frequency_penalty,
                                      self.presence_penalty
                                      )

    def generate_s_prompt(self, n_mod, gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness):
        n_mod = 4 #to test a predefined prompt
        if(self.curr_mod == "P_LLM"):
            if(not self.play_media_flag): #check
                if(not self.end_timer_flag):
                    self.s_prompt = PERS_SYSTEM_PROMPT_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
                    self.log.log_system_prompt(self.s_prompt)
                elif(self.end_timer_flag):
                    self.s_prompt = PERS_SYSTEM_PROMPT_END_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
                    self.log.log_system_prompt(self.s_prompt)
                    self.play_media_flag = True
            elif(self.play_media_flag):
                self.s_prompt = MEDIA_PROPOSAL_PROMPT.format()###mettere lista dei media
                self.log.log_system_prompt(self.s_prompt)


##############fare come sopra
        elif(self.curr_mod == "LLM"):


                if(not self.end_timer_flag):
                    self.s_prompt = STD_SYSTEM_PROMPT_TEMPLATE
                    self.log.log_system_prompt(self.s_prompt)
                elif(self.end_timer_flag):
                    self.s_prompt = STD_SYSTEM_PROMPT_END_TEMPLATE
                    self.log.log_system_prompt(self.s_prompt)
                    self.play_media_flag = True
                #print(self.s_prompt)

        elif(self.curr_mod == "CHATBOT"):
            print("chatbot")

        elif(n_mod == 4):
            self.s_prompt = SYSTEM_PROMPT_TEMPLATE
            self.log.log_system_prompt(self.s_prompt)
            #print(self.s_prompt)

    #methods to add the system prompts, the user messages and the model responses to the dictionary containing all the conversation
    def add_c_s_prompt(self):
        self.conversation["int"+str(self.n_interactions)] = [{"role": "system", "content": self.s_prompt}]

    def add_c_h_prompt(self, h_prompt):
        self.conversation["int"+str(self.n_interactions)].append({"role": "user", "content": h_prompt})

    def add_c_a_prompt(self, m_response):
        self.conversation["int"+str(self.n_interactions)].append({"role": "assistant", "content": m_response})

    ####to add a complete interaction to the conversational hystory
    def conversational_hystory(self, h_prompt, m_response):
        self.add_c_s_prompt()
        self.add_c_h_prompt(h_prompt)
        self.add_c_a_prompt(m_response)
            
    def generate_response(self, n_interactions, s_prompt, h_prompt):
        if(n_interactions == 1):
            self.messages = [
                {"role": "system", "content": s_prompt}
            ]
        self.messages.append({"role": "user", "content": h_prompt})   
        response = self.client.chat.completions.create(
            model = self.model,
            temperature = self.temperature,
            max_tokens = self.max_tokens,
            seed = self.seed,
            frequency_penalty = self.frequency_penalty,
            presence_penalty = self.presence_penalty,
            messages = self.messages
        )
        #DECIDERE SE RITORNARE TUTTO
        self.tot_tokens = self.tot_tokens + response.usage.total_tokens
        print(self.tot_tokens) #remove after testing
        model_resp = response.choices[0].message.content
        self.messages.append({"role": "assistant", "content": model_resp})
        if(self.end_timer_flag):
            model_resp += "Canzone 1, canzone 2 oppure canzone 3"
        return model_resp
    #  #print(completion['choices'][0]['message']['content']+'\n') #NON VA FATTO COSÌ