#!/usr/bin/env python3

from openai import OpenAI

from .cfg import OPENAI_API_KEY, MODEL, MAX_TOKENS, TEMPERATURE, SEED, FREQUENCY_PENALTY, PRESENCE_PENALTY, \
                            PERS_SYSTEM_PROMPT_TEMPLATE, PERS_SYSTEM_PROMPT_END_TEMPLATE, STD_SYSTEM_PROMPT_TEMPLATE, \
                            STD_SYSTEM_PROMPT_END_TEMPLATE, MUSIC_PROPOSAL_PROMPT, VIDEO_PROPOSAL_PROMPT, AUDIOLIBRO_PROPOSAL_PROMPT

from .logger import ChatLogger

import json

class OpenAIChatter():
    def __init__(self, logdir='log'): # TODO. Use kwargs to set `logdir`
        self.log = ChatLogger(logdir=logdir)
        self.client = OpenAI()
        self.log.log_open()
        self.end_timer_flag = False
        self.check_media_flag = False
        self.play_media_flag = False
        self.find_media_flag = False
        self.end_media_flag = False

        self.curr_mod = ""
        self.curr_media = ""
        self.curr_trial = 0
        self.media_list = ['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']

        self.messages = []
        self.conversation = {}
        self.n_interactions = 0
        self.tot_tokens = 0
        self.compl_tokens = 0
        self.prompt_tokens = 0
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
    ###POSSIBILE RIORGANIZZAZIONE DEL CODICE
    # def generate_s_prompt(self, gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness):
    #     n_mod = 4 #to test a predefined prompt
    #     if(self.curr_mod == "P_LLM"):
    #         if(not self.check_media_flag): #check
    #             if(not self.end_timer_flag):
    #                 self.s_prompt = PERS_SYSTEM_PROMPT_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
    #                 #self.log.log_system_prompt(self.s_prompt)
    #             elif(self.end_timer_flag):
    #                 self.s_prompt = PERS_SYSTEM_PROMPT_END_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
    #                 #self.log.log_system_prompt(self.s_prompt)
    #                 self.check_media_flag = True
    #         elif(self.check_media_flag):
    #             self.play_media_flag = True
    #             self.s_prompt = MEDIA_PROPOSAL_PROMPT.format("['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']")###mettere lista dei media
    #             #self.log.log_system_prompt(self.s_prompt)

    #     elif(self.curr_mod == "LLM"):
    #         if(not self.check_media_flag):
    #             if(not self.end_timer_flag):
    #                 self.s_prompt = STD_SYSTEM_PROMPT_TEMPLATE
    #                 #self.log.log_system_prompt(self.s_prompt)
    #             elif(self.end_timer_flag):
    #                 self.s_prompt = STD_SYSTEM_PROMPT_END_TEMPLATE
    #                 #self.log.log_system_prompt(self.s_prompt)
    #                 self.check_media_flag = True
    #         elif(self.check_media_flag):
    #             self.play_media_flag = True
    #             self.s_prompt = MEDIA_PROPOSAL_PROMPT.format("['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']")###mettere lista dei media
    #             #self.log.log_system_prompt(self.s_prompt)

    ###CODICE RIORGANIZZATO, DA TESTARE, POSSIBILE ULTERIORE RIORGANIZZAZIONE
    # def generate_s_prompt(self, gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness):    
    #         if(not self.check_media_flag): #check
    #             if(not self.end_timer_flag):
    #                 if(self.curr_mod == "P_LLM"):
    #                     self.s_prompt = PERS_SYSTEM_PROMPT_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
    #                 elif(self.curr_mod == "LLM"):
    #                     self.s_prompt = STD_SYSTEM_PROMPT_TEMPLATE
    #             elif(self.end_timer_flag):
    #                 if(self.curr_mod == "P_LLM"):
    #                     self.s_prompt = PERS_SYSTEM_PROMPT_END_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
    #                 elif(self.curr_mod == "LLM"):
    #                     self.s_prompt = STD_SYSTEM_PROMPT_END_TEMPLATE
    #                 self.check_media_flag = True
    #         elif(self.check_media_flag):
    #             self.play_media_flag = True
    #             #mettere le tre condizioni a seconda del media
    #             self.s_prompt = MEDIA_PROPOSAL_PROMPT.format("['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']")###mettere lista dei media

    ###CODICE RIORGANIZZATO 2, DA TESTARE
    # def generate_s_prompt(self, gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness):    
    #          #check
    #         if(not self.end_timer_flag):
    #             if(self.curr_mod == "P_LLM"):
    #                 self.s_prompt = PERS_SYSTEM_PROMPT_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
    #             elif(self.curr_mod == "LLM"):
    #                 self.s_prompt = STD_SYSTEM_PROMPT_TEMPLATE
    #         elif(self.end_timer_flag):
    #             if(not self.check_media_flag):
    #                 if(self.curr_mod == "P_LLM"):
    #                     self.s_prompt = PERS_SYSTEM_PROMPT_END_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
    #                 elif(self.curr_mod == "LLM"):
    #                     self.s_prompt = STD_SYSTEM_PROMPT_END_TEMPLATE
    #                 self.check_media_flag = True
    #             elif(self.check_media_flag):
    #                 if(self.curr_media == "M"):
    #                     self.s_prompt = MUSIC_PROPOSAL_PROMPT.format("['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']")###mettere lista dei media
    #                 elif(self.curr_media == "V"):
    #                     self.s_prompt = VIDEO_PROPOSAL_PROMPT.format("['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']")###mettere lista video
    #                 elif(self.curr_media == "AL"):
    #                     self.s_prompt = AUDIOLIBRO_PROPOSAL_PROMPT.format("['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']")###mettere lista audiolibro
    #                 self.find_media_flag = True #probabilmente è meglio metterla dopo check associazione media-risposta
    
    def generate_s_prompt(self, gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness):    
             #check
            if(not self.end_timer_flag):
                if(self.curr_mod == "P_LLM"):
                    self.s_prompt = PERS_SYSTEM_PROMPT_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
                elif(self.curr_mod == "LLM"):
                    self.s_prompt = STD_SYSTEM_PROMPT_TEMPLATE
            elif(self.end_timer_flag):
                if(not self.check_media_flag):
                    if(self.curr_mod == "P_LLM"):
                        self.s_prompt = PERS_SYSTEM_PROMPT_END_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
                    elif(self.curr_mod == "LLM"):
                        self.s_prompt = STD_SYSTEM_PROMPT_END_TEMPLATE
                    #self.check_media_flag = True
                # elif(self.check_media_flag):
                #     if(self.curr_media == "M"):
                #         self.s_prompt = MUSIC_PROPOSAL_PROMPT.format("['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']")###mettere lista dei media
                #     elif(self.curr_media == "V"):
                #         self.s_prompt = VIDEO_PROPOSAL_PROMPT.format("['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']")###mettere lista video
                #     elif(self.curr_media == "AL"):
                #         self.s_prompt = AUDIOLIBRO_PROPOSAL_PROMPT.format("['Bad Romance', 'Bandita', 'Blue Sky', 'Closer', 'Pamplona']")###mettere lista audiolibro
                #     #self.find_media_flag = True #probabilmente è meglio metterla dopo check associazione media-risposta
    

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
        
        #Trovare modo per cambiare messages in funzione del system prompt
        if(not self.end_timer_flag): #aggiungo tutti i messaggi al system prompt
            response = self.client.chat.completions.create(
                model = self.model,
                temperature = self.temperature,
                max_tokens = self.max_tokens,
                seed = self.seed,
                frequency_penalty = self.frequency_penalty,
                presence_penalty = self.presence_penalty,
                messages = self.messages
            )
        elif(self.end_timer_flag):
            #print("generazione ultimo prompt")
            #print([{"role": "system", "content": self.s_prompt}, self.messages[len(self.messages)-1]])
            response = self.client.chat.completions.create(
                model = self.model,
                temperature = self.temperature,
                max_tokens = self.max_tokens,
                seed = self.seed,
                frequency_penalty = self.frequency_penalty,
                presence_penalty = self.presence_penalty,
                messages = [{"role": "system", "content": self.s_prompt}, self.messages[len(self.messages)-1]]
            )
        
        #DECIDERE SE RITORNARE TUTTO
        self.compl_tokens += response.usage.completion_tokens
        self.prompt_tokens += response.usage.prompt_tokens
        self.tot_tokens += response.usage.total_tokens
        print(self.tot_tokens) #remove after testing
        model_resp = response.choices[0].message.content
        self.messages.append({"role": "assistant", "content": model_resp})
        print(self.messages)

        if(self.end_timer_flag):
            print(model_resp) #DA TESTARE
            media = model_resp.strip("/n") #In some cases the model adds "\n"
            if(media in self.media_list):
                self.play_media_flag = True
            else:
                print("eh no no")
         
        return model_resp
    #  #print(completion['choices'][0]['message']['content']+'\n') #NON VA FATTO COSÌ