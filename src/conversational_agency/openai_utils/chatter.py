#!/usr/bin/env python

from openai import OpenAI

from conversational_agency.openai_utils.cfg import OPENAI_API_KEY, MODEL, MAX_TOKENS, TEMPERATURE, PERS_SYSTEM_PROMPT_TEMPLATE, STD_SYSTEM_PROMPT_TEMPLATE
from conversational_agency.openai_utils.logger import ChatLogger


class OpenAIChatter():
    def __init__(self, model = MODEL):
        self.log = ChatLogger()
        self.log.log_open()
        self.client = OpenAI()
        self.model = model
    
    def generate_s_prompt(self, n_mod, gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness):
        if(n_mod == 1):
            self.s_prompt = PERS_SYSTEM_PROMPT_TEMPLATE.format(gender, age, education, job, interests, extraversion, agreeableness, conscientiousness, neuroticism, openness)
            self.log.log_system_prompt(self.s_prompt)
            print(self.s_prompt)

        elif(n_mod == 2):
            self.s_prompt = STD_SYSTEM_PROMPT_TEMPLATE
            self.log.log_system_prompt(self.s_prompt)
            print(self.s_prompt)

        elif(n_mod == 3):
            print("chatbot")


    def generate_response(self, prompt, u_prompt, max_tokens = MAX_TOKENS, temperature = TEMPERATURE):
        response = self.client.chat.completions.create(
            model = self.model,
            temperature = temperature,
            max_tokens = max_tokens,
            messages = [
                {"role": "system", "content": prompt},
                {"role": "user", "content": u_prompt}
            ]     
        )
        #DECIDERE SE RITORNARE TUTTO
        model_resp = response.choices[0].message.content
        return model_resp

#print(completion['choices'][0]['message']['content']+'\n') #NON VA FATTO COSÌ
    