#!/usr/bin/env python

import os
from dotenv import load_dotenv
load_dotenv()

''' You can generate your API key from https://platform.openai.com/account/api-keys 
See the README file for more info about setup and keeping you personal API key secret!
The API key is stored in a .env file
'''

OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')

''' Set the model parameters 
'''
MODEL = 'gpt-3.5-turbo'
MAX_TOKENS  = 50   # max length of the responses
TEMPERATURE = 0.6   # tune the "creativity" (i.e. confabulation) of the generated answers

''' Describe the kind of reponse you want from the model. {} will be replaced with the text input received by the node implementing the conversational agent.
Some tips to define a good prompt can be found here: https://help.openai.com/en/articles/6654000-best-practices-for-prompt-engineering-with-openai-api
'''

#prima coppia di prompt
STD_SYSTEM_PROMPT_TEMPLATE = "Sei un robot di servizio di nome Tiago. Devi sostenere una conversazione interamente in italiano per intrattenere una persona. Rispondi in meno di 50 parole."
PERS_SYSTEM_PROMPT_TEMPLATE = "Sei un robot di servizio di nome Tiago. Devi sostenere una conversazione interamente in italiano per intrattenere una persona che possiede le caratteristiche elencate. Rispondi in meno di 50 parole.\n\n \
Caratteristiche:\n \
    genere: {},\n \
    fascia d'eta': {},\n \
    grado di istruzione massimo raggiunto: {},\n \
    principali esperienze lavorative: {},\n \
    interessi: {},\n \
    estroversione {},\n \
    gradevolezza {},\n \
    coscienziosità {},\n \
    nevroticismo {},\n \
    apertura mentale {}.\n"
