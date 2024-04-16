#!/usr/bin/env python3

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
MAX_TOKENS  = 2   # max length of the responses
TEMPERATURE = 0.8   # Defaults to 1,tune the "creativity" (i.e. confabulation) of the generated answers
SEED = 0
FREQUENCY_PENALTY = 0 # Defaults to 0
PRESENCE_PENALTY = 0 # Defaults to 0
 

''' Describe the kind of reponse you want from the model. {} will be replaced with the text input received by the node implementing the conversational agent.
Some tips to define a good prompt can be found here: https://help.openai.com/en/articles/6654000-best-practices-for-prompt-engineering-with-openai-api
'''

#prima coppia di prompt
STD_SYSTEM_PROMPT_TEMPLATE = "Sei un robot di servizio di nome Tiago. Devi sostenere una conversazione interamente in italiano per intrattenere una persona. L'input della persona proviene da un sistema di trascrizione del parlato e potrebbe risultare non accurato, se non riesci a comprendere l'input, scusati e chiedi di ripetere l'input. Rispondi in meno di 50 parole."
PERS_SYSTEM_PROMPT_TEMPLATE = "Sei un robot di servizio di nome Tiago. Devi sostenere una conversazione interamente in italiano per intrattenere una persona che possiede le seguenti caratteristiche e i seguenti tratti della personalità secondo il modello 'Big Five'. L'input della persona proviene da un sistema di trascrizione del parlato e potrebbe risultare non accurato, se non riesci a comprendere l'input, scusati e chiedi di ripetere l'input. Rispondi in meno di 50 parole.\n\n \
Caratteristiche:\n \
    Genere: {};\n \
    Fascia d'eta': {};\n \
    Grado di istruzione massimo raggiunto: {};\n \
    Principali esperienze lavorative: {};\n \
    Interessi: {}.\n\n \
Tratti della personalità:\n \
    Estroversione: {:.0%};\n \
    Gradevolezza: {:.0%};\n \
    Coscienziosità: {:.0%};\n \
    Nevroticismo: {:.0%};\n \
    Apertura mentale: {:.0%}.\n\n"
    
SYSTEM_PROMPT_TEMPLATE = ""