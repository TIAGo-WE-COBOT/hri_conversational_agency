''' You can generate your API key from https://platform.openai.com/account/api-keys 
See the README file for more info about setup and keeping you personal API key secret!
'''
API_KEY     = '<your API key here>'

''' Set the model parameters 
'''
MAX_TOKENS  = 256   # max length of the responses
TEMPERATURE = 0.6   # tune the "creativity" (i.e. confabulation) of the generated answers

''' Describe the kind of reponse you want from the model. {} will be replaced with the text input received by the node implementing the conversational agent.
Some tips to define a good prompt can be found here: https://help.openai.com/en/articles/6654000-best-practices-for-prompt-engineering-with-openai-api
'''
PROMPT_TEMPLATE = "Answer in max two sentences. Answer in the language of the input.\nText:###{}###"