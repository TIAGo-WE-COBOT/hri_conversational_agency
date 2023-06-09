import openai

from conversational_agency.openai_utils.cfg import API_KEY
from conversational_agency.openai_utils.logger import ChatLogger
 
MAX_TOKENS  = 256   # max length of the responses
TEMPERATURE = 0.6   # tune the "creativity" (i.e. confabulation) of the generated answers

class OpenAIChatter():
    def __init__(self, openai_api_key = API_KEY, model = "text-curie-001"):
        self.log = ChatLogger()
        self.log.log_open()
        openai.api_key = API_KEY

        self.model = model
    
    def generate_prompt(self, in_text):
        #prompt = "Rispondi all'input sotto. Il testo è la trascrizione automatica di un discorso. Rispondi sempre in italiano. Usa al massimo due frasi.\nText: ###\n{}\n###".format(in_text)
        # Log the input for later inspection of the dialogue
        self.log.log_input(in_text)
        return in_text

    def generate_response(self, prompt, max_tokens = MAX_TOKENS, temperature = TEMPERATURE):
        response = openai.Completion.create(
            model = self.model,
            prompt = prompt,
            max_tokens = max_tokens,    # response max length
            temperature = temperature   # tune the "creativity" of the generated answers
            )
        # Log the output for later inspection of the dialogue
        self.log.log_output(response, self.model)
        return response["choices"][0]["text"]

