import openai

from hri_conversational_agency.openai.cfg import API_KEY, MAX_TOKENS, TEMPERATURE, PROMPT_TEMPLATE
from hri_conversational_agency.openai.logger import OpenAILogger    

class OpenAIChatter():
    def __init__(self, openai_api_key = API_KEY, model = "text-curie-001"):
        self.log = OpenAILogger()
        self.log.log_open()
        openai.api_key = API_KEY

        self.model = model
    
    def generate_prompt(self, in_text):
        prompt = PROMPT_TEMPLATE.format(in_text)
        # Log the input for later inspection of the dialogue
        self.log.log_input(prompt)
        return prompt

    def generate_response(self, msg, max_tokens = MAX_TOKENS, temperature = TEMPERATURE):
        prompt = self.generate_prompt(msg.data)
        response = openai.Completion.create(
            model = self.model,
            prompt = prompt,
            max_tokens = max_tokens,    # response max length
            temperature = temperature   # tune the "creativity" of the generated answers
            )
        # Log the output for later inspection of the dialogue
        self.log.log_output(response, self.model)
        return response["choices"][0]["text"]

