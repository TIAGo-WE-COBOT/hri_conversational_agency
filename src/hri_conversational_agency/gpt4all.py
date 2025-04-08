"""
A conversational agent that generates responses to user input based on GPT4All models.
"""

from gpt4all import GPT4All

from hri_conversational_agency.logger import Logger

class GPT4AllChatter():
    def __init__(self, model="Meta-Llama-3-8B-Instruct.Q4_0.gguf", model_path=None, device='gpu',
                 prompt="{request}"):
        """_summary_

        Args:
            model (str, optional): _description_. Defaults to "Meta-Llama-3-8B-Instruct.Q4_0.gguf".
            prompt (str, optional): _description_. Defaults to "{request}".
        """
        self.log = Logger(agent_name=model)
        self.log.logfile_open()
        self.set_prompt(prompt)
        for model_dict in GPT4All.list_models():
            print(model_dict['name'], '\t', model_dict['filename'])
        self.model=GPT4All(model)
        
        

    def generate_response(self, request):
        """_summary_

        Args:
            request (_type_): _description_

        Returns:
            _type_: _description_
        """
        if not hasattr(self, 'chat'):
            self.chat = open(self.model.chat_session())
        self.log.log_input(request)
        response = self.model.generate(request)
        # Log the output for later inspection of the dialogue
        self.log.log_output(response)
        return response

    def on_shutdown(self):
        self.log.logfile_close()
        self.chat.close()
        self.model.close()
    
    def set_context(self, context):
        self.context = context

    def set_prompt(self, prompt):
        self.prompt = prompt

    def _prompt_wrapping(self, request):
        if hasattr(self, 'context'):
            self.prompt
        return self.prompt()
