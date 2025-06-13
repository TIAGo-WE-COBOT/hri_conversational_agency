"""
A conversational agent that generates responses to user input based on Ollama models.
"""

from ollama import chat

from hri_conversational_agency.base import BaseChatter
from hri_conversational_agency.logger import Logger

class OllamaChatter(BaseChatter):
    def __init__(self, model='llama3.2:3b'):
        """Initialize the OllamaChatter object.

        Args:
            model (str, optional): The name of the model to use for the Ollama agent. Defaults to 'llama3.2:3b'.
            The model should be available in the Ollama server. You can check the available models with `ollama list` and retrieve the model with `ollama pull <model_name>` if needed.
        """
        self.log = Logger(agent_name='ollama - ' + model)
        self.log.logfile_open()
        self.model = model

    def generate_response(self, request):
        """The method basically implements a script reader. It will return the next line in the script. Unlike typical conversational agents, the method does not take the user input into account. The `request` parameter is used for logging purposes only. The script is defined in the `responses` parameter.

        Returns:
            string: The next line in the script.
        """
        # Log the input for later inspection of the dialogue
        self.log.log_input(request)
        # Generate the response
        response = chat(
            self.model,
            messages=self.messages + [{'role': 'user', 'content': request}]
        )
        # Add the reponse to the messages to maintain the chat history
        self.messages += [
            {'role': 'user', 'content': request},
            {'role': 'assistant', 'content': response.message.content}
        ]
        # Log the output for later inspection of the dialogue
        self.log.log_output(response.message.content)
        return response.message.content

    def set_history(self, history):
        """Receive a list of tuples (role, content) to set the history of the agent. Any previous history will be overwritten.

        Args:
            history ([(str, str)]): List of tuples (role, content) to set the history of the agent.
        """
        self.messages = [{'role': role, 'content': content} 
                         for role, content in history
                         if role in ['user', 'assistant', 'system']]
        # TODO. Return the history as set (?).

    def set_sys_prompt(self, prompt):
        """Receive a system prompt to set the system prompt of the agent. Any previous system prompt will be deleted.
        
        Args:
            prompt (str): System prompt to set the system prompt of the agent.
        """
        # Parse messages to remove the system prompt
        messages = [message for message in self.messages 
                    if message['role'] != 'system']
        # Append the new system prompt to the list of messages
        self.messages = messages+ [{'role': 'system', 'content': prompt}]
        # Log the system prompt for later inspection of the dialogue
        self.log.log_system_prompt(prompt)

    def on_shutdown(self):
        self.log.logfile_close()