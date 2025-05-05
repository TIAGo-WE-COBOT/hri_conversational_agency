"""
A dummy conversational agent that "generates" responses to user input based on a script (i.e. a list of strings). On each request, the chatter will simply return the next line in the script, regardless the content of the input.
"""

from ollama import chat

from hri_conversational_agency.logger import Logger

class OllamaChatter():
    def __init__(self, model='llama3.2:3b', old_messages=[]):
        """Initialize the DummyChatter object.

        Args:
            responses ([str], optional): list of strings where each list item is a line of the script. Defaults to DEFAULT_RESPONSES.
            restart (bool, optional): if True, the DummyChatter will restart from the first line when the last line in the script is reached. Defaults to False.
        """
        self.log = Logger(agent_name='ollama - ' + model)
        self.log.logfile_open()
        self.model = model
        self.messages = old_messages

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