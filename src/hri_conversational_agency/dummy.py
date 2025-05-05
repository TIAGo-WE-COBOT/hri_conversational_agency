"""
A dummy conversational agent that "generates" responses to user input based on a script (i.e. a list of strings). On each request, the chatter will simply return the next line in the script, regardless the content of the input.
"""

from hri_conversational_agency.logger import Logger

DEFAULT_RESPONSES = ["I am a dummy model, I cannot answer your question.", 
                     "Goodbye!"
                     ]

class DummyChatter():
    def __init__(self, responses = DEFAULT_RESPONSES, restart=False):
        """Initialize the DummyChatter object.

        Args:
            responses ([str], optional): list of strings where each list item is a line of the script. Defaults to DEFAULT_RESPONSES.
            restart (bool, optional): if True, the DummyChatter will restart from the first line when the last line in the script is reached. Defaults to False.
        """
        self.log = Logger(agent_name='dummy')
        self.log.logfile_open()
        self.responses = responses
        self.restart = restart

    def generate_response(self, request):
        """The method basically implements a script reader. It will return the next line in the script. Unlike typical conversational agents, the method does not take the user input into account. The `request` parameter is used for logging purposes only. The script is defined in the `responses` parameter.

        Returns:
            string: The next line in the script.
        """
        self.log.log_input(request)
        if not hasattr(self, "response_idx"):
            self.response_idx = 0
        response = self.responses[self.response_idx]
        self.response_idx += 1
        if self.response_idx >= len(self.responses):
            if self.restart:
                # The agent will restart the script after the last line
                self.response_idx = 0
            else:
                # The agent will repeat the last line of the script
                self.response_idx = len(self.responses) - 1
        # Log the output for later inspection of the dialogue
        self.log.log_output(response)
        return response

    def set_history(self, history):
        """Receive a list of tuples (role, content) to set the history of the agent. The method is not implemented in this class, as the dummy agent does not have any history. The method is only here to comply with the interface of the other agents.
        The method will raise a NotImplementedError if called.

        Args:
            history ([(str, str)]): List of tuples (role, content) to set the history of the agent.

        Raises:
            NotImplementedError: The method is only here to comply with the interface of the other agents. The dummy agent cannot have any history.
        """
        raise NotImplementedError("The dummy agent cannot implement any history, it is just a script reader.")

    def set_sys_prompt(self, prompt):
        """Receive a system prompt to set the system prompt of the agent. The method is not implemented in this class, as the dummy agent does not have any system prompt. The method is only here to comply with the interface of the other agents.
        The method will raise a NotImplementedError if called.

        Args:
            prompt (str): System prompt to set the system prompt of the agent.

        Raises:
            NotImplementedError: The method is only here to comply with the interface of the other agents. The dummy agent cannot have any system prompt.
        """
        raise NotImplementedError("The dummy agent cannot implement any system prompt, it is just a script reader.")

    def on_shutdown(self):
        self.log.logfile_close()