"""
A dummy conversational agent that "generates" responses to user input based on a script (i.e. a list of strings). On each request, the chatter will simply return the next line in the script, regardless the content of the input.
"""

from hri_conversational_agency.logger import Logger

DEFAULT_RESPONSES = ["I am a dummy model, I cannot answer your question.", 
                     "Goodbye!"
                     ]

class DummyChatter():
    def __init__(self, responses = DEFAULT_RESPONSES, restart=False):
        """_summary_

        Args:
            responses (_type_, optional): _description_. Defaults to DEFAULT_RESPONSES.
            restart (bool, optional): _description_. Defaults to False.
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