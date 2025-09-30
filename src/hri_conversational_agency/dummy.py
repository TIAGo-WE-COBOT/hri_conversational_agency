"""
A dummy conversational agent that "generates" responses to user input based on a script (i.e. a list of strings). On each request, the chatter will simply return the next line in the script, regardless the content of the input.
"""
import os
import yaml

from hri_conversational_agency.base import BaseChatter
from hri_conversational_agency.logger import Logger

DEFAULT_RESPONSES = ["I am a dummy model, I cannot answer your question.", 
                     "Goodbye!"
                     ]

class DummyChatter(BaseChatter):
    def __init__(self, responses = DEFAULT_RESPONSES, restart=False):
        """Initialize the DummyChatter object.

        Args:
            responses (str | [str], optional): (list of) string(s) or path to valid YAML file containing a list of strings. Each list item is a line of the script. Defaults to DEFAULT_RESPONSES.
            restart (bool, optional): if True, the DummyChatter will restart from the first line when the last line in the script is reached. Defaults to False.
        """
        self.log = Logger(agent_name='dummy')
        self.log.logfile_open()
        if type(responses) == list:
            self.responses = responses
        elif type(responses) == str:
            if os.path.exists(responses) and responses.endswith('.yaml'):
                # Load responses from YAML file
                with open(responses, 'r', encoding='utf-8') as f:
                    self.responses = yaml.safe_load(f)
        self.is_end_reached = False
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
            self.is_end_reached = True # does not impact the behavior of the agent, but can be used from outside to check if the agent has finished the script
            if self.restart:
                # The agent will restart the script after the last line
                self.response_idx = 0
            else:
                # The agent will repeat the last line of the script
                self.response_idx = len(self.responses) - 1
        # Log the output for later inspection of the dialogue
        self.log.log_output(response)
        return response

    def on_shutdown(self):
        self.log.logfile_close()