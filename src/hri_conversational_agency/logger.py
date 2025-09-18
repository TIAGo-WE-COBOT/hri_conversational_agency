"""
This module provides a logger class for logging conversations with conversational agents.
The logger creates a log file for each day and appends the conversation to it.
"""
import os
from datetime import datetime

class Logger:
    def __init__(self, agent_name='agent', verbose=False):
        # Check for the `log` folder existence
        pkg_root = os.path.join(
            os.path.abspath(os.path.dirname(__file__)),
            '..', # up to 'src 
            '..'  # up to the package root
            )
        try:
            os.mkdir(os.path.join(pkg_root, 'log'))
        except FileExistsError:
            pass
        # Create a file for the current day interactions
        self.fname = 'log_' + self._get_date().replace('/', '_') + '.md'
        self.fpath = os.path.join(pkg_root, 'log', self.fname)
        try:
            f = open(self.fpath, 'x')
            f.write('# {} Conversation Log\n\n'.format(self._get_date()))
            f.close()
        except FileExistsError:
            pass
        # Define the agent name for logging (if provided)
        self.agent_name = agent_name.upper()
        # Enable disable printing messages to 
        self.verbose = verbose

    def log_input(self, string):
        msg = 'USER: {}'.format(string)
        if self.verbose:
            print(msg)
        self.f.write(msg + '\n\n' )

    def log_output(self, response):
        msg = '{}: {}'.format(self.agent_name, response)
        if self.verbose:
            print(msg)
        self.f.write(msg + '\n\n')

    def log_system_prompt(self, prompt):
        self.f.write('SYSTEM: {}\n\n'.format(prompt))

    def logfile_open(self):
        self.f = open(self.fpath, 'a')
        self.f.write('## {}\n\n'.format(self._get_time()))

    def logfile_close(self):
        self.f.close()
    
    def set_agent_name(self, name):
        self.agent_name = name.upper()

    def _get_date(self):
        return datetime.now().strftime("%Y/%m/%d")
        
    def _get_time(self):
        return datetime.now().strftime("%H:%M:%S")