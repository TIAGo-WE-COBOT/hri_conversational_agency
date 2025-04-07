"""
This module provides a logger class for logging conversations with conversational agents.
The logger creates a log file for each day and appends the conversation to it.
"""
import os
from datetime import datetime

class Logger:
    def __init__(self, agent_name='agent'):
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
        # Define the agent name for logging
        self.agent_name = agent_name.upper()

    def log_input(self, string):
        self.f.write('USER: {}\n\n'.format(string))

    def log_output(self, response):
        self.f.write('{}: {}\n\n'.format(self.agent_name, response))

    def logfile_open(self):
        self.f = open(self.fpath, 'a')
        self.f.write('## {}\n\n'.format(self._get_time()))

    def logfile_close(self):
        self.f.close()

    def _get_date(self):
        return datetime.now().strftime("%Y/%m/%d")
        
    def _get_time(self):
        return datetime.now().strftime("%H:%M:%S")