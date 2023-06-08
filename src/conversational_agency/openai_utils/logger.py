import os
import sys
from datetime import datetime

class ChatLogger:
    def __init__(self):
        # Check for the `log` folder existence
        try:
            os.mkdir('log')
        except FileExistsError:
            pass

        # Create a file for the current day interactions
        self.fname = 'log_' + self._get_time(sep = '_')[0] + '.md'
        self.fpath = os.path.join('.', 'log', self.fname)
        try:
            f = open(self.fpath, 'x')
            f.write('# {} Conversation Log\n\n'.format(self._get_time(sep = '/')[0]))
            f.close()
        except FileExistsError:
            pass
    
    def log_input(self, string):
        self.f.write('USER: {}\n\n'.format(string))

    def log_output(self, response, model):
        self.f.write('{model}: {text}\n\n'.format(model = model.upper(),
                                                text = response["choices"][0]["text"].strip('\n')
                                              ))

    def log_open(self):
        self.f = open(self.fpath, 'a')
        self.f.write('## {}\n\n'.format(self._get_time(sep = ':')[1]))

    def log_close(self):
        self.f.close()

    def _get_time(self, sep = '_'):
        now = datetime.now()    # current date and time
        day = now.strftime("%Y{}%m{}%d".format(sep, sep))
        time = now.strftime("%H{}%M{}%S".format(sep, sep))
        return day, time


