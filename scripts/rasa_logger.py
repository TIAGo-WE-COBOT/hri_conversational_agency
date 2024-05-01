#!/usr/bin/env python3

import os
import time
import sys
from datetime import datetime

class RasaChatLogger:
    def __init__(self, logdir='log'):
        # Check for the log folder existence in the working directory
        try:
            os.mkdir(logdir)
        except FileExistsError:
            # Do nothing if the log folder exists
            pass
            
        # Create a file for the current day interactions
        self.fname = 'log_' + self._get_time(sep = '_')[0] + '.md'
        self.fpath = os.path.join(logdir, self.fname)
        try:
            f = open(self.fpath, 'x')
            f.write('# {} Conversation Log\n\n'.format(self._get_time(sep = '/')[0]))
            f.close()
        except FileExistsError:
            pass
    
    ###OK
    #print the user input prompt in the log file
    def     rasa_log_input(self, string):
        self.f.write('USER: {}\n\n'.format(string))

    ###OK
    #print the response of the model in the log file
    def rasa_log_output(self, response):
        self.f.write('CHATBOT: {text}\n\n'.format(text = response.strip('\n')))

    ###OK    
    #print the current number of the interaction
    def rasa_log_curr_interaction(self, ni):
        self.f.write('Interaction n.{}:\n'.format(ni))

    #print the number of interactions considering as a complete interaction a question of the human and the corresponding answer of the robot
    def rasa_log_n_interactions(self, n):
        self.f.write('Total number of interactions: {}\n'.format(n))

    #print the duration of the conversation in the log file
    def rasa_log_duration(self, t):
        duration = round(t)
        if(t < 60):
            self.f.write('Duration of the conversation: {}\n\n'.format(time.strftime("%Ss", time.gmtime(duration))))

        elif(t < 3600):
            self.f.write('Duration of the conversation: {}\n\n'.format(time.strftime("%Mm%Ss", time.gmtime(duration))))

        else:
            self.f.write('Duration of the conversation: {}\n\n'.format(time.strftime("%Hh%Mm%Ss", time.gmtime(duration))))

    #open the log file and add a line with the current time    
    def rasa_log_open(self):
        self.f = open(self.fpath, 'a')
        self.f.write('## {}\n\n'.format(self._get_time(sep = ':')[1]))

    #close the log file 
    def rasa_log_close(self):
        #print("Closing log file.")
        self.f.close()
    
    def _get_time(self, sep = '_'):
        now = datetime.now()    # current date and time
        day = now.strftime("%Y{}%m{}%d".format(sep, sep))
        time = now.strftime("%H{}%M{}%S".format(sep, sep))
        return day, time