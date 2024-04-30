#!/usr/bin/env python3

import os
import time
import sys
from datetime import datetime

class ChatLogger:
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
    
    #print model parameters
    def log_model_parameters(self, model, max_tokens, temperature, seed, frequency_penalty, presence_penalty):
        self.f.write('MODEL PARAMETERS:\n \
    Model: {}\n \
    Max tokens: {}\n \
    Temperature: {}\n \
    Seed: {}\n \
    Frequency penalty: {}\n \
    Presence penalty: {}\n\n'.format(model, max_tokens, temperature, seed, frequency_penalty, presence_penalty)
                    )

    #print the sysyem prompt in the log file
    def log_system_prompt(self, s_p_string):
         self.f.write('SYSTEM PROMPT: {}\n\n'.format(s_p_string))

    #print the user input prompt in the log file
    def log_input(self, string):
        self.f.write('USER: {}\n\n'.format(string))

    #print the response of the model in the log file
    def log_output(self, response, model):
        self.f.write('{model}: {text}\n\n'.format(model = model.upper(),
                                                text = response.strip('\n')
                                              ))
        
    #print the current number of the interaction
    def log_curr_interaction(self, ni):
        self.f.write('Interaction n.{}:\n'.format(ni))

    def log_input_tokens(self, input_tokens):
        self.f.write('Total number of input tokens: {} tokens\n'.format(input_tokens))

    def log_output_tokens(self, output_tokens):
        self.f.write('Total number of output tokens: {} tokens\n'.format(output_tokens))

    def log_tot_tokens(self, tot_tokens):
        self.f.write('Total number of tokens: {} tokens\n\n\n'.format(tot_tokens))

    #print the number of interactions considering as a complete interaction a question of the human and the corresponding answer of the robot
    def log_n_interactions(self, n):
        self.f.write('Total number of interactions: {}\n'.format(n))

    #print the duration of the conversation in the log file
    def log_duration(self, t):
        duration = round(t)
        if(t < 60):
            self.f.write('Duration of the conversation: {}\n\n'.format(time.strftime("%Ss", time.gmtime(duration))))

        elif(t < 3600):
            self.f.write('Duration of the conversation: {}\n\n'.format(time.strftime("%Mm%Ss", time.gmtime(duration))))

        else:
            self.f.write('Duration of the conversation: {}\n\n'.format(time.strftime("%Hh%Mm%Ss", time.gmtime(duration))))

    #open the log file and add a line with the current time    
    def log_open(self):
        self.f = open(self.fpath, 'a')
        self.f.write('## {}\n\n'.format(self._get_time(sep = ':')[1]))

    #close the log file 
    def log_close(self):
        #print("Closing log file.")
        self.f.close()
    
    def _get_time(self, sep = '_'):
        now = datetime.now()    # current date and time
        day = now.strftime("%Y{}%m{}%d".format(sep, sep))
        time = now.strftime("%H{}%M{}%S".format(sep, sep))
        return day, time