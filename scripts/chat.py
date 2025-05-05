#!/usr/bin/env python

''' This node simply waits for a request from the user and then generates a response using the selected conversational agent.
The idea of this node is to implement a standard interface for several conversational agents. The core implementation of the agent should be located in the `hri_conversational_agency/src` subfolder.
Any higher-level interaction coordination should be implemented in a state-machine like node, sending requests to this node and receiving responses.
'''
import sys
import argparse

import rospy
import rospkg
from std_msgs.msg import String

from hri_conversational_agency.srv import SetHistory, SetHistoryResponse, SetSystemPrompt, SetSystemPromptResponse

class ChatBotNode():
    def __init__(self, backend='dummy', **kwargs):
        r = rospkg.RosPack()
        pkg_root = r.get_path('hri_conversational_agency')
        # Initialize the agent to generate responses
        if backend == 'dummy':
            from hri_conversational_agency.dummy import DummyChatter
            self.chatter = DummyChatter()
        elif backend == 'gpt4all':
            from hri_conversational_agency.gpt4all import GPT4AllChatter
            self.chatter = GPT4AllChatter(model=pkg_root + '/models')
        elif backend == 'ollama':
            from hri_conversational_agency.ollama import OllamaChatter
            self.chatter = OllamaChatter()
        else:
            raise NotImplementedError("The backend '{}' is not implemented. See `src` folder for available backends, or implement your own bindings for a new backend.".format(backend))
        # Expose services for the user to interact with the agent
        self.set_history_srv = rospy.Service('set_history', 
                                             SetHistory, 
                                             self.set_history
                                             )
        self.set_sys_prompt_srv = rospy.Service('set_sys_prompt',
                                                SetSystemPrompt, 
                                                self.set_sys_prompt
                                                )
        # Listen to user input
        self.req_sub = rospy.Subscriber('request',
                                        String, 
                                        self.respond
                                        )
        # TODO. Consider switching to `ChatMessage` msg type (cleaner, though more complex for quick-and-dirty use).
        # Initialize the publisher to broadcast the model response
        self.res_pub = rospy.Publisher('response',
                                       String, 
                                       queue_size=1
                                       )
        self.text = ""

    def respond(self, msg):
        ans = self.chatter.generate_response(msg.data)
        self.res_pub.publish(ans)
        self.text = ""
    
    def set_history(self, req):
        try:
            self.chatter.set_history([(chat_msg.role, chat_msg.content) 
                                    for chat_msg in req.history]
                                    )
        except Exception as e:
            rospy.logerr("Error setting history: \n{}".format(e))
            return SetHistoryResponse(False, str(e))
        return SetHistoryResponse(True, "")

    def set_sys_prompt(self, req):
        try:
            self.chatter.set_sys_prompt(req.content)
        except Exception as e:
            rospy.logerr("Error setting system prompt: \n{}".format(e))
            return SetSystemPromptResponse(False, str(e))
        return SetSystemPromptResponse(True, "")
    
if __name__ == "__main__":
    rospy.init_node('chatbot')
    parser = argparse.ArgumentParser(
        description='A ROS node implementing a text-based conversational agent'
        )
    parser.add_argument('-b', '--backend', type=str, 
                        default='dummy',
                        help="The backend used to implement the conversational agent to use. Available backends: dummy, openai"
                        )
    args, kwargs = parser.parse_known_args(rospy.myargv()[1:])
    cb = ChatBotNode(backend=args.backend, 
                     **dict((kwargs[i].rstrip('--'), kwargs[i+1]) 
                     for i in range(0, len(kwargs), 2))
                    )
    rospy.on_shutdown(cb.chatter.on_shutdown)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')