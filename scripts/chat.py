#!/usr/bin/env python

''' This node simply waits for a request from the user and then generates a response using the selected conversational agent.
The idea of this node is to implement a standard interface for several conversational agents. The core implementation of the agent should be located in the `hri_conversational_agency/src` subfolder.
Any higher-level interaction coordination should be implemented in a state-machine like node, sending requests to this node and receiving responses.
'''
import argparse

import rospy
from std_msgs.msg import String

class ChatBotNode():
    def __init__(self, backend='dummy', **kwargs):
        # Initialize the agent to generate responses
        if backend == 'dummy':
            from hri_conversational_agency.dummy import DummyChatter
            self.chatter = DummyChatter()
        #elif backend == 'another':
        #    pass
        else:
            raise NotImplementedError("The backend '{}' is not implemented. See `src` folder for available backends, or implement your own bindings for a new backend.".format(backend))
        # Listen to user input
        self.req_sub = rospy.Subscriber('request',
                                        String, 
                                        self.respond
                                        )
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
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Implement a '
    )
    parser.add_argument('-b', '--backend', type=str, 
                        default='dummy',
                        description="The backend used to implement the conversational agent to use. Available backends: dummy, openai"
                        )
    parser.parse_args()
    rospy.init_node('chatbot')
    cb = ChatBotNode(backend=parser.args.backend, 
                     **dict((k,v) 
                            for k,v in vars(parser.args).items() 
                            if k!='backend'
                            )
                    )
    rospy.on_shutdown(cb.chatter.log.logfile_close)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')