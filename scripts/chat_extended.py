#!/usr/bin/env python

''' This node simply waits for a request from the user and then generates a response using the selected conversational agent.
The idea of this node is to implement a standard interface for several conversational agents. The core implementation of the agent should be located in the `hri_conversational_agency/src` subfolder.
Any higher-level interaction coordination should be implemented in a state-machine like node, sending requests to this node and receiving responses.
'''

import rospy

from chat import ChatBotNode
from hri_conversational_agency.langchain.multiprompt import LangchainChatter
from hri_conversational_agency.srv import SetAgentContent, SetAgentContentResponse

class ExtendedChatBotNode(ChatBotNode):
    def __init__(self, backend='langchain', **kwargs):
        """Initialize the ExtendedChatBotNode object.

        Args:
            backend (str, optional): The name of the model to use for the Langchain agent. Defaults to 'langchain'.
            The model should be available in the Langchain server. You can check the available models with `langchain list` and retrieve the model with `langchain pull <model_name>` if needed.
        """
        self.chatter = LangchainChatter()
        # Initialize the services, publishers and subscribers to interact with the agent via ROS
        self._init_services()
        self._init_subs()
        self._init_pubs()

    def _init_services(self):
        super()._init_services()
        # Add additional services
        self.set_rag_prompt = rospy.Service('set_rag_prompt',
                                            SetAgentContent, 
                                            self.set_context
                                            )
        self.set_fallback_prompt = rospy.Service('set_fallback_prompt',
                                                SetAgentContent, 
                                                self.set_fallback_prompt
                                                )
    
    def set_rag_prompt(self, req):
        return SetAgentContentResponse(
            success=self.chatter.set_sys_prompt(req.content)
        )

    def set_fallback_prompt(self, req):
        return SetAgentContentResponse(
            success=self.chatter.set_fallback_prompt(req.content)
        )

if __name__ == "__main__":
    rospy.init_node('chatbot')
    """
    parser = argparse.ArgumentParser(
        description='A ROS node implementing a text-based conversational agent'
        )
    parser.add_argument('-b', '--backend', type=str, 
                        default='dummy',
                        help="The backend used to implement the conversational agent to use. Available backends: dummy, openai"
                        )
    args, kwargs = parser.parse_known_args(rospy.myargv()[1:])
    """
    cb = ExtendedChatBotNode()
    rospy.on_shutdown(cb.chatter.on_shutdown)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down on user request.')
    
