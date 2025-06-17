#!/usr/bin/env python

"""This script implements a ROS node that extends the ChatBotNode to use Langchain for conversational agents.
"""

import os
import yaml

import rospy
import rospkg

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
        r = rospkg.RosPack()
        pkg_root = r.get_path('hri_conversational_agency')
        # Get the agent configuration from the YAML file
        agent_cfg_yaml_path = rospy.get_param(
            '~agent_cfg_yaml_path',
            os.path.join(pkg_root, 'cfg', 'multiprompt_default.yaml')
        )
        with open(agent_cfg_yaml_path, 'r', encoding='utf-8') as f:
            agent_cfg_dict = yaml.safe_load(f)
        # Initialize the agent to generate responses
        self.chatter = LangchainChatter(
            prompts_dict=agent_cfg_dict.pop("prompts_dict", {}),
            router_experts_dict=agent_cfg_dict.pop("router_experts_dict", {}),
            rag_context=agent_cfg_dict.pop("rag_context", ""),
            **agent_cfg_dict
        )
        # Initialize the services, publishers and subscribers to interact with the agent via ROS
        self._init_services()
        self._init_subs()
        self._init_pubs()

    def _init_services(self):
        super()._init_services()
        # Add additional services
        self.set_rag_prompt_srv = rospy.Service('set_rag_prompt',
                                            SetAgentContent, 
                                            self.set_rag_prompt
                                            )
        self.set_fallback_prompt_srv = rospy.Service('set_fallback_prompt',
                                                SetAgentContent, 
                                                self.set_fallback_prompt
                                                )
    
    def set_rag_prompt(self, req):
        """Set the prompt for the RAG (Retrieval-Augmented Generation) agent.

        Args:
            req (hri_conversational_agency/SetAgentContent): The request containing the content to set as prompt.

        Returns:
            SetAgentContentResponse: The response indicating success or failure of the operation.
        """
        try:
            return SetAgentContentResponse(
                success=self.chatter.set_rag_prompt(req.content),
                status=""
            )
        except Exception as e:
            rospy.logerr("Error setting RAG prompt: \n{}".format(e))
            return SetAgentContentResponse(
                success=False,
                status=str(e)
            )

    def set_fallback_prompt(self, req):
        """Set the prompt for the fallback agent.
        
        Args:
            req (hri_conversational_agency/SetAgentContent): The request containing the content to set as prompt.
        
        Returns:
            SetAgentContentResponse: The response indicating success or failure of the operation.
        """
        try:
            return SetAgentContentResponse(
                success=self.chatter.set_fallback_prompt(req.content),
                status=""
            )
        except Exception as e:
            rospy.logerr("Error setting fallback prompt: \n{}".format(e))
            return SetAgentContentResponse(
                success=False,
                status=str(e)
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
    
