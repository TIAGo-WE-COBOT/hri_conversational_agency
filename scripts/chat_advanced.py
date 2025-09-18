#!/usr/bin/env python

"""This script implements a ROS node that extends the ChatBotNode to use Langchain for conversational agents.
"""

import os

import rospy
import rospkg

from chat_simple import ChatBotNode
from hri_conversational_agency.langchain.multiprompt import LangchainChatter
from hri_conversational_agency.srv import SetAgentConfig, SetAgentConfigResponse
from hri_conversational_agency.msg import ChainStats, AgentStats
from hri_conversational_agency.srv import GetAgentStats, GetAgentStatsResponse

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
            os.path.join(pkg_root, 'cfg', 'multiprompt_demo.yaml')
        )
        # Initialize the agent to generate responses
        self.chatter = LangchainChatter(
            config=agent_cfg_yaml_path,
        )
        # Initialize the services, publishers and subscribers to interact with the agent via ROS
        self._init_services()
        self._init_subs()
        self._init_pubs()

    def _init_services(self):
        super()._init_services()
        self.get_agent_stats_srv = rospy.Service('get_agent_stats',
                                                 GetAgentStats,
                                                 self.get_agent_stats
                                                 )
        self.set_config_srv = rospy.Service('set_config',
                                            SetAgentConfig,
                                            self.set_config
                                            )
    
    def get_agent_stats(self, req):
        stats = self.chatter.get_agent_stats()
        stats_msg = AgentStats()
        for chain_name, chain_stats in stats.items():
            stats_msg.chain_names.append(chain_name)
            stats_msg.chain_stats.append(
                ChainStats(calls_count=chain_stats['calls_count'])
            )
        res = GetAgentStatsResponse(stats=stats_msg)
        return res

    def set_sys_prompt(self, req):
        raise NotImplementedError(
            "The set_sys_prompt service is not implemented in the ExtendedChatBotNode. Use set_config instead."
            )
    def set_context(self, req):
        raise NotImplementedError(
            "The set_context service is not implemented in the ExtendedChatBotNode. Use set_config instead."
            )   
    
    def set_config(self, req):
        try:
            return SetAgentConfigResponse(
                success=self.chatter.set_config(req.yaml_path),
                status=""
            )
        except Exception as e:
            rospy.logerr("Error setting agent configuration: \n{}".format(e))
            return SetAgentConfigResponse(
                success=False,
                status=str(e)
            )

if __name__ == "__main__":
    rospy.init_node('chatbot')
    """ TODO. Adapt to receive agent "name" (e.g. multiprompt, rag...) and YAML config path.
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
    
