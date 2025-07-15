from abc import ABC, abstractmethod

class BaseTool(ABC):
    """Base class for all tools in the conversational agent."""
    
    @abstractmethod
    def __call__(self, **kwargs):
        """Execute the tool with given parameters."""
        pass
    
    @classmethod
    @abstractmethod
    def get_name(cls):
        """Return the tool name for YAML configuration."""
        pass
    
    @classmethod
    @abstractmethod
    def get_description(cls):
        """Return the tool description for the router."""
        pass
    
    @classmethod
    def build_chain(cls, config):
        """Build the tool chain from configuration."""
        tool_instance = cls(config)
        return lambda: tool_instance()