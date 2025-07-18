from .datetime_tool import DateTimeTool
from .dummy_tool import DummyTool
from .weather_tool import WeatherTool

# Tool registry - automatically populated
AVAILABLE_TOOLS = {
    DateTimeTool.get_name(): DateTimeTool,
    DummyTool.get_name(): DummyTool,
    WeatherTool.get_name(): WeatherTool
}

def get_tool_class(tool_name):
    """Get tool class by name."""
    return AVAILABLE_TOOLS.get(tool_name)

def list_available_tools():
    """List all available tool names."""
    return list(AVAILABLE_TOOLS.keys())

def get_tool_descriptions():
    """Get descriptions for all available tools."""
    return {name: tool_class.get_description() 
            for name, tool_class in AVAILABLE_TOOLS.items()}