from .weather_tool import WeatherTool
from .datetime_tool import DateTimeTool

# Tool registry - automatically populated
AVAILABLE_TOOLS = {
    WeatherTool.get_name(): WeatherTool,
    DateTimeTool.get_name(): DateTimeTool,
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