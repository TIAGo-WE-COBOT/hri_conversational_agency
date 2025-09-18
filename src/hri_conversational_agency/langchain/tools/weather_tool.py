from .base_tool import BaseTool
from ._secrets import OPEN_WEATHER_API_KEY
import requests

# Weather message templates
WEATHER_MSGS = {
    "en": {
        "output_template": "Today the weather in {location} is {weather}, with a temperature of {temperature} degrees.",
        "error_template": "Sorry, could not get weather info for {location}.",
        "weather": {
            "clear": "clear",
            "clouds": "cloudy",
            "rain": "rainy",
            "snow": "snowy",
            "thunderstorm": "stormy",
            "drizzle": "drizzly"
        }
    },
    "it": {
        "output_template": "Oggi il meteo a {location} è {weather}, con una temperatura di {temperature} gradi.",
        "error_template": "Spiacente, non è stato possibile ottenere informazioni meteo per {location}.",
        "weather": {
            "clear": "sereno",
            "clouds": "nuvoloso",
            "rain": "piovoso",
            "snow": "nevoso",
            "thunderstorm": "temporalesco",
            "drizzle": "leggermente piovoso"
        }
    }
}

class WeatherTool(BaseTool):
    def __init__(self, config):
        self.location = config.get('location', 'New York')
        self.language = config.get('language', 'en')
        self.api_key = config.get('api_key', OPEN_WEATHER_API_KEY)
    
    def __call__(self):
        """Get weather information."""
        return self._get_weather_info(self.location, self.language)
    
    def _get_weather_info(self, location: str, lang: str = "en") -> str:
        """Get current weather information for a location.
        
        Args:
            location (str): The location to get weather for
            lang (str): Language code for output
            
        Returns:
            str: Weather information as a formatted string
        """
        try:
            url = f"http://api.openweathermap.org/data/2.5/weather?q={location}&appid={self.api_key}&units=metric&lang={lang}"
            response = requests.get(url)
            data = response.json()
            
            # Extract relevant information
            weather = data["weather"][0]["main"].lower()
            temperature = round(data["main"]["temp"])
            
            # Format the output based on language
            return WEATHER_MSGS[lang]["output_template"].format(
                location=location,
                weather=WEATHER_MSGS[lang]["weather"].get(weather, weather),
                temperature=temperature
            )
        except Exception as e:
            print(f"Weather API error: {e}")
            # Return error message based on language
            return WEATHER_MSGS[lang]["error_template"].format(location=location)
    
    @classmethod
    def get_name(cls):
        return "weather"
    
    @classmethod
    def get_description(cls):
        return "provides current weather information for the specified location"

# For testing
if __name__ == "__main__":
    tool = WeatherTool({"location": "Lecco", "language": "it"})
    print(tool())
