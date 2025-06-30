import requests
import json
from datetime import datetime
import calendar

def get_weather_info(location: str) -> str:
    """Get current weather information for a location.
    
    Args:
        location (str): The location to get weather for
        
    Returns:
        str: Weather information as a formatted string
    """
    try:
        # Example using OpenWeatherMap API (you'll need an API key)
        api_key = "646c1930c10a69e4095e057c9ed149da"
        url = f"http://api.openweathermap.org/data/2.5/weather?q={location}&appid={api_key}&units=metric"
        response = requests.get(url)
        data = response.json()
        # TODO. Implement logic for translation
        return f"Weather in {location}: {data['weather'][0]['description']}, {data['main']['temp']}°C"
    except Exception as e:
        return f"Could not get weather for {location}: {str(e)}"

def get_datetime_info(date_str: str = None) -> str:
    """Get month, date, and season information.
    
    Args:
        date_str (str, optional): Date string in YYYY-MM-DD format. If None, uses current date.
        
    Returns:
        str: Date and season information as a formatted string
    """
    try:
        if date_str:
            date_obj = datetime.strptime(date_str, "%Y-%m-%d")
        else:
            date_obj = datetime.now()
        
        month = calendar.month_name[date_obj.month]
        day = date_obj.day
        year = date_obj.year
        
        # Determine season (Northern Hemisphere)
        month_num = date_obj.month
        if month_num in [12, 1, 2]:
            season = "Winter"
        elif month_num in [3, 4, 5]:
            season = "Spring"
        elif month_num in [6, 7, 8]:
            season = "Summer"
        else:
            season = "Autumn"
        
        return f"Date: {month} {day}, {year}. Season: {season}"
    except Exception as e:
        return f"Could not parse date information: {str(e)}"

if __name__ == "__main__":
    # Example usage
    print(get_weather_info("Lecco"))
    print(get_datetime_info())