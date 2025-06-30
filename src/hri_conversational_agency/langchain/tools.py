import requests
import json
from datetime import datetime, timedelta
import calendar

weather_msgs = {
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

datetime_msgs = {
    "en": {
        "output_template": "Today is {weekday} {month} {day}, {year}. It is currently {season}.",
        "error_template": "Sorry, could not get date information.",
        "weekday": [
            "Monday", "Tuesday", "Wednesday", "Thursday", "Friday","Saturday", "Sunday"
                    ],
        "month": [
            "January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"
        ],
        "season": {
            "winter": "winter",
            "spring": "spring",
            "summer": "summer",
            "autumn": "autumn"
        }
    },
    "it": {
        "output_template": "Oggi è {weekday} {day} {month} {year}. È {season}.",
        "error_template": "Spiacente, non è stato possibile ottenere informazioni sulla data.",
        "weekday": [
            "lunedì", "martedì", "mercoledì", "giovedì", "venerdì", "sabato", "domenica"
        ],
        "month": [
            "gennaio", "febbraio", "marzo", "aprile", "maggio", "giugno",  "luglio", "agosto", "settembre", "ottobre", "novembre", "dicembre"
        ],
        "season": {
            "winter": "inverno",
            "spring": "primavera",
            "summer": "estate",
            "autumn": "autunno"
        }
    }
}

def get_weather_info(location: str = None, lang: str = "en") -> str:
    """Get current weather information for a location.
    
    Args:
        location (str): The location to get weather for
        
    Returns:
        str: Weather information as a formatted string
    """
    try:
        # Example using OpenWeatherMap API (you'll need an API key)
        api_key = "646c1930c10a69e4095e057c9ed149da"
        url = f"http://api.openweathermap.org/data/2.5/weather?q={location}&appid={api_key}&units=metric&lang={lang}"
        response = requests.get(url)
        data = response.json()
        # Extract relevant information
        weather = data["weather"][0]["main"].lower()
        temperature = round(data["main"]["temp"])
        # Format the output based on language (see `weather_msgs` dictionary)
        return weather_msgs[lang]["output_template"].format(
                location=location,
                weather=weather_msgs[lang]["weather"][weather],
                temperature=temperature
            )
    except Exception as e:
        print(e) # for debugging
        # Return error message based on language
        return weather_msgs[lang]["error_template"].format(location=location)

def get_datetime_info(date_str: str = None, lang: str = "en") -> str:
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
        # Determine season (northern hemisphere)
        year = date_obj.year
        if date_obj <= datetime(year, 3, 21) or date_obj > datetime(year, 12, 21):
            season = "winter"
        elif date_obj >= datetime(year, 3, 21) and date_obj < datetime(year, 6, 21):
            season = "spring"
        elif date_obj >= datetime(year, 6, 21) and date_obj < datetime(year, 9, 23):
            season = "summer"
        else:
            season = "autumn"
        return datetime_msgs[lang]["output_template"].format(
            weekday=datetime_msgs[lang]["weekday"][date_obj.weekday()],
            month=datetime_msgs[lang]["month"][date_obj.month - 1],
            day=date_obj.day,
            year=year,
            season=datetime_msgs[lang]["season"][season]
        )
    except Exception as e:
        print(e) # for debugging
        # Return error message based on language
        return datetime_msgs[lang]["error_template"]

if __name__ == "__main__":
    # Example usage
    print(get_weather_info("Lecco", lang='it'))
    print(get_datetime_info(lang='it'))