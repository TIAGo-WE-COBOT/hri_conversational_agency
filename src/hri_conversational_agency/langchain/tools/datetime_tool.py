from .base_tool import BaseTool
from datetime import datetime

# DateTime message templates
DATETIME_MSGS = {
    "en": {
        "output_template": "Today is {weekday} {month} {day}, {year}. It is {hour}:{minute}. It is currently {season}.",
        "error_template": "Sorry, could not get date information.",
        "hour_period": 12,  # i.e. AM/PM format
        "weekday": [
            "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"
        ],
        "month": [
            "January", "February", "March", "April", "May", "June", 
            "July", "August", "September", "October", "November", "December"
        ],
        "season": {
            "winter": "winter",
            "spring": "spring",
            "summer": "summer",
            "autumn": "autumn"
        }
    },
    "it": {
        "output_template": "Oggi è {weekday} {day} {month} {year}. Sono le {hour} e {minute} minuti. È {season}.",
        "error_template": "Spiacente, non è stato possibile ottenere informazioni sulla data.",
        "hour_period": 24,  # i.e. 24-hour format
        "weekday": [
            "lunedì", "martedì", "mercoledì", "giovedì", "venerdì", "sabato", "domenica"
        ],
        "month": [
            "gennaio", "febbraio", "marzo", "aprile", "maggio", "giugno",
            "luglio", "agosto", "settembre", "ottobre", "novembre", "dicembre"
        ],
        "season": {
            "winter": "inverno",
            "spring": "primavera",
            "summer": "estate",
            "autumn": "autunno"
        }
    }
}

class DateTimeTool(BaseTool):
    def __init__(self, config):
        self.language = config.get('language', 'en')
    
    def __call__(self):
        """Get current date and time information."""
        return self._get_datetime_info(None, self.language)
    
    def _get_datetime_info(self, date_str: str = None, lang: str = "en") -> str:
        """Get month, date, and season information.
        
        Args:
            date_str (str, optional): Date string in YYYY-MM-DD format. If None, uses current date.
            lang (str): Language code for output
            
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
            
            # Format the hour based on `hour_period`
            if DATETIME_MSGS[lang]["hour_period"] == 12:
                hour = date_obj.hour % 12 or 12
                minute = f"{date_obj.minute:02d}"
                period = "AM" if date_obj.hour < 12 else "PM"
                minute += period
            else:
                hour = date_obj.hour
                minute = f"{date_obj.minute:02d}"
            
            # Format the output based on language
            return DATETIME_MSGS[lang]["output_template"].format(
                weekday=DATETIME_MSGS[lang]["weekday"][date_obj.weekday()],
                month=DATETIME_MSGS[lang]["month"][date_obj.month - 1],
                day=date_obj.day,
                year=year,
                hour=hour,
                minute=minute,
                season=DATETIME_MSGS[lang]["season"][season]
            )
        except Exception as e:
            print(f"DateTime error: {e}")
            # Return error message based on language
            return DATETIME_MSGS[lang]["error_template"]
    
    @classmethod
    def get_name(cls):
        return "datetime"
    
    @classmethod
    def get_description(cls):
        return "provides current date, month, and season information"

# For testing
if __name__ == "__main__":
    tool = DateTimeTool({"language": "it"})
    print(tool())