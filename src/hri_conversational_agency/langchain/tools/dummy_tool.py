from .base_tool import BaseTool
import random

DUMMY_MSGS = {
    "en": {
        "on_sequence_end": "Sorry, I don't know what to say anymore.",
    },
    "it": {
        "on_sequence_end": "Spiacente, non so più cosa dire.",
    }
}

class DummyTool(BaseTool):
    def __init__(self, config):
        self.responses = config.get("responses", [])
        self.order = config.get("order", "sequential")
        self.language = config.get('language', 'en')

    def __call__(self):
        """Return the next response from the script."""
        return self._get_response()

    def _get_response(self):
        """Get the next response based on the configured order."""
        if not self.responses:
            return DUMMY_MSGS[self.language]["on_sequence_end"]
        if self.order == "sequential":
            # If the order is sequential, return the first response and remove it from the list
            response = self.responses.pop(0)
        elif self.order == "random":
            # If the order is random, return a random response from the list
            response = random.choice(self.responses)
            self.responses.remove(response)
        else:
            raise ValueError(f"Unknown order type: {self.order}")
        return response

    @classmethod
    def get_name(cls):
        return "dummy"
    
    def get_description(cls):
        return "A tool that reads a scripted conversation and plays it back on each call."