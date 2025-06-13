"""Define the base class for a Chatter.
"""

from abc import ABC, abstractmethod

class BaseChatter(ABC):
    @abstractmethod
    def generate_response(self, request: str) -> str:
        pass

    def set_context(self, context):
        raise NotImplementedError

    def set_history(self, history):
        raise NotImplementedError

    def set_sys_prompt(self, prompt):
        raise NotImplementedError

    def on_shutdown(self):
        pass