"""Define the base class for a Chatter.
"""

from abc import ABC, abstractmethod

class BaseChatter(ABC):
    @abstractmethod
    def generate_response(self, request: str) -> str:
        pass

    def set_context(self, context):
        raise NotImplementedError("The set_context method is not implemented in this class.")

    def set_history(self, history):
        raise NotImplementedError("The set_history method is not implemented in this class.")

    def set_sys_prompt(self, prompt):
        raise NotImplementedError("The set_sys_prompt method is not implemented in this class.")

    def on_shutdown(self):
        pass