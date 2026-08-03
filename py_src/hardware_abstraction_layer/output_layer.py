import threading

from abc import ABC, abstractmethod
from data_structures import Action


# Adapted from: https://github.com/maanas444/go2-simulation/blob/main/unitreego2/output.py
class OutputLayer(ABC):
    @abstractmethod
    def connect(self, interrupt:threading.Event):
        pass

    @abstractmethod
    def send_action(self, action:Action):
        pass
    
    @abstractmethod
    def get_low_state(self):
        pass
