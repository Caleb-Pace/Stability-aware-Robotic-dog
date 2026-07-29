from abc import ABC, abstractmethod

# Taken from: https://github.com/maanas444/go2-simulation/blob/main/unitreego2/output.py
class OutputLayer(ABC):
    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def send_commands(self, target_angles, torques):
        pass
    
    @abstractmethod
    def get_low_state(self):
        pass
