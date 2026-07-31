from abc import ABC, abstractmethod

# Adapted from: https://github.com/maanas444/go2-simulation/blob/main/unitreego2/output.py
class OutputLayer(ABC):
    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def send_action(self, target_angles, feedforward_torques):
        pass
    
    @abstractmethod
    def get_low_state(self):
        pass
