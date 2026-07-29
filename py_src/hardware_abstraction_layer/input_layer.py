from abc import ABC, abstractmethod
from data_structures.controller_input import ControllerData, JoyStickData


class InputLayer(ABC):
    @abstractmethod
    def poll(self) -> ControllerData:
        pass