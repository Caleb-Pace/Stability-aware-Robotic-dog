from enum import Enum
from data_structures.gait_definition import Gait
from control.gaits import TROT, CRAWL

from data_structures.controller_input import ControllerData, Button


class GaitOptions(Enum):
    TROT = ("Trot", TROT)
    CRAWL = ("Crawl", CRAWL)
    
    @property
    def display_name(self) -> str:
        return self.value[0]

    @property
    def gait(self) -> Gait:
        return self.value[1]

GAIT_SELECTOR = list(GaitOptions)

class GaitSwitcher():
    def __init__(self, first_gait_index:int = 0):
        self._gait_index:int = first_gait_index
        self._set_gait()

        self._l1_btn = Button()
        self._r1_btn = Button()

    def _set_gait(self):
        selected_gait = GAIT_SELECTOR[self._gait_index]
        self.current_gait: Gait     = selected_gait.gait
        self.current_gait_name: str = selected_gait.display_name

    def does_gait_change(self, input_data:ControllerData) -> bool:
        previous_index = self._gait_index

        # Update index
        if self._l1_btn.is_just_pressed(input_data.l1_btn_down):
            self._gait_index -= 1
        if self._r1_btn.is_just_pressed(input_data.r1_btn_down):
            self._gait_index += 1

        # Check for change
        if self._gait_index == previous_index:
            return False

        # Wrap index
        self._gait_index %= len(GAIT_SELECTOR)
        
        # Check for change, again
        if self._gait_index == previous_index:
            return False

        # Change gait
        self._set_gait()
        return True
