import numpy as np
from data_structures import Point3D, Vector
from typing import NamedTuple

class AngleLimits(NamedTuple):
    minimum:float
    maximum:float

class ArcSettings:
    pivot_point:Point3D
    radius:float
    width:float
    u_unit:Vector
    v_unit:Vector

    def __init__(self, pivot_point:Point3D, radius:float, width:float, u_unit:Vector|None = None, v_unit:Vector|None = None):
        from data_structures import Standard3DUnitVectors as STD_UNIT
        
        self.pivot_point = pivot_point
        self.radius = radius
        self.width = width
        self.u_unit = u_unit if (u_unit is not None) else STD_UNIT.X
        self.v_unit = v_unit if (v_unit is not None) else STD_UNIT.Y

class JointAngle:
    start:float  # In Radians
    end:float    # In Radians
    limits:AngleLimits  # In Radians
    total_degrees:int

    def __init__(self, start_angle:float, end_angle:float, limits:AngleLimits):
        self.start  = start_angle
        self.end    = end_angle
        self.limits = AngleLimits(*limits)

        self.total_degrees = int(np.round(self.get_total_angle_in_degrees()))

    def get_total_angle_in_radians(self, angle_start:float|None = None, angle_end:float|None = None) -> float:
        if angle_end is None:
            angle_start = self.start
            angle_end = self.end
        elif angle_start is None:
            angle_start = self.end

        return np.abs(angle_start - angle_end)
    
    def get_total_angle_in_degrees(self, angle_start:float|None = None, angle_end:float|None = None) -> float:
        total_angle_in_radians = self.get_total_angle_in_radians(angle_start, angle_end)
        return np.degrees(total_angle_in_radians)
