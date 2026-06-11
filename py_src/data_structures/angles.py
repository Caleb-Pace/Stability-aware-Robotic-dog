import numpy as np
from data_structures import Point3D, Vector
from data_structures import Standard3DUnitVectors as STD_UNIT
from typing import NamedTuple

class AngleLimits(NamedTuple):
    min:float
    max:float

class ArcSettings:
    pivot_point:Point3D
    radius:float
    u_unit:Vector
    v_unit:Vector

    def __init__(self, pivot_point:Point3D, radius:float, u_unit:Vector|None = None, v_unit:Vector|None = None):
        self.pivot_point = pivot_point
        self.radius = radius
        self.u_unit = u_unit if (u_unit is not None) else STD_UNIT.X
        self.v_unit = v_unit if (v_unit is not None) else STD_UNIT.Y

class JointAngle:
    start:float
    end:float
    limits:AngleLimits
    _total_angle:float  # In Radians
    total_degrees:int

    def __init__(self, start_angle:float, end_angle:float, limits:AngleLimits):
        self.start  = start_angle
        self.end    = end_angle
        self.limits = limits

        self._total_angle  = np.abs(self.start - self.end)
        self.total_degrees = int(np.round(np.degrees(self._total_angle)))
