import numpy as np
import numpy.typing as npt
from typing import Annotated

type Point2D = Annotated[npt.NDArray[np.float64], (2,)]
type Point3D = Annotated[npt.NDArray[np.float64], (3,)]  # (A Coordinate) 1D array with 3 elements
type Vector  = Point3D
type Point3DList = Annotated[npt.NDArray[np.float64], (None, 3)]  # 2D array of 3D points/coordinates
