import numpy as np
import numpy.typing as npt
from typing import Annotated

type Point3D = Annotated[npt.NDArray[np.float64], (3,)]         # (A Coordinate) 1D array with 3 elements
type PointList = Annotated[npt.NDArray[np.float64], (None, 3)]  # 2D array of points/coordinates
