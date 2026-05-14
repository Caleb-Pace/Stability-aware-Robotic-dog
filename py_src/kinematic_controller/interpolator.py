import numpy as np
import numpy.typing as npt
from interpolation import Interpolator, CatmullRomSpline
from kinematic_controller.gait_definition import Gait, LEG_COUNT

# TODO: move function or rename file
def get_foot_trajectories(gait:Gait, node_count:int) -> npt.NDArray[np.float64]:
    interpolator:Interpolator = CatmullRomSpline()

    foot_trajectories = np.empty((LEG_COUNT, node_count, 3), np.float64)  # Leg, Node, Coordinate

    for i in range(len(gait.leg_control_points)):
        control_points = gait.leg_control_points[i]
        foot_trajectories[i] = interpolator.compute_interpolated_points(control_points, node_count)

    return foot_trajectories