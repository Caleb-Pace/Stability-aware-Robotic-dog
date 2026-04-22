import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt

# Show plot
# TODO: Rename
def show_trajectory():
    ax = plt.figure().add_subplot(projection='3d')

    node_count = 32
    t = np.linspace(0, 2*np.pi, node_count)  # Nodes
    tx = 2 * np.cos(t)
    ty = 2 * np.sin(t)
    tz = (10/5) - (2/5)*(2 * np.cos(t)) - (3/5)*(2 * np.sin(t))

    draw_parametric_function(ax, tx, ty, tz, 'foot trajectory')
    plot_points(ax, tx, ty, tz)

    ax.legend()
    plt.show()

def draw_parametric_function(ax, tx:npt.NDArray[np.float64], ty:npt.NDArray[np.float64], tz:npt.NDArray[np.float64], label:str):

    # Draw parametric curve
    ax.plot(tx, ty, tz, label=label)

def plot_points(ax, x:npt.NDArray[np.float64], y:npt.NDArray[np.float64], z:npt.NDArray[np.float64]):
    points = get_points(x, y, z)
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', label='Points')

def get_points(x:npt.NDArray[np.float64], y:npt.NDArray[np.float64], z:npt.NDArray[np.float64]):
    max_len = max(x.size, y.size, z.size)

    x_repeated = np.resize(x, max_len)
    y_repeated = np.resize(y, max_len)
    z_repeated = np.resize(z, max_len)

    points = np.column_stack((x_repeated, y_repeated, z_repeated))
    print(repr(points))
    print(f"{len(points)} points")
    
    return points