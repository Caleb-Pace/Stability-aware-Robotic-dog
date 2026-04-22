import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt

# Show plot
# TODO: Rename
def show_trajectory():
    ax = plt.figure().add_subplot(projection='3d')

    # t = np.linspace(-4 * np.pi, 4 * np.pi, 100)
    # t = np.linspace(0, 10, 100)
    # tx = np.array((np.pi / 2), dtype=np.float64)
    # tx = np.linspace(0, 10, 100)
    # ty = np.array((np.pi / 2), dtype=np.float64)
    # ty = np.linspace(10, 0, 100)
    # tz = np.linspace(10, 0, 10)

    # tx = np.linspace(0, 10, 10)
    # ty = tx**2
    # tz = tx**3

    t = np.linspace(0, 2*np.pi, 32)  # Nodes
    tx = 2 * np.cos(t)
    ty = 2 * np.sin(t)
    tz = (10/5) - (2/5)*(2 * np.cos(t)) - (3/5)*(2 * np.sin(t))

    draw_parametric_function(ax, tx, ty, tz, 'foot trajectory')

    ax.legend()
    plt.show()

def draw_parametric_function(ax, tx:npt.NDArray[np.float64], ty:npt.NDArray[np.float64], tz:npt.NDArray[np.float64], label:str):

    # Prepare arrays x, y, z
    # theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
    # theta = t
    # z = np.linspace(-2, 2, 10)
    # # r = z**2 + 1
    # x = tx * np.sin(tx)
    # y = ty * np.cos(ty)
    x = tx
    y = ty
    z = tz

    get_points(x, y, z)

    # Draw parametric curve
    ax.plot(x, y, z, label=label)

def get_points(x:npt.NDArray[np.float64], y:npt.NDArray[np.float64], z:npt.NDArray[np.float64]):
    max_len = max(x.size, y.size, z.size)

    x_repeated = np.resize(x, max_len)
    y_repeated = np.resize(y, max_len)
    z_repeated = np.resize(z, max_len)

    points = np.column_stack((x_repeated, y_repeated, z_repeated))
    print(repr(points))
    print(f"{len(points)} points")