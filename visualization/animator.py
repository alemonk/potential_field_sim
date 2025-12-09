# visualization/animator.py
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle, Wedge
from matplotlib.transforms import Affine2D
import config as cfg

def make_animation(df, obstacles, target, GX, GY, U, V):
    fig, ax = plt.subplots(figsize=(16, 8))

    ax.scatter([o[0] for o in obstacles], [o[1] for o in obstacles],
               s=50, marker="o", color="black", label="obstacles")
    ax.scatter([target[0]], [target[1]], s=120, marker="*", color="green", label="target")

    path_line, = ax.plot([], [], linewidth=1, color="red")

    # Robot rectangle (centered at origin before transform)
    L = cfg.ROBOT_LENGTH
    W = cfg.ROBOT_WIDTH
    robot_rect = Rectangle((-L/2, -W/2), L, W, fill=True, alpha=0.6, zorder=6, color="red")
    ax.add_patch(robot_rect)

    # theta1=-90, theta2=90 creates a semicircle facing forward along positive x in robot frame
    semicircle = Wedge((0, 0), cfg.MAX_OBST_DIST, -90, 90, facecolor="blue", alpha=0.1, zorder=4)
    ax.add_patch(semicircle)

    ax.set_xlim(-5, 105)
    ax.set_ylim(-30, 30)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Potential Field Obstacle Avoidance - Animation")

    ax.quiver(GX, GY, U, V, alpha=0.0, scale=100, color="blue")

    def update(frame):
        # update path
        xs = df["x"][:frame]
        ys = df["y"][:frame]
        path_line.set_data(xs, ys)

        # current robot pose
        x = float(df["x"].iloc[frame])
        y = float(df["y"].iloc[frame])
        yaw = float(df["yaw"].iloc[frame])

        # Transform: rotate about origin then translate to (x,y)
        trans = Affine2D().rotate_around(0.0, 0.0, yaw).translate(x, y) + ax.transData

        # apply transform to robot rectangle and semicircle
        semicircle.set_transform(trans)
        robot_rect.set_transform(trans)

        # return all artists that changed (required for blitting)
        return (path_line, robot_rect, semicircle)

    anim = FuncAnimation(fig, update, frames=range(0, len(df), 10), interval=40, blit=True)
    return anim
