# visualization/animator.py
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle, Wedge
from matplotlib.transforms import Affine2D
import config as cfg

def make_animation(df, obstacles, waypoints, GX, GY, U, V):
    fig, ax = plt.subplots(figsize=(12, 12))

    ax.scatter([o[0] for o in obstacles], [o[1] for o in obstacles],
               s=50, marker="o", color="black", label="obstacles")
    
    # Plot all waypoints
    ax.scatter([w[0] for w in waypoints], [w[1] for w in waypoints],
               s=120, marker="*", color="green", label="waypoints", zorder=10)
    
    # Draw lines connecting waypoints to show the path
    waypoint_xs = [w[0] for w in waypoints]
    waypoint_ys = [w[1] for w in waypoints]
    ax.plot(waypoint_xs, waypoint_ys, 'g--', alpha=0.5, linewidth=1, label="planned path")

    path_line, = ax.plot([], [], linewidth=1, color="red")

    # Robot rectangle (centered at origin before transform)
    L = cfg.ROBOT_LENGTH
    W = cfg.ROBOT_WIDTH
    robot_rect = Rectangle((-L/2, -W/2), L, W, fill=True, alpha=0.6, zorder=6, color="red")
    ax.add_patch(robot_rect)

    # theta1=-90, theta2=90 creates a semicircle facing forward along positive x in robot frame
    semicircle = Wedge((0, 0), cfg.MAX_OBST_DIST, -90, 90, facecolor="blue", alpha=0.1, zorder=4)
    ax.add_patch(semicircle)

    # Draw square boundary walls
    wall_thickness = 0.5
    size = cfg.SQUARE_SIZE
    perimeter_top = Rectangle((0, size), size, wall_thickness, fill=True, alpha=1.0, zorder=6, color="black")
    perimeter_bottom = Rectangle((0, 0), size, -wall_thickness, fill=True, alpha=1.0, zorder=6, color="black")
    perimeter_left = Rectangle((0, 0), -wall_thickness, size, fill=True, alpha=1.0, zorder=6, color="black")
    perimeter_right = Rectangle((size, 0), wall_thickness, size, fill=True, alpha=1.0, zorder=6, color="black")
    ax.add_patch(perimeter_top)
    ax.add_patch(perimeter_bottom)
    ax.add_patch(perimeter_left)
    ax.add_patch(perimeter_right)

    ax.set_xlim(-2, size + 2)
    ax.set_ylim(-2, size + 2)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Potential Field Obstacle Avoidance - Lawn Mower Pattern")

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
