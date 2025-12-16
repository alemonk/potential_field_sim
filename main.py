# main.py
import matplotlib.pyplot as plt
from simulation import run_simulation
from environment.force_field import build_force_grid
from visualization.animator import make_animation
from visualization.plots import plot_time_series
import config as cfg

def main():
    df, OBSTACLES, WAYPOINTS = run_simulation()

    # Build quiver grid for visualization (use first waypoint for force field)
    GX, GY, U, V = build_force_grid(
        x_min=0, x_max=cfg.SQUARE_SIZE, y_min=0, y_max=cfg.SQUARE_SIZE, 
        nx=int(cfg.SQUARE_SIZE), ny=int(cfg.SQUARE_SIZE),
        obstacles=OBSTACLES, target=WAYPOINTS[0]
    )

    anim = make_animation(df, OBSTACLES, WAYPOINTS, GX, GY, U, V)

    # Show animation in blocking mode
    plt.show()

    # Time series plots
    # plot_time_series(df)

if __name__ == "__main__":
    main()
