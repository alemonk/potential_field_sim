# main.py
import matplotlib.pyplot as plt
from simulation import run_simulation
from environment.force_field import build_force_grid
from visualization.animator import make_animation
from visualization.plots import plot_time_series
import config as cfg

def main():
    df, OBSTACLES = run_simulation()

    # Build quiver grid for visualization
    GX, GY, U, V = build_force_grid(
        x_min=0, x_max=100, y_min=-10, y_max=10, nx=100, ny=20,
        obstacles=OBSTACLES, target=cfg.TARGET
    )

    anim = make_animation(df, OBSTACLES, cfg.TARGET, GX, GY, U, V)

    # Show animation in blocking mode
    plt.show()

    # Time series plots
    # plot_time_series(df)

if __name__ == "__main__":
    main()
