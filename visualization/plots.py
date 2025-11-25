# visualization/plots.py
import matplotlib.pyplot as plt

def plot_time_series(df):
    fig, axes = plt.subplots(2, 1, figsize=(10,6), sharex=True)

    axes[0].plot(df["t"], df["vL"], label="vL (left)", linewidth=1.5)
    axes[0].plot(df["t"], df["vR"], label="vR (right)", linewidth=1.5)
    axes[0].set_ylabel("wheel speed [m/s]")
    axes[0].grid(True)
    axes[0].legend(loc="upper right")
    axes[0].set_title("Wheel speeds")

    axes[1].plot(df["t"], df["yaw"], label="yaw", linewidth=1.5)
    axes[1].plot(df["t"], df["abs_speed"], label="linear speed", linewidth=1.5)
    axes[1].set_ylabel("yaw [rad], speed [m/s]")
    axes[1].set_xlabel("time [s]")
    axes[1].grid(True)
    axes[1].legend(loc="upper right")
    axes[1].set_title("Heading and absolute speed")

    plt.tight_layout()
    plt.show()
