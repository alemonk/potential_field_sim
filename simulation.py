# simulation.py
import pandas as pd
import numpy as np
import config as cfg
from controller.robot import DifferentialDriveRobot
from environment.obstacles import generate_clustered_obstacles

def run_simulation():
    OBSTACLES = generate_clustered_obstacles(
        num_clusters=cfg.NUM_CLUSTERS,
        points_per_cluster=cfg.POINTS_PER_CLUSTER,
        map_x_range=cfg.X_RANGE,
        map_y_range=cfg.Y_RANGE,
        cluster_std_dev=cfg.STD_DEV
    )

    robot = DifferentialDriveRobot(cfg.INIT_X, cfg.INIT_Y, cfg.INIT_YAW, cfg.WHEEL_BASE)

    history = []
    stop_simulation = False
    i = 0
    while (not stop_simulation):
        t = i * cfg.DT
        x, y, yaw, vL, vR, abs_speed, stop_simulation = robot.step(cfg.TARGET, OBSTACLES, cfg.DT)
        history.append({
            "t": t, "x": x, "y": y, "yaw": yaw, "vL": vL, "vR": vR, "abs_speed": abs_speed,
            "target_x": cfg.TARGET[0], "target_y": cfg.TARGET[1]
        })

        dx = cfg.TARGET[0] - x
        dy = cfg.TARGET[1] - y
        d = np.linalg.norm([dx, dy])
        if d < cfg.TARGET_THRESHOLD:
            stop_simulation = True

        i += 1
        if i > cfg.MAX_STEPS:
            print("Stopping simulation: max steps reached")
            break

    df = pd.DataFrame(history)
    return df, OBSTACLES
