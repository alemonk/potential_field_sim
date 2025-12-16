# simulation.py
import pandas as pd
import numpy as np
import config as cfg
from controller.robot import DifferentialDriveRobot
from environment.obstacles import generate_clustered_obstacles

def generate_lawn_mower_waypoints():
    """Generate vertical lawn mower pattern with waypoints at each 90° turn."""
    waypoints = []
    x = cfg.MARGIN_FROM_EDGE  # Start at left
    going_up = True
    
    while x <= cfg.SQUARE_SIZE - cfg.MARGIN_FROM_EDGE:
        if going_up:
            # Go up to top
            waypoints.append((x, cfg.SQUARE_SIZE - cfg.MARGIN_FROM_EDGE))
            # Move right
            x += cfg.LAWN_MOWER_STEP
            if x <= cfg.SQUARE_SIZE - cfg.MARGIN_FROM_EDGE:
                waypoints.append((x, cfg.SQUARE_SIZE - cfg.MARGIN_FROM_EDGE))
        else:
            # Go down to bottom
            waypoints.append((x, cfg.MARGIN_FROM_EDGE))
            # Move right
            x += cfg.LAWN_MOWER_STEP
            if x <= cfg.SQUARE_SIZE - cfg.MARGIN_FROM_EDGE:
                waypoints.append((x, cfg.MARGIN_FROM_EDGE))
        
        going_up = not going_up
    
    return waypoints

def run_simulation():
    OBSTACLES = generate_clustered_obstacles(
        num_clusters=cfg.NUM_CLUSTERS,
        points_per_cluster=cfg.POINTS_PER_CLUSTER,
        map_x_range=cfg.X_RANGE,
        map_y_range=cfg.Y_RANGE,
        cluster_std_dev=cfg.STD_DEV
    )

    WAYPOINTS = generate_lawn_mower_waypoints()
    print(f"Generated {len(WAYPOINTS)} waypoints for lawn mower pattern")
    
    robot = DifferentialDriveRobot(cfg.INIT_X, cfg.INIT_Y, cfg.INIT_YAW, cfg.WHEEL_BASE)

    history = []
    stop_simulation = False
    current_waypoint_idx = 0
    current_target = WAYPOINTS[current_waypoint_idx]
    prev_target = (cfg.INIT_X, cfg.INIT_Y)  # Start from initial position
    
    i = 0
    while (not stop_simulation):
        t = i * cfg.DT
        x, y, yaw, vL, vR, abs_speed, stop_simulation = robot.step(current_target, prev_target, OBSTACLES, cfg.DT)
        history.append({
            "t": t, "x": x, "y": y, "yaw": yaw, "vL": vL, "vR": vR, "abs_speed": abs_speed,
            "target_x": current_target[0], "target_y": current_target[1]
        })

        dx = current_target[0] - x
        dy = current_target[1] - y
        d = np.linalg.norm([dx, dy])
        if d < cfg.TARGET_THRESHOLD:
            print(f"Reached waypoint {current_waypoint_idx + 1}/{len(WAYPOINTS)}: {current_target}")
            prev_target = current_target  # Current becomes previous
            current_waypoint_idx += 1
            if current_waypoint_idx >= len(WAYPOINTS):
                print("All waypoints reached!")
                stop_simulation = True
            else:
                current_target = WAYPOINTS[current_waypoint_idx]

        i += 1
        if i > cfg.MAX_STEPS:
            print("Stopping simulation: max steps reached")
            break

    df = pd.DataFrame(history)
    return df, OBSTACLES, WAYPOINTS
