# controller/potential_field.py
import math
import numpy as np
from controller.geometry import clampf, wrapToPi, quantize
import config as cfg

EPS = 1e-6

# --- Geofencing Configuration ---
# Perimeter: (0,-10), (0,10), (100,-10), (100,10)
# This forms a rectangle:
GEO_X_MIN = 0.0
GEO_X_MAX = 100.0
GEO_Y_MIN = -10.0
GEO_Y_MAX = 10.0

def compute_total_force_field(x, y, obstacles, target_x, target_y):
    # Attractive force (unit vector to target)
    F_att = np.array([target_x - x, target_y - y], dtype=float)
    dist_to_goal = np.linalg.norm(F_att)
    F_att = F_att / (dist_to_goal + EPS)

    # Repulsive: only closest obstacle within MAX_OBST_DIST
    closest_obs = None
    closest_dist = cfg.MAX_OBST_DIST
    for ox, oy in obstacles:
        d = math.hypot(ox - x, oy - y)
        if d < closest_dist:
            closest_dist = d
            closest_obs = (ox, oy)

    F_rep = np.zeros(2, dtype=float)
    if closest_obs is not None:
        ox, oy = closest_obs
        d_vec = np.array([x - ox, y - oy], dtype=float)
        d = np.linalg.norm(d_vec)
        if d < cfg.MIN_OBST_DIST:
            F_rep = np.array([0.0, 0.0])
        elif d < cfg.MAX_OBST_DIST:
            rep_mag = (cfg.MAX_OBST_DIST - d) / cfg.MAX_OBST_DIST
            rep_mag = clampf(rep_mag, 0.0, 1.0)
            F_rep = d_vec / (d + EPS) * rep_mag

    F_total = cfg.K_ANG_ATTRACT * F_att + cfg.K_REP_ANG * F_rep
    return F_total

def computePotentialFieldAvoidance(
    robot_x, robot_y, robot_yaw,
    target_x, target_y,
    obstacles,
    cur_left_speed, cur_right_speed,
    wheel_base_m
):
    # Safety Check: Out of Bounds
    # If we are already outside, STOP immediately.
    # We add a small buffer (EPS) to avoid floating point flicker at the exact edge.
    if (robot_x < GEO_X_MIN - EPS or robot_x > GEO_X_MAX + EPS or
        robot_y < GEO_Y_MIN - EPS or robot_y > GEO_Y_MAX + EPS):
        # Determine closest point inside for recovery (optional, but good for debugging)
        # For now, we return the stop flag as requested for safety.
        return 0.0, 0.0, True

    # Calculate Wall Repulsive Forces
    F_wall = np.zeros(2, dtype=float)

    # Left Wall (x = 0) - Pushes Right (+x)
    dist_left = robot_x - GEO_X_MIN
    if dist_left < cfg.WALL_INFLUENCE_DIST:
        mag = (cfg.WALL_INFLUENCE_DIST - dist_left) / cfg.WALL_INFLUENCE_DIST
        F_wall[0] += mag * cfg.K_WALL

    # Right Wall (x = 100) - Pushes Left (-x)
    dist_right = GEO_X_MAX - robot_x
    if dist_right < cfg.WALL_INFLUENCE_DIST:
        mag = (cfg.WALL_INFLUENCE_DIST - dist_right) / cfg.WALL_INFLUENCE_DIST
        F_wall[0] -= mag * cfg.K_WALL

    # Bottom Wall (y = -10) - Pushes Up (+y)
    dist_bottom = robot_y - GEO_Y_MIN
    if dist_bottom < cfg.WALL_INFLUENCE_DIST:
        mag = (cfg.WALL_INFLUENCE_DIST - dist_bottom) / cfg.WALL_INFLUENCE_DIST
        F_wall[1] += mag * cfg.K_WALL

    # Top Wall (y = 10) - Pushes Down (-y)
    dist_top = GEO_Y_MAX - robot_y
    if dist_top < cfg.WALL_INFLUENCE_DIST:
        mag = (cfg.WALL_INFLUENCE_DIST - dist_top) / cfg.WALL_INFLUENCE_DIST
        F_wall[1] -= mag * cfg.K_WALL

    # robot half extents
    HF = cfg.ROBOT_LENGTH / 2
    HB = cfg.ROBOT_LENGTH / 2
    HS = cfg.ROBOT_WIDTH / 2

    closest_obs = None
    closest_dist = float("inf")

    for ox, oy in obstacles:
        dx = ox - robot_x
        dy = oy - robot_y
        cy = math.cos(robot_yaw)
        sy = math.sin(robot_yaw)
        xr =  cy * dx + sy * dy
        yr = -sy * dx + cy * dy

        if np.linalg.norm([xr, yr]) > cfg.MAX_OBST_DIST:
            continue
        if xr < 0.0:
            continue

        if xr > HF:
            dxm = xr - HF
        elif xr < -HB:
            dxm = -HB - xr
        else:
            dxm = 0.0

        if yr > HS:
            dym = yr - HS
        elif yr < -HS:
            dym = -HS - yr
        else:
            dym = 0.0

        margin_dist = math.hypot(dxm, dym)

        if margin_dist < closest_dist:
            closest_dist = margin_dist
            closest_obs = (quantize(ox, 0.05), quantize(oy, 0.05))

    # Repulsive force
    F_rep = np.zeros(2, dtype=float)

    if closest_obs is not None:
        obs_x, obs_y = closest_obs
        d_vec = np.array([robot_x - obs_x, robot_y - obs_y], dtype=float)
        d = np.linalg.norm(d_vec)

        if d < cfg.MIN_OBST_DIST:
            # immediate stop situation
            return 0.0, 0.0, True

        if d > EPS and d < cfg.MAX_OBST_DIST:
            dx = obs_x - robot_x
            dy = obs_y - robot_y
            cy = math.cos(robot_yaw)
            sy = math.sin(robot_yaw)
            xr = cy * dx + sy * dy
            yr = -sy * dx + cy * dy
            obs_dist = math.hypot(xr, yr)

            if d < (cfg.MIN_OBST_DIST + cfg.MARGIN):
                return 0.2, -0.2, False

            if obs_dist > EPS and xr >= 0.0:
                rep_mag = (cfg.MAX_OBST_DIST - obs_dist) / cfg.MAX_OBST_DIST
                rep_mag = clampf(rep_mag, 0.0, 1.0)

                F_rep_robot = np.array([xr, yr], dtype=float)
                F_rep_robot = -F_rep_robot / (obs_dist + EPS) * rep_mag

                # rotate back to world
                F_rep = np.array([
                    cy * F_rep_robot[0] - sy * F_rep_robot[1],
                    sy * F_rep_robot[0] + cy * F_rep_robot[1]
                ], dtype=float)

    # Attractive force
    F_att = np.array([target_x - robot_x, target_y - robot_y], dtype=float)
    dist_to_goal = np.linalg.norm(F_att)
    F_att = F_att / (dist_to_goal + EPS)

    # Combine
    F_total = cfg.K_ANG_ATTRACT * F_att + cfg.K_REP_ANG * F_rep

    desired_heading = math.atan2(F_total[1], F_total[0])
    force_magnitude = np.linalg.norm(F_total)

    desired_v = cfg.MIN_LIN_MPS + ((cfg.MAX_LIN_MPS-0.1) - cfg.MIN_LIN_MPS) * min(force_magnitude, 1.0)

    desired_omega = wrapToPi(desired_heading - robot_yaw)
    desired_omega = clampf(desired_omega, -cfg.MAX_ANG_RADS, cfg.MAX_ANG_RADS)

    v_left  = desired_v - desired_omega * (wheel_base_m * 0.5)
    v_right = desired_v + desired_omega * (wheel_base_m * 0.5)

    # Smooth transition
    v_left  = cfg.SMOOTH_ALPHA * v_left  + (1.0 - cfg.SMOOTH_ALPHA) * cur_left_speed
    v_right = cfg.SMOOTH_ALPHA * v_right + (1.0 - cfg.SMOOTH_ALPHA) * cur_right_speed
    v_left  = clampf(v_left, -cfg.MAX_LIN_MPS, cfg.MAX_LIN_MPS)
    v_right = clampf(v_right, -cfg.MAX_LIN_MPS, cfg.MAX_LIN_MPS)

    return v_left, v_right, False
