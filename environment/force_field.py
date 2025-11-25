# environment/force_field.py
import numpy as np
from controller.potential_field import compute_total_force_field

def build_force_grid(x_min, x_max, y_min, y_max, nx, ny, obstacles, target):
    grid_x = np.linspace(x_min, x_max, nx)
    grid_y = np.linspace(y_min, y_max, ny)
    GX, GY = np.meshgrid(grid_x, grid_y)
    U = np.zeros_like(GX)
    V = np.zeros_like(GY)

    for i in range(GX.shape[0]):
        for j in range(GX.shape[1]):
            fx, fy = compute_total_force_field(GX[i, j], GY[i, j], obstacles, target[0], target[1])
            U[i, j] = fx
            V[i, j] = fy

    return GX, GY, U, V
