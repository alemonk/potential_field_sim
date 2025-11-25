# environment/obstacles.py
import random
import numpy as np

def generate_clustered_obstacles(num_clusters, points_per_cluster, map_x_range, map_y_range, cluster_std_dev):
    all_obstacles = []
    min_x, max_x = map_x_range
    min_y, max_y = map_y_range

    for _ in range(num_clusters):
        center_x = random.uniform(min_x, max_x)
        center_y = random.uniform(min_y, max_y)

        for _ in range(points_per_cluster):
            obstacle_x = np.random.normal(loc=center_x, scale=cluster_std_dev)
            obstacle_y = np.random.normal(loc=center_y, scale=cluster_std_dev)
            all_obstacles.append((obstacle_x, obstacle_y))

    return all_obstacles
