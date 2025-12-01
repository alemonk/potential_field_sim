# config.py
# Tunable constants and global parameters

DT = 0.05

# Robot geometry
WHEEL_BASE = 0.72
ROBOT_LENGTH = 1.2
ROBOT_WIDTH = 0.8

# Potential field constants
MARGIN = 0.1
MAX_OBST_DIST = 3.0 + (ROBOT_LENGTH/2)
MIN_OBST_DIST = 1.1 + (ROBOT_LENGTH/2) - MARGIN

# Desired velocity ranges
MAX_LIN_MPS = 0.8
MIN_LIN_MPS = MAX_LIN_MPS / 2.0
MAX_ANG_RADS = 0.8

# Force combination gains
K_ANG_ATTRACT = 1.5
K_REP_ANG = 6.0

# Frontal cone and smoothing parameters
SMOOTH_ALPHA = 0.05
QUANTIZE_STEP = 0.001

# Initial robot state
INIT_X = 0.0
INIT_Y = 0.0
INIT_YAW = 0.0

# Environment: goal location
TARGET = (100.0, 0.0)

# Obstacle generation parameters
X_RANGE = (10.0, 90.0)
Y_RANGE = (-5.0, 5.0)
NUM_CLUSTERS = 10
POINTS_PER_CLUSTER = 30
STD_DEV = 1.5

# Simulation limits
MAX_STEPS = 20000
TARGET_THRESHOLD = 1.0
