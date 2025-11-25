# config.py
# Tunable constants and global parameters

DT = 0.05

# Robot geometry
WHEEL_BASE = 0.72
ROBOT_LENGTH = 1.2
ROBOT_WIDTH = 0.8

# Potential field constants
MARGIN = 0.3
MAX_FRONT_OBST_DIST = 3.5 + (ROBOT_LENGTH/2)
MAX_BACK_OBST_DIST = 2.0 + (ROBOT_LENGTH/2)
MAX_LATERAL_OBST_DIST = 2.0 + (ROBOT_WIDTH/2)
MIN_OBST_DIST = 1.1 - MARGIN + (ROBOT_WIDTH/2)

# Desired velocity ranges
MIN_LIN_MPS = 0.25
MAX_LIN_MPS = 1.0
MAX_ANG_RADS = 0.8

# Force combination gains
K_ANG_ATTRACT = 1.0
K_REP_ANG = 8.0

# Frontal cone and smoothing parameters
FRONTAL_COS_CUTOFF = 0.5
SMOOTH_ALPHA = 0.1
QUANTIZE_STEP = 0.01

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
STD_DEV = 2.0

# Simulation limits
MAX_STEPS = 20000
TARGET_THRESHOLD = 1.0
