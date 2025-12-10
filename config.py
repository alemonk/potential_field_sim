# config.py
# Tunable constants and global parameters

DT = 0.05

# Robot geometry
WHEEL_BASE = 0.72
ROBOT_LENGTH = 2.2
ROBOT_WIDTH = 0.8

# Potential field constants
MARGIN = 0.1
MAX_OBST_DIST = 4.5
MIN_OBST_DIST = 1.1 + (ROBOT_LENGTH/2) - MARGIN

# Desired velocity ranges
MAX_LIN_MPS = 0.8
MIN_LIN_MPS = 0.4
MAX_ANG_RADS = 0.4

# Force combination gains
K_ANG_ATTRACT = 1.5
K_REP_ANG = 10.0
K_PATH = 0.3

# Frontal cone and smoothing parameters
SMOOTH_ALPHA = 0.9
QUANTIZE_STEP = 1e-5

# Initial robot state
INIT_X = 0.0
INIT_Y = 0.0
INIT_YAW = 0.0

# Environment: goal location
ORIGIN = (0.0, 0.0)
TARGET = (100.0, 0.0)

# Obstacle generation parameters
X_RANGE = (10.0, 90.0)
Y_RANGE = (-4.0, 4.0)
NUM_CLUSTERS = 10
POINTS_PER_CLUSTER = 20
STD_DEV = 1.0

# Wall Parameters
WALL_INFLUENCE_DIST = 0.5   # (Meters) Start feeling the wall force this far away
K_WALL = 500.0              # Gain: Make walls stronger than normal obstacles

# Simulation limits
MAX_STEPS = 20000
TARGET_THRESHOLD = 1.0
