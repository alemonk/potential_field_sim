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
SMOOTH_ALPHA = 0.25
QUANTIZE_STEP = 1e-5

# Initial robot state
INIT_X = 5.0
INIT_Y = 5.0
INIT_YAW = 0.0

# Environment: goal location
ORIGIN = (5.0, 5.0)

# Lawn mower pattern parameters
SQUARE_SIZE = 50.0
LAWN_MOWER_STEP = 5.0  # Vertical distance between rows
MARGIN_FROM_EDGE = 5.0  # Keep waypoints away from edges

# Obstacle generation parameters
X_RANGE = (10.0, 40.0)
Y_RANGE = (10.0, 40.0)  # Changed to square region
NUM_CLUSTERS = 15
POINTS_PER_CLUSTER = 30
STD_DEV = 0.3

# Wall Parameters
WALL_INFLUENCE_DIST = 0.5   # (Meters) Start feeling the wall force this far away
K_WALL = 500.0              # Gain: Make walls stronger than normal obstacles

# Simulation limits
MAX_STEPS = 20000
TARGET_THRESHOLD = 1.0
