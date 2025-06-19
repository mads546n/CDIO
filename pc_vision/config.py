# pc_vision/config.py

# EV3 Wi-Fi connection
EV3_IP = "192.168.63.92"
EV3_PORT = 12345

# Field dimensions (in cm) — adjust to match your real setup
FIELD_WIDTH = 180
FIELD_HEIGHT = 120

# Vision HSV thresholds (to be tuned)
HSV_WHITE_LOWER = (0, 0, 203)
HSV_WHITE_UPPER = (180, 113, 255)

HSV_ORANGE_LOWER = (7, 137, 167)
HSV_ORANGE_UPPER = (109, 255, 255)

HSV_BLUE_LOWER = (56, 0, 0)
HSV_BLUE_UPPER = (124, 255, 255)

HSV_PINK_LOWER = (0, 69, 196)
HSV_PINK_UPPER = (62, 144, 255)

HSV_RED_LOWER = (45, 95, 0)
HSV_RED_UPPER = (180, 255, 255)

# Robot properties (used later for path planning)
ROBOT_RADIUS_CM = 9
WHEEL_BASE_CM = 17

# Safe area
WALL_MARGIN_PX = 50

# Control mock mode globally if EV3 is not powered
MOCK_MODE = False
