# pc_vision/config.py

# EV3 Wi-Fi connection
EV3_IP = "192.168.63.92"
EV3_PORT = 12345

# Field dimensions (in cm) — adjust to match your real setup
FIELD_WIDTH = 180
FIELD_HEIGHT = 120

# Vision HSV thresholds (to be tuned)
HSV_WHITE_LOWER = (0, 0, 187)
HSV_WHITE_UPPER = (180, 101, 255)

HSV_ORANGE_LOWER = (0, 173, 0)
HSV_ORANGE_UPPER = (84, 255, 255)

HSV_BLUE_LOWER = (58, 0, 0)
HSV_BLUE_UPPER = (115, 255, 255)

HSV_PINK_LOWER = (0, 52, 193)
HSV_PINK_UPPER = (180, 102, 255)

HSV_RED_LOWER = (25, 100, 0)
HSV_RED_UPPER = (180, 255, 255)

# Robot properties (used later for path planning)
ROBOT_RADIUS_CM = 9
WHEEL_BASE_CM = 17

# Safe area
WALL_MARGIN_PX = 50

# Control mock mode globally if EV3 is not powered
MOCK_MODE = False
