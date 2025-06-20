# pc_vision/config.py

# EV3 Wi-Fi connection
EV3_IP = "192.168.63.92"
EV3_PORT = 12345

# Field dimensions (in cm) — adjust to match your real setup
FIELD_WIDTH = 180
FIELD_HEIGHT = 120

# Vision HSV thresholds (to be tuned)
HSV_WHITE_LOWER = (0, 0, 209)
HSV_WHITE_UPPER = (180, 75, 255)

HSV_ORANGE_LOWER = (0, 75, 0)
HSV_ORANGE_UPPER = (96, 255, 255)

HSV_BLUE_LOWER = (74, 12, 109)
HSV_BLUE_UPPER = (158, 255, 255)

HSV_PINK_LOWER = (0, 59, 209)
HSV_PINK_UPPER = (180, 128, 255)

HSV_RED_LOWER = (0, 80, 0)
HSV_RED_UPPER = (180, 255, 255)

# Robot properties (used later for path planning)
ROBOT_RADIUS_CM = 9
WHEEL_BASE_CM = 17

# Safe area
WALL_MARGIN_PX = 50

# Control mock mode globally if EV3 is not powered
MOCK_MODE = False

TIMER_RESET = 80
