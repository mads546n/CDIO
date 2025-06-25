# pc_vision/config.py

# EV3 Wi-Fi connection
EV3_IP = "192.168.176.92"
EV3_PORT = 12345

# Field dimensions (in cm) — adjust to match your real setup
FIELD_WIDTH = 180
FIELD_HEIGHT = 120

# Vision HSV thresholds (to be tuned)
HSV_WHITE_LOWER = (0, 0, 204)
HSV_WHITE_UPPER = (180, 71, 255)

HSV_ORANGE_LOWER = (9, 71, 190)
HSV_ORANGE_UPPER = (48, 255, 255)

HSV_BLUE_LOWER = (82, 39, 158)
HSV_BLUE_UPPER = (162, 90, 255)

HSV_PINK_LOWER = (31, 78, 164)
HSV_PINK_UPPER = (148, 172, 255)

HSV_RED_LOWER = (0, 127, 133)
HSV_RED_UPPER = (180, 255, 255)

# Robot properties (used later for path planning)
ROBOT_RADIUS_CM = 9
WHEEL_BASE_CM = 17

# Safe area
WALL_MARGIN_PX = 50

# Control mock mode globally if EV3 is not powered
MOCK_MODE = False

TIMER_RESET = 80
