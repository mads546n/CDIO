# pc_vision/config.py

# EV3 Wi-Fi connection
EV3_IP = "192.168.43.89"
EV3_PORT = 12345

# Field dimensions (in cm) — adjust to match your real setup
FIELD_WIDTH = 180
FIELD_HEIGHT = 120

# Vision HSV thresholds (to be tuned)
HSV_WHITE_LOWER = (0, 0, 171)
HSV_WHITE_UPPER = (180, 119, 255)

HSV_ORANGE_LOWER = (8, 110, 160)
HSV_ORANGE_UPPER = (89, 255, 255)

HSV_BLUE_LOWER = (35, 49, 181)
HSV_BLUE_UPPER = (106, 255, 255)

HSV_PINK_LOWER = (110, 53, 58)
HSV_PINK_UPPER = (180, 255, 255)

HSV_RED_LOWER = (0, 172, 0)
HSV_RED_UPPER = (86, 255, 235)

# Robot properties (used later for path planning)
ROBOT_RADIUS_CM = 9
WHEEL_BASE_CM = 17

# Control mock mode globally if EV3 is not powered
MOCK_MODE = False
