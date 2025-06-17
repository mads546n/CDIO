# pc_vision/config.py

# EV3 Wi-Fi connection
EV3_IP = "192.168.43.89"
EV3_PORT = 12345

# Field dimensions (in cm) — adjust to match your real setup
FIELD_WIDTH = 180
FIELD_HEIGHT = 120

# Vision HSV thresholds (to be tuned)
HSV_WHITE_LOWER = (0, 0, 202)
HSV_WHITE_UPPER = (180, 85, 255)

HSV_ORANGE_LOWER = (0, 103, 179)
HSV_ORANGE_UPPER = (98, 255, 255)

HSV_BLUE_LOWER = (37, 0, 0)
HSV_BLUE_UPPER = (107, 255, 255)

HSV_PINK_LOWER = (113, 72, 0)
HSV_PINK_UPPER = (180, 159, 255)

HSV_RED_LOWER = (146, 160, 0)
HSV_RED_UPPER = (180, 255, 255)

# Robot properties (used later for path planning)
ROBOT_RADIUS_CM = 9
WHEEL_BASE_CM = 17

# Control mock mode globally if EV3 is not powered
MOCK_MODE = False
