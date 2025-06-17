# pc_vision/config.py

# EV3 Wi-Fi connection
EV3_IP = "192.168.43.89"
EV3_PORT = 12345

# Field dimensions (in cm) — adjust to match your real setup
FIELD_WIDTH = 180
FIELD_HEIGHT = 120

# Vision HSV thresholds (to be tuned)
HSV_WHITE_LOWER = (0, 0, 226)
HSV_WHITE_UPPER = (180, 78, 255)

HSV_ORANGE_LOWER = (0, 48, 225)
HSV_ORANGE_UPPER = (59, 255, 255)

HSV_BLUE_LOWER = (49, 28, 131)
HSV_BLUE_UPPER = (150, 254, 255)

HSV_PINK_LOWER = (38, 84, 145)
HSV_PINK_UPPER = (180, 175, 255)

HSV_RED_LOWER = (38, 140, 87)
HSV_RED_UPPER = (180, 255, 255)

# Robot properties (used later for path planning)
ROBOT_RADIUS_CM = 9
WHEEL_BASE_CM = 17

# Control mock mode globally if EV3 is not powered
MOCK_MODE = False
