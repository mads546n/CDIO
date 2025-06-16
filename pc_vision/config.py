# pc_vision/config.py

# EV3 Wi-Fi connection
EV3_IP = "192.168.43.89"
EV3_PORT = 12345

# Field dimensions (in cm) — adjust to match your real setup
FIELD_WIDTH = 180
FIELD_HEIGHT = 120

# Vision HSV thresholds (to be tuned)
HSV_WHITE_LOWER = (0, 0, 186)
HSV_WHITE_UPPER = (180, 59, 255)

HSV_ORANGE_LOWER = (0, 74, 215)
HSV_ORANGE_UPPER = (102, 255, 255)

HSV_BLUE_LOWER = (26, 27, 151)
HSV_BLUE_UPPER = (169, 255, 255)

HSV_PINK_LOWER = (163, 109, 203)
HSV_PINK_UPPER = (172, 255, 254)

HSV_RED_LOWER = (30, 152, 111)
HSV_RED_UPPER = (180, 255, 255)

# Robot properties (used later for path planning)
ROBOT_RADIUS_CM = 9
WHEEL_BASE_CM = 17

# Control mock mode globally if EV3 is not powered
MOCK_MODE = False
