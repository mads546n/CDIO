# tools/threshold_tuner.py

import cv2
import numpy as np
import re

CONFIG_PATH = r"C:\Users\User\Documents\CDIO\pc_vision\config.py"

def nothing(x):
    pass

def update_config(color_name, lower, upper):
    with open(CONFIG_PATH, "r") as f:
        config_lines = f.readlines()

    def format_tuple(t):
        return f"({t[0]}, {t[1]}, {t[2]})"

    lower_pattern = re.compile(f"HSV_{color_name.upper()}_LOWER\s*=\s*\([^\)]*\)")
    upper_pattern = re.compile(f"HSV_{color_name.upper()}_UPPER\s*=\s*\([^\)]*\)")


    for i, line in enumerate(config_lines):
        if lower_pattern.match(line):
            config_lines[i] = f"HSV_{color_name.upper()}_LOWER = {format_tuple(lower)}\n"
        elif upper_pattern.match(line):
            config_lines[i] = f"HSV_{color_name.upper()}_UPPER = {format_tuple(upper)}\n"

    with open(CONFIG_PATH, "w") as f:
        f.writelines(config_lines)

    print(f"[INFO] updated HSV_{color_name.upper()} thresholds in Config.py")

cv2.namedWindow("Tuner")

for name, max_val in [("H", 180), ("S", 255), ("V", 255)]:
    cv2.createTrackbar(f"{name}_min", "Tuner", 0, max_val, nothing)
    cv2.createTrackbar(f"{name}_max", "Tuner", max_val, max_val, nothing)

# Define color-buttons
COLOR_BUTTONS = {
    "white": (10, 10),
    "orange": (110, 10),
    "blue": (210, 10),
    "pink": (310, 10),
    "red": (410, 10),
}

BUTTON_SIZE = (80, 30)

# Mouse event callback
selected_color = None
def mouse_callback(event, x, y, flags, param):
    global selected_color
    if event == cv2.EVENT_LBUTTONDOWN:
        for color, (bx, by) in COLOR_BUTTONS.items():
            if bx <= x <= bx + BUTTON_SIZE[0] and by <= y <= by + BUTTON_SIZE[1]:
                selected_color = color
                print(f"[INFO] Selected color: {color}")

cv2.setMouseCallback("Tuner", mouse_callback)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

print("Adjust sliders and press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Read values from sliders
    try:
        h_min = cv2.getTrackbarPos("H_min", "Tuner")
        h_max = cv2.getTrackbarPos("H_max", "Tuner")
        s_min = cv2.getTrackbarPos("S_min", "Tuner")
        s_max = cv2.getTrackbarPos("S_max", "Tuner")
        v_min = cv2.getTrackbarPos("V_min", "Tuner")
        v_max = cv2.getTrackbarPos("V_max", "Tuner")
    except cv2.error:
        print("[WARNING] OpenCV window was closed or crashed unexpectedly!")
        break

    lower = (h_min, s_min, v_min)
    upper = (h_max, s_max, v_max)
    mask = cv2.inRange(hsv, lower, upper)

    result = cv2.bitwise_and(frame, frame, mask=mask)
    stacked = np.hstack((frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), result))
    
    # Draw buttons
    for color, (bx, by) in COLOR_BUTTONS.items():
        cv2.rectangle(stacked, (bx, by), (bx + BUTTON_SIZE[0], by + BUTTON_SIZE[1]), (255, 255, 255), -1)
        cv2.putText(stacked, color, (bx + 5, by + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    if selected_color:
        update_config(selected_color, lower, upper)
        cv2.waitKey(500)
        selected_color = None 
    
    cv2.imshow("Tuner", stacked)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(f"HSV LOWER: {lower}")
        print(f"HSV UPPER: {upper}")

cap.release()
cv2.destroyAllWindows()
