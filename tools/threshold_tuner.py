# tools/threshold_tuner.py

import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow("Tuner")

# Create trackbars for HSV ranges
cv2.createTrackbar("H_min", "Tuner", 0, 180, nothing)
cv2.createTrackbar("H_max", "Tuner", 180, 180, nothing)
cv2.createTrackbar("S_min", "Tuner", 0, 255, nothing)
cv2.createTrackbar("S_max", "Tuner", 255, 255, nothing)
cv2.createTrackbar("V_min", "Tuner", 0, 255, nothing)
cv2.createTrackbar("V_max", "Tuner", 255, 255, nothing)

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

print("Adjust sliders and press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Read values from sliders
    h_min = cv2.getTrackbarPos("H_min", "Tuner")
    h_max = cv2.getTrackbarPos("H_max", "Tuner")
    s_min = cv2.getTrackbarPos("S_min", "Tuner")
    s_max = cv2.getTrackbarPos("S_max", "Tuner")
    v_min = cv2.getTrackbarPos("V_min", "Tuner")
    v_max = cv2.getTrackbarPos("V_max", "Tuner")

    lower = (h_min, s_min, v_min)
    upper = (h_max, s_max, v_max)
    mask = cv2.inRange(hsv, lower, upper)

    result = cv2.bitwise_and(frame, frame, mask=mask)
    stacked = np.hstack((frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), result))
    cv2.imshow("Tuner", stacked)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(f"HSV LOWER: {lower}")
        print(f"HSV UPPER: {upper}")

cap.release()
cv2.destroyAllWindows()
