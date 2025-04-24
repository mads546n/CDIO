import cv2
import numpy as np

# === Setup camera ===
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("❌ Error: Could not access camera")
    exit()

# === Trackbar setup window ===
cv2.namedWindow("Trackbars")

def nothing(x):
    pass

# Create trackbars for HSV range tuning
cv2.createTrackbar("Lower H", "Trackbars", 0, 180, nothing)
cv2.createTrackbar("Lower S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Lower V", "Trackbars", 80, 255, nothing)
cv2.createTrackbar("Upper H", "Trackbars", 180, 180, nothing)
cv2.createTrackbar("Upper S", "Trackbars", 80, 255, nothing)
cv2.createTrackbar("Upper V", "Trackbars", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to read frame")
        break

    height, width, _ = frame.shape

    # --- Apply CLAHE to boost brightness ---
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl, a, b))
    enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    # --- Convert to HSV ---
    hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)

    # === Read values from trackbars ===
    lh = cv2.getTrackbarPos("Lower H", "Trackbars")
    ls = cv2.getTrackbarPos("Lower S", "Trackbars")
    lv = cv2.getTrackbarPos("Lower V", "Trackbars")
    uh = cv2.getTrackbarPos("Upper H", "Trackbars")
    us = cv2.getTrackbarPos("Upper S", "Trackbars")
    uv = cv2.getTrackbarPos("Upper V", "Trackbars")

    lower_hsv = np.array([lh, ls, lv])
    upper_hsv = np.array([uh, us, uv])

    # --- Apply mask based on live-tuned HSV values ---
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # --- Clean up the mask ---
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # --- Find contours ---
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        if area > 100 and perimeter > 0:
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            if 0.7 < circularity < 1.2:  # Roughly circular
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    # Draw and label object
                    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                    cv2.putText(frame, f"Object at ({cx}, {cy})", (cx + 10, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Show windows
    cv2.imshow("Camera", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
