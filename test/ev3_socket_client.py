import cv2
import numpy as np
import socket
import math

def angle_between(v1, v2):
    # Dot product and magnitude
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.hypot(v1[0], v1[1])
    mag2 = math.hypot(v2[0], v2[1])

    if mag1 == 0 or mag2 == 0:
        return None  # Can't calculate angle with zero-length vector

    cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))  # Clamp for safety
    angle_rad = math.acos(cos_angle)
    angle_deg = math.degrees(angle_rad)
    return angle_deg

def signed_angle(v1, v2):
    cross = v1[0]*v2[1] - v1[1]*v2[0]
    angle = angle_between(v1, v2)
    if angle is not None:
        return -angle if cross < 0 else angle
    return 0

def find_largest_circular_object(contours, frame, draw_color=(0, 255, 0)):
    largest_area = 0
    cx, cy = 0, 0

    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        if area > 100 and perimeter > 0:
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            if 0.7 < circularity < 1.2:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    current_cx = int(M['m10'] / M['m00'])
                    current_cy = int(M['m01'] / M['m00'])

                    if area > largest_area:
                        largest_area = area
                        cx, cy = current_cx, current_cy

                    # Draw all valid circles
                    cv2.circle(frame, (current_cx, current_cy), 10, draw_color, -1)
                    cv2.putText(frame, f"Object at ({current_cx}, {current_cy})",
                                (current_cx + 10, current_cy), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, draw_color, 2)

    return frame, cx, cy

def find_closest_object((rx, ry), contours, frame, draw_color=(0, 255, 0)):
    largest_distance = 0
    cx, cy = 0, 0

    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        if area > 100 and perimeter > 0:
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            if 0.7 < circularity < 1.2:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    current_cx = int(M['m10'] / M['m00'])
                    current_cy = int(M['m01'] / M['m00'])

                    # Draw all valid circles
                    cv2.circle(frame, (current_cx, current_cy), 10, draw_color, -1)
                    cv2.putText(frame, f"Object at ({current_cx}, {current_cy})",
                                (current_cx + 10, current_cy), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, draw_color, 2)

                    d = sqrt(pow(rx - current_cx, 2) + (ry - current_cy, 2))
                    if d > largest_distance:
                        cx, cy = current_cx, current_cy

    return frame, cx, cy

# === EV3 Socket Setup ===
EV3_IP = '192.168.43.89'  # Update this if needed
PORT = 12345

#sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#sock.connect((EV3_IP, PORT))
print("✅ Connected to EV3")

# === Camera Setup ===
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Error: Could not access camera")
    exit()

# === Create HSV tuning trackbars ===
cv2.namedWindow("Trackbars")

def nothing(x): pass

cv2.createTrackbar("Lower H", "Trackbars", 0, 180, nothing)
cv2.createTrackbar("Lower S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Lower V", "Trackbars", 80, 255, nothing)
cv2.createTrackbar("Upper H", "Trackbars", 180, 180, nothing)
cv2.createTrackbar("Upper S", "Trackbars", 80, 255, nothing)
cv2.createTrackbar("Upper V", "Trackbars", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Frame read failed.")
        break

    height, width, _ = frame.shape
    center_x = width // 2

    # === Preprocessing ===
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl, a, b))
    enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)

    # === Read HSV range from trackbars ===
    lh = cv2.getTrackbarPos("Lower H", "Trackbars")
    ls = cv2.getTrackbarPos("Lower S", "Trackbars")
    lv = cv2.getTrackbarPos("Lower V", "Trackbars")
    uh = cv2.getTrackbarPos("Upper H", "Trackbars")
    us = cv2.getTrackbarPos("Upper S", "Trackbars")
    uv = cv2.getTrackbarPos("Upper V", "Trackbars")

    lower_hsv = np.array([lh, ls, lv])
    upper_hsv = np.array([uh, us, uv])

    # Define HSV ranges
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([85, 255, 255])

    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # === Apply Mask ===
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # === Find Contours ===
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    frame, fx, fy = find_largest_circular_object(contours_green, frame (255, 0, 255))
    frame, bx, by = find_largest_circular_object(contours_blue, frame (255, 255, 0))

    rx = min(fx, bx) + abs(fx - bx) // 2
    ry = min(fy, by) + abs(fy - by) // 2

    cv2.circle(frame, (rx, ry), 10, (0, 255, 0), -1)
    cv2.putText(frame, f"Robot at ({rx}, {ry})",
                (rx + 10, ry), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)

    orientation = (fx - bx, by - fy)

    frame, cx, cy = find_closest_object((rx, ry), contours, frame, (0, 0, 0))

    direction = (cx - rx, ry - cy)

    angle = signed_angle(orientation, direction)

    # === Robot Control Decision ===
    if angle is not None:
        print(angle)
        if angle = 0:
           sock.sendall(b'F')
        elif angle > 0:
            sock.sendall(b'L')
        elif angle < 0:
            sock.sendall(b'R')
    else:
        sock.sendall(b'S')

    # === Show Camera + Mask Output ===
    cv2.imshow("Camera", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        sock.sendall(b'S')
        break

# === Cleanup ===
cap.release()
cv2.destroyAllWindows()
sock.close()
print("✅ Disconnected from EV3")
