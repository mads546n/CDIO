
import sys
from pc_vision.config import (
    HSV_WHITE_LOWER, HSV_WHITE_UPPER,
    HSV_ORANGE_LOWER, HSV_ORANGE_UPPER,
    HSV_BLUE_LOWER, HSV_BLUE_UPPER,
    HSV_PINK_LOWER, HSV_PINK_UPPER,
    HSV_RED_LOWER, HSV_RED_UPPER
)

import cv2
import numpy as np
import math

class VisionSystem:
    def __init__(self, camera_index=1):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open webcam.")

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera initialized: {self.frame_width}x{self.frame_height}")

        # Setup CLAHE (adaptive contrast enhancement)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Morph kernel
        self.kernel = np.ones((5, 5), np.uint8)

        # Ball tracking state
        self.prev_ball_ids = {}
        self.next_id = 0

    def detect_state(self, show_debug=True):
        ret, frame = self.cap.read()
        if not ret:
            return [], None

        balls = self.detect_balls(frame)
        robot_pos = self.detect_robot(frame)
        eggs = self.detect_eggs(frame, draw_debug=show_debug)


        if show_debug:
            for ball_id, (x, y, is_vip) in balls.items():
                color = (0, 140, 255) if is_vip else (255, 255, 255)
                cv2.circle(frame, (x, y), 10, color, 2)
                cv2.putText(frame, str(ball_id), (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            if robot_pos:
                (x, y), (dx, dy) = robot_pos
                cv2.circle(frame, (x, y), 12, (0, 255, 0), -1)
                cv2.putText(frame, "Robot", (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.arrowedLine(frame, (x, y), (int(x + dx * 2), int(y + dy * 2)), (255, 0, 255), 2)

            self.detect_walls(frame, draw_debug=show_debug)
            cv2.imshow("GolfBot Vision", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit()

        return list(balls.values()), robot_pos, eggs

    def preprocess_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = self.clahe.apply(v)
        hsv_clahe = cv2.merge([h, s, v])
        return hsv_clahe

    def clean_mask(self, mask):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask

    def detect_walls(self, frame, draw_debug=False):
        hsv = self.preprocess_frame(frame)

        red_mask = self.clean_mask(cv2.inRange(hsv, HSV_RED_LOWER, HSV_RED_UPPER))

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        wall_boxes = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:  # Tune this threshold based on wall size
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            wall_boxes.append((x, y, w, h))

            if draw_debug:
                cv2.rectangle(frame, (x,y),(x+w,y+h), (0, 255, 0), 2)
                cv2.putText(frame, "Wall", (x,y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return wall_boxes

    def detect_eggs(self, frame, draw_debug=False):
        hsv = self.preprocess_frame(frame)

        white_mask = self.clean_mask(cv2.inRange(hsv, HSV_WHITE_LOWER, HSV_WHITE_UPPER))
        eggs = []

        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 80:  # Tune this
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            circularity = 4 * math.pi * area / (perimeter ** 2)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h if h != 0 else 0

            # Heuristic: eggs = more elongated or less circular
            if circularity < 0.65 and (aspect_ratio < 0.75 or aspect_ratio > 1.3):
                cx, cy = x + w // 2, y + h // 2
                eggs.append((cx, cy, w, h))

                if draw_debug:
                    cv2.ellipse(frame, (cx, cy), (w // 2, h // 2), 0, 0, 360, (200, 0, 255), 2)
                    cv2.putText(frame, "Egg", (cx - 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 255), 1)

        return eggs

    def detect_balls(self, frame):
        hsv = self.preprocess_frame(frame)

        white_mask = self.clean_mask(cv2.inRange(hsv, HSV_WHITE_LOWER, HSV_WHITE_UPPER))
        orange_mask = self.clean_mask(cv2.inRange(hsv, HSV_ORANGE_LOWER, HSV_ORANGE_UPPER))

        raw_detections = []
        for mask, is_vip in [(white_mask, False), (orange_mask, True)]:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 50:
                    continue
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                circularity = 4 * math.pi * area / (perimeter ** 2)
                if circularity < 0.6:
                    continue
                (x, y), _ = cv2.minEnclosingCircle(cnt)
                raw_detections.append((int(x), int(y), is_vip))

        return self.assign_ball_ids(raw_detections)



    def assign_ball_ids(self, current_detections):
        new_ids = {}
        used_prev_ids = set()

        for x, y, is_vip in current_detections:
            min_dist = float('inf')
            matched_id = None
            for ball_id, (px, py, _) in self.prev_ball_ids.items():
                if ball_id in used_prev_ids:
                    continue
                dist = math.hypot(x - px, y - py)
                if dist < min_dist and dist < 40:
                    min_dist = dist
                    matched_id = ball_id

            if matched_id is not None:
                new_ids[matched_id] = (x, y, is_vip)
                used_prev_ids.add(matched_id)
            else:
                new_ids[self.next_id] = (x, y, is_vip)
                self.next_id += 1

        self.prev_ball_ids = new_ids.copy()
        return new_ids


    def detect_robot(self, frame):
        hsv = self.preprocess_frame(frame)

        blue_mask = self.clean_mask(cv2.inRange(hsv, HSV_BLUE_LOWER, HSV_BLUE_UPPER))
        pink_mask = self.clean_mask(cv2.inRange(hsv, HSV_PINK_LOWER, HSV_PINK_UPPER))

        def find_center(mask):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None
            largest = max(contours, key=cv2.contourArea)
            (x, y), _ = cv2.minEnclosingCircle(largest)
            return int(x), int(y)

        blue_pos = find_center(blue_mask)
        pink_pos = find_center(pink_mask)

        if blue_pos and pink_pos:
            center_x = (blue_pos[0] + pink_pos[0]) // 2
            center_y = (blue_pos[1] + pink_pos[1]) // 2
            return (center_x, center_y), (pink_pos[0] - blue_pos[0], pink_pos[1] - blue_pos[1])

        return None
    
    def find_goal(self, frame):
        print("Scoring not yet implemented!")
        sys.exit(0)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
