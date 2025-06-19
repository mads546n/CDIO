
import sys
from config import (
    HSV_WHITE_LOWER, HSV_WHITE_UPPER,
    HSV_ORANGE_LOWER, HSV_ORANGE_UPPER,
    HSV_BLUE_LOWER, HSV_BLUE_UPPER,
    HSV_PINK_LOWER, HSV_PINK_UPPER,
    HSV_RED_LOWER, HSV_RED_UPPER, WALL_MARGIN_PX
)

import cv2
import numpy as np
import math

class VisionSystem:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
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

        self.strategy = None  # Link will be set externally (e.g. in main.py)


    def detect_state(self, show_debug=True):
        ret, frame = self.cap.read()
        if not ret:
            return [], None

        balls = self.detect_balls(frame)
        robot_pos = self.detect_robot(frame)
        goals = self.find_goal(frame, draw_debug=show_debug)
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

            if self.strategy and hasattr(self.strategy, "get_debug_draw"):
                debug = self.strategy.get_debug_draw()
                if "offset_point" in debug:
                    ox, oy = debug["offset_point"]
                    cv2.circle(frame, (int(ox), int(oy)), 6, (255, 0, 0), -1)
                if "path" in debug:
                    path = debug["path"]
                    for i in range(len(path) - 1):
                        pt1 = (int(path[i][0]), int(path[i][1]))
                        pt2 = (int(path[i + 1][0]), int(path[i + 1][1]))
                        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)


            cv2.imshow("GolfBot Vision", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit()

        return list(filter(lambda ball: ball not in eggs, list(balls.values()))), robot_pos, eggs, goals

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

                ball_data = {
                    "x": int(x),
                    "y": int(y),
                    "is_vip": is_vip,
                    "wall_proximity": self.get_wall_proximity(int(x), int(y))
                }
                raw_detections.append(ball_data)


        return self.assign_ball_ids(raw_detections)



    def assign_ball_ids(self, current_detections):
        new_ids = {}
        used_prev_ids = set()

        for ball in current_detections:
            x, y = ball["x"], ball["y"]
            min_dist = float('inf')
            matched_id = None

            for ball_id, prev_ball in self.prev_ball_ids.items():
                if ball_id in used_prev_ids:
                    continue
                px, py = prev_ball["x"], prev_ball["y"]
                dist = math.hypot(x - px, y - py)
                if dist < min_dist and dist < 40:
                    min_dist = dist
                    matched_id = ball_id

            if matched_id is not None:
                new_ids[matched_id] = ball
                used_prev_ids.add(matched_id)
            else:
                new_ids[self.next_id] = ball
                self.next_id += 1

        self.prev_ball_ids = new_ids.copy()
        return new_ids

    def detect_walls(self, frame, draw_debug=False):
        WALL_MARGIN_PX = 95  # Margin based on ~17cm robot width

        # 1) Convert to HSV and mask red
        hsv = self.preprocess_frame(frame)
        red_mask = self.clean_mask(
            cv2.inRange(hsv, HSV_RED_LOWER, HSV_RED_UPPER)
        )

        # 2) Find red contours
        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        # 3) Merge all contour points
        all_pts = np.vstack(contours).reshape(-1, 2)

        # 4) Compute convex hull
        hull = cv2.convexHull(all_pts)

        # 5) Fit rotated rectangle
        rect = cv2.minAreaRect(hull)         # ((cx,cy), (w,h), angle)
        box = cv2.boxPoints(rect).astype(np.int32)

        # 6) Extract axis-aligned bounding box
        x_min, y_min = np.min(box, axis=0)
        x_max, y_max = np.max(box, axis=0)

        # 7) Store wall bounds for reuse
        self.wall_bounds = (x_min, x_max, y_min, y_max)

        if draw_debug:
            # A. Draw green outer wall rectangle
            cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)

            # B. Mark center and angle
            cx, cy = map(int, rect[0])
            cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
            w, h = rect[1]
            angle = rect[2]
            cv2.putText(
                frame,
                f"{w:.0f}x{h:.0f}@{angle:.1f}°",
                (cx - 50, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

            # C. Draw yellow safe area as a frame (not filled)
            margin = WALL_MARGIN_PX

            # Top border
            cv2.rectangle(
                frame,
                (x_min, y_min),
                (x_max, y_min + margin),
                (0, 255, 255), -1
            )
            # Bottom border
            cv2.rectangle(
                frame,
                (x_min, y_max - margin),
                (x_max, y_max),
                (0, 255, 255), -1
            )
            # Left border
            cv2.rectangle(
                frame,
                (x_min, y_min + margin),
                (x_min + margin, y_max - margin),
                (0, 255, 255), -1
            )
            # Right border
            cv2.rectangle(
                frame,
                (x_max - margin, y_min + margin),
                (x_max, y_max - margin),
                (0, 255, 255), -1
            )

        return box


    def get_wall_proximity(self, x, y):
        WALL_MARGIN_PX = 95

        if not hasattr(self, "wall_bounds"):
            return {
                "is_near_wall": False,
                "side": None,
                "distance_px": None
            }

        x_min, x_max, y_min, y_max = self.wall_bounds
        proximity = {
            "is_near_wall": False,
            "side": None,
            "distance_px": None
        }

        if x < x_min + WALL_MARGIN_PX:
            proximity.update({"is_near_wall": True, "side": "left", "distance_px": x - x_min})
        elif x > x_max - WALL_MARGIN_PX:
            proximity.update({"is_near_wall": True, "side": "right", "distance_px": x_max - x})
        elif y < y_min + WALL_MARGIN_PX:
            proximity.update({"is_near_wall": True, "side": "top", "distance_px": y - y_min})
        elif y > y_max - WALL_MARGIN_PX:
            proximity.update({"is_near_wall": True, "side": "bottom", "distance_px": y_max - y})

        return proximity



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

    def find_goal(self, frame, draw_debug=False):
        box = self.detect_walls(frame, draw_debug=draw_debug)

        if box is None or len(box) != 4:
            return {"big_goal": None, "small_goal": None}

        box = np.array(box).reshape(4,2)

        sorted_pts = sorted(box, key=lambda p: (p[1], p[0]))
        top_pts = sorted(sorted_pts[:2], key=lambda p: p[0])
        bottom_pts = sorted(sorted_pts[2:], key=lambda p: p[0])
        ordered_box = np.array([top_pts[0], top_pts[1], bottom_pts[1], bottom_pts[0]])

        def midpoint(p1, p2):
            return ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)

        sides = [
            (ordered_box[0], ordered_box[1]),
            (ordered_box[1], ordered_box[2]),
            (ordered_box[2], ordered_box[3]),
            (ordered_box[3], ordered_box[0])
        ]
        side_lenghts = [np.linalg.norm(np.array(p2)-np.array(p1)) for (p1, p2) in sides]
        short_side_indices = sorted(range(4), key=lambda i: side_lenghts[i])[:2]
        goal_centers = [midpoint(*sides[i]) for i in short_side_indices]

        if goal_centers[0][0] < goal_centers[1][0]:
            big_goal = goal_centers[0]
            small_goal = goal_centers[1]
        else:
            big_goal = goal_centers[1]
            small_goal = goal_centers[0]

        if draw_debug:
            for pt, color, label in [
                (big_goal, (0,0,255), "BIG"),
                (small_goal, (255,0,255), "SMALL")
            ]:
                cv2.rectangle(frame, (pt[0]-5, pt[1]-5), (pt[0]+5, pt[1]+5), color, -1)
                cv2.putText(frame, label, (pt[0] + 6, pt[1] -6),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        return {
            "big_goal": big_goal,
            "small_goal": small_goal,
        }


    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
