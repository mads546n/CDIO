# pc_vision/strategy.py

import math
import sys

class StrategyPlanner:
    def __init__(self):
        self.current_target = None
        self.command_queue = []

    def approach_ball(self, balls, robot_pos):
        if self.command_queue:
            return self.command_queue.pop(0)
        
        if not balls:
            return "DONE"

        if robot_pos is None:
            return None

        if isinstance(robot_pos, tuple) and isinstance(robot_pos[0], tuple):
            (robot_x, robot_y), _ = robot_pos
        else:
            return None

        vip_balls = [b for b in balls if b[2]]
        white_balls = [b for b in balls if not b[2]]
        # candidates = white_balls if white_balls else vip_balls
        candidates = vip_balls

        target = min(candidates, key=lambda b: self._distance(robot_x, robot_y, b[0], b[1]))
        self.current_target = target

        dx = target[0] - robot_x
        dy = target[1] - robot_y
        angle_deg = -math.degrees(math.atan2(dy, dx))
        distance_cm = self._distance(robot_x, robot_y, target[0], target[1]) / 10

        print(f"[TARGET] Ball at ({target[0]}, {target[1]}) — angle: {angle_deg:.1f}°, dist: {distance_cm:.1f}cm")

        self.command_queue = [
            "intake on",
            f"move {int(distance_cm)}",
            f"rotate {int(angle_deg)}",
        ]

        return self.command_queue.pop()
    
    def score(self, goal, robot_pos):
        if self.command_queue:
            return self.command_queue.pop(0)

        if robot_pos is None:
            return None

        if isinstance(robot_pos, tuple) and isinstance(robot_pos[0], tuple):
            (robot_x, robot_y), _ = robot_pos
        else:
            return None

        if isinstance(goal, tuple):
            (goal_x, goal_y), _ = goal
        else:
            return None

        target = (goal_x, goal_y)
        self.current_target = target

        dx = target[0] - robot_x
        dy = target[1] - robot_y
        angle_deg = math.degrees(math.atan2(dy, dx))
        distance_cm = math.hypot(dx, dy) * 10

        print(f"[TARGET] Ball at ({target[0]}, {target[1]}) — angle: {angle_deg:.1f}°, dist: {distance_cm:.1f}cm")

        self.command_queue = [
            "intake reverse",
            f"move {int(distance_cm)}",
            f"rotate {int(angle_deg)}",
        ]

        return self.command_queue.pop()

    def _distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def _adjust_angle(self, angle_deg):
        self.command_queue = [
            "intake on",
            f"rotate {int(angle_deg)}"
        ]

    def _adjust_position(self, distance_cm):
        self.command_queue = [
            "intake on",
            f"move {int(distance_cm)}"
        ]

