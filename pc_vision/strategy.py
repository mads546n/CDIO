# pc_vision/strategy.py

import math

class StrategyPlanner:
    def __init__(self):
        self.target_ball = None
        self.command_queue = []

    def decide_next_action(self, balls, robot_pos):
        if self.command_queue:
            return self.command_queue.pop(0)

        if self.target_ball is not None:
            # We already completed a full plan
            self.done = True
            return None

        if not balls or robot_pos is None:
            return None

        # Unpack position and ignore direction vector
        if isinstance(robot_pos, tuple) and isinstance(robot_pos[0], tuple):
            (robot_x, robot_y), _ = robot_pos
        else:
            return None

        vip_balls = [b for b in balls if b[2]]
        candidates = vip_balls if vip_balls else balls

        target = min(candidates, key=lambda b: self._distance(robot_x, robot_y, b[0], b[1]))
        self.target_ball = target

        dx = target[0] - robot_x
        dy = target[1] - robot_y
        angle_deg = math.degrees(math.atan2(dy, dx))
        distance_cm = math.hypot(dx, dy) / 10

        print(f"[TARGET] Ball at ({target[0]}, {target[1]}) — angle: {angle_deg:.1f}°, dist: {distance_cm:.1f}cm")

        self.command_queue = [
            f"rotate {int(angle_deg)}",
            f"move {int(distance_cm)}",
            "intake on"
        ]

        return self.command_queue.pop(0)

    def _distance(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)
