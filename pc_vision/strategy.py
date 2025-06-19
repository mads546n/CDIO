import math
from config import WALL_MARGIN_PX

class StrategyPlanner:
    def __init__(self, vision_system):
        self.current_target = None
        self.command_queue = []
        self.vision = vision_system
        self.debug_draw = {}

    def approach_ball(self, balls, robot_pos):
        self.debug_draw.clear()

        if self.command_queue:
            return self.command_queue.pop(0)

        if not balls or robot_pos is None:
            return "DONE"

        (robot_x, robot_y), (robot_dx, robot_dy) = robot_pos

        vip_balls = [b for b in balls if b["is_vip"]]
        white_balls = [b for b in balls if not b["is_vip"]]
        white_balls = [b for b in white_balls if self._distance(robot_x, robot_y, b["x"], b["y"]) > 3.0]
        candidates = white_balls if white_balls else vip_balls
        if not candidates:
            return None

        target = min(candidates, key=lambda b: self._distance(robot_x, robot_y, b["x"], b["y"]))
        self.current_target = target

        tx, ty = target["x"], target["y"]
        is_near_wall = target["wall_proximity"]["is_near_wall"]
        robot_near_wall = self.vision.robot_inside_safe_area(robot_x, robot_y)

        if is_near_wall and not robot_near_wall:
            print("[PLAN] Special wall approach logic activated")
            side = target["wall_proximity"]["side"]

            approach_dx = 0
            approach_dy = 0
            if side == "left":
                approach_dx = WALL_MARGIN_PX
            elif side == "right":
                approach_dx = -WALL_MARGIN_PX
            elif side == "top":
                approach_dy = WALL_MARGIN_PX
            elif side == "bottom":
                approach_dy = -WALL_MARGIN_PX

            px, py = tx + approach_dx, ty + approach_dy

            # Save debug info
            self.debug_draw["offset_point"] = (px, py)
            self.debug_draw["path"] = [(robot_x, robot_y), (px, py), (tx, ty)]

            # Step 1: rotate toward offset point and move
            to_offset_dx = px - robot_x
            to_offset_dy = py - robot_y
            angle_to_offset = math.degrees(math.atan2(to_offset_dy, to_offset_dx))
            distance_to_offset = self._distance(robot_x, robot_y, px, py) / 10

            # Step 2: rotate toward ball and move
            angle_to_ball = math.degrees(math.atan2(ty - py, tx - px))

            self.command_queue = [
                "intake on",
                f"move 20",
                f"rotate {int(angle_to_ball)}",
                f"move {int(distance_to_offset)}",
                f"rotate {int(angle_to_offset)}",
            ]
            return self.command_queue.pop(0)

        # Normal field logic
        dx = tx - robot_x
        dy = ty - robot_y
        angle_deg = math.degrees(math.atan2(robot_dx * dy - robot_dy * dx,
                                            robot_dx * dx + robot_dy * dy))
        distance_cm = self._distance(robot_x, robot_y, tx, ty) / 10

        print(f"[TARGET] Ball at ({tx}, {ty}) — angle: {angle_deg:.1f}°, dist: {distance_cm:.1f}cm")

        self.command_queue = [
            "intake on",
            f"move {int(distance_cm)}",
            f"rotate {int(angle_deg)}"
        ]
        return self.command_queue.pop(0)

    def get_debug_draw(self):
        return self.debug_draw

    def _distance(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

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
