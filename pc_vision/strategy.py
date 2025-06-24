import math
from config import WALL_MARGIN_PX
from config import TIMER_RESET

class StrategyPlanner:
    def __init__(self, vision_system):
        self.closer_waypoint = None
        self.farther_waypoint = None
        self.command_queue = []
        self.vision = vision_system
        self.debug_draw = {}
        self.timer = TIMER_RESET
        self.scoring = False
        self.switch_sides = False

    def decide_next_move(self, balls, robot_pos):
        # If we are trying to score go into the score routine
        if self.scoring:
            return self.score(robot_pos)
        else:
            # Otherwise if we are trying to switch sides initialize the waypoints
            if self.switch_sides:
                self.switch_sides_func(robot_pos)

            command = self.approach_ball(balls, robot_pos)

            # Force the robot to go into its scoring logic on its next move if it runs out its timer
            self.timer -= 1
            if self.timer == 0:
                self.timer = TIMER_RESET
                self.scoring = True

            return command

    def switch_sides_func(self, robot_pos):
        w1_x, _ = self.vision.waypoint1

        center_x1 = self.vision.center_bounds[0]
        center_x2 = self.vision.center_bounds[1]

        # Waypoint 1 is the closer waypoint if it is to the right of the center's right margin and if the robot is to the right of the center's left margin
        # otherwise waypoint 2 is closer. The farthest waypoint is always the waypoint that was not chosen as being closer.
        self.closer_waypoint, self.farther_waypoint = (self.vision.waypoint1, self.vision.waypoint2) if w1_x > center_x2 and robot_pos > center_x1 else (self.vision.waypoint2, self.vision.waypoint1)

        self.switch_sides = False

    def approach_ball(self, balls, robot_pos):
        self.debug_draw.clear()

        if self.command_queue:
            return self.command_queue.pop(0)

        if not balls or robot_pos is None:
            return "DONE"

        (robot_x, robot_y), (robot_dx, robot_dy) = robot_pos

        # vip_balls = [b for b in balls if b["is_vip"]]
        # white_balls = [b for b in balls if not b["is_vip"]]
        # white_balls = [b for b in white_balls if self._distance(robot_x, robot_y, b["x"], b["y"]) > 3.0]
        # candidates = white_balls if white_balls else vip_balls

        min_distance = 1

        center_x1 = self.vision.center_bounds[0]
        center_x2 = self.vision.center_bounds[1]
        candidates = [b for b in balls if ((b["x"] < center_x1 and robot_x < center_x1) or (b["x"] > center_x2 and robot_x > center_x2)) and self._distance(robot_x, robot_y, b["x"], b["y"]) > min_distance]

        if not candidates and (not self.closer_waypoint) and (not self.farther_waypoint):
            self.switch_sides = True
            self.scoring = True
            return None

        if candidates:
            target = min(candidates, key=lambda b: self._distance(robot_x, robot_y, b["x"], b["y"]))

            # tx and ty should be set to the closest ball or to one of the closer waypoint if it exists, or to the farther waypoint if it exists
            # the waypoints are changed to None when the robot has gotten sufficiently close to one of them
            tx, ty = target["x"], target["y"]

        if self.closer_waypoint:
            target = self.closer_waypoint
            if self._distance(robot_x, robot_y, self.closer_waypoint[0], self.closer_waypoint[1]) < min_distance:
                self.closer_waypoint = None
                tx, ty = self.farther_waypoint[0], self.farther_waypoint[1]
            else:
                tx, ty = self.closer_waypoint[0], self.closer_waypoint[1]
        elif self.farther_waypoint:
            if self._distance(robot_x, robot_y, self.farther_waypoint[0], self.farther_waypoint[1]) < min_distance:
                self.farther_waypoint = None
                target = min(candidates, key=lambda b: self._distance(robot_x, robot_y, b["x"], b["y"]))
            else:
                tx, ty = self.farther_waypoint[0], self.farther_waypoint[1]

        # is_near_wall = target["wall_proximity"]["is_near_wall"]
        # robot_near_wall = self.vision.robot_inside_safe_area(robot_x, robot_y)

        """if is_near_wall and not robot_near_wall:
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
                f"rotate {int(angle_to_offset)}",
                f"move {int(distance_to_offset)}",
                f"rotate {int(angle_to_ball)}",
                f"move 20",
            ]
            return self.command_queue.pop(0)"""

        # Normal field logic
        dx = tx - robot_x
        dy = ty - robot_y
        angle_deg = math.degrees(math.atan2(robot_dx * dy - robot_dy * dx,
                                            robot_dx * dx + robot_dy * dy))
        distance_cm = self._distance(robot_x, robot_y, tx, ty) / 20

        print(f"[TARGET] Ball at ({tx}, {ty}) — angle: {angle_deg:.1f}°, dist: {distance_cm:.1f}cm")

        self.command_queue = [
            "intake on",
            f"rotate {int(angle_deg)}",
            f"move {int(distance_cm)}"
        ]
        return self.command_queue.pop(0)
    
    def score(self, robot_pos):
        self.debug_draw.clear()

        if self.command_queue:
            return self.command_queue.pop(0)
        
        goal = self.vision.choose_goal(robot_pos)

        (tx, ty) = goal
        (robot_x, robot_y), (robot_dx, robot_dy) = robot_pos

        dx = tx - robot_x
        dy = ty - robot_y
        angle_deg = math.degrees(math.atan2(robot_dx * dy - robot_dy * dx,
                                            robot_dx * dx + robot_dy * dy))
        distance_cm = self._distance(robot_x, robot_y, tx, ty) / 10

        print(f"[TARGET] Goal at ({tx}, {ty}) — angle: {angle_deg:.1f}°, dist: {distance_cm:.1f}cm")

        if distance_cm > 10:
            self.command_queue = [
                f"intake off",
                f"rotate {int(angle_deg)}",
                f"move {int(distance_cm)}"
            ]
        else:
            move_back = 20

            # Make sure the robot doesn't get stuck doing this, for some reason it did at one point
            self.scoring = False
            self.command_queue = [
                f"intake reverse",
                f"move {int(-move_back)}"
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
