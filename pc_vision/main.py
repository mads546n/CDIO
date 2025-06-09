# pc_vision/main.py

from vision import VisionSystem
from strategy import StrategyPlanner
from communication import EV3SocketClient
from config import MOCK_MODE
import time
import math

def main():
    print("🎥 LIVE MODE – Vision + EV3 Communication")

    vision = VisionSystem()
    ev3 = EV3SocketClient(mock_mode=MOCK_MODE)
    ev3.connect()

    planner = StrategyPlanner()

    while True:
        balls, robot_pos = vision.detect_state(show_debug=True)

        if not balls or robot_pos is None:
            print("⚠️ No balls or robot detected. Retrying...")
            continue

        command = planner.approach_ball(balls, robot_pos)

        if command is None:
            print("Could not generate command")
            continue
        elif command == "DONE":
            break

        print(f"→ Sending command: {command}")
        response = ev3.send_command(command)
        print(f"← Response: {response}")

    while True:
        goal, robot_pos = vision.find_goal()

        if goal is None or robot_pos is None:
            print("⚠️ No balls or robot detected. Retrying...")
            continue
        elif command == "DONE":
            ev3.close()

        command = planner.score(goal, robot_pos)

        if command is None:
            print("Could not generate command")
            continue
        elif command == "DONE":
            break

        print(f"→ Sending command: {command}")
        response = ev3.send_command(command)
        print(f"← Response: {response}")

if __name__ == "__main__":
    main()
