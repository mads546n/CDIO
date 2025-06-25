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

    planner = StrategyPlanner(vision)
    vision.strategy = planner
    vision.set_walls()
    sleep = True

   

    while True:
        if(sleep):
            time.sleep(5)
            sleep = False
        balls, robot_pos, eggs = vision.detect_state(show_debug=True)

        if not robot_pos:
            print("⚠️ No balls or robot detected. Retrying...")
            continue

        command = planner.decide_next_move(balls, robot_pos)

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
