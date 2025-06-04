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
            time.sleep(0.5)
            continue

        command = planner.decide_next_action(balls, robot_pos)

        if command is None:
            print("✅ All balls handled or nothing to do.")
            break

        print(f"→ Sending command: {command}")
        response = ev3.send_command(command)
        print(f"← Response: {response}")

        # 🧠 Post-action check
        time.sleep(0.8)  # Let robot settle
        _, new_robot_pos = vision.detect_state(show_debug=False)

        if new_robot_pos is None:
            print("❌ Robot lost after command! Aborting.")
            break

        (old_pos, _) = robot_pos
        (new_pos, _) = new_robot_pos
        dx = new_pos[0] - old_pos[0]
        dy = new_pos[1] - old_pos[1]
        movement = math.hypot(dx, dy)

        if movement < 10:
            print(f"⚠️ Robot did not move significantly ({movement:.1f}px). Retrying...")
            continue  # Optional: Add retry logic later

        time.sleep(0.5)

    ev3.close()

if __name__ == "__main__":
    main()
