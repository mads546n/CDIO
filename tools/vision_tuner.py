import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from pc_vision.vision import VisionSystem
import time
import cv2

def main():
    print("🎥 TEST MODE – Vision Only (No EV3)")

    vision = VisionSystem()

    try:
        while True:
            balls, robot_pos, eggs = vision.detect_state(show_debug=True)

            if not balls:
                print("⚠️ No balls detected.")
            else:
                print(f"🎯 Balls: {balls}")

            if robot_pos is None:
                print("⚠️ Robot not detected.")
            else:
                print(f"🤖 Robot position: {robot_pos}")

            if eggs:
                print(f"🥚 Eggs: {eggs}")

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("🛑 Quitting.")
                break

            time.sleep(0.1)

    finally:
        del vision

if __name__ == "__main__":
    main()
