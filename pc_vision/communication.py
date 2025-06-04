# pc_vision/communication.py

import socket
from config import EV3_IP, EV3_PORT

class EV3SocketClient:
    def __init__(self, mock_mode=False):
        self.mock_mode = mock_mode
        self.sock = None

    def connect(self):
        if self.mock_mode:
            print("[MOCK] Pretending to connect to EV3")
            return

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)
        try:
            self.sock.connect((EV3_IP, EV3_PORT))
            print(f"[CONNECTED] EV3 at {EV3_IP}:{EV3_PORT}")
        except Exception as e:
            print("[ERROR] Could not connect to EV3:", e)
            self.sock = None

    def send_command(self, command):
        if self.mock_mode:
            print(f"[MOCK] Sending command: {command}")
            return "done"

        if not self.sock:
            print("[ERROR] Not connected to EV3.")
            return None

        try:
            self.sock.sendall((command + "\n").encode())
            response = self.sock.recv(1024).decode().strip()
            return response
        except Exception as e:
            print("[ERROR] Communication failed:", e)
            return None

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            print("[CLOSED] Connection to EV3 closed.")
