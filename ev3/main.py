#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import socket

# Setup EV3 and motors
ev3 = EV3Brick()

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
intake_left = Motor(Port.C)
intake_right = Motor(Port.B)

# Setup drive base
wheel_diameter = 70  # mm
axle_track = 100     # mm
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

# Setup socket server
HOST = ''
PORT = 12345

s = socket.socket()
s.bind((HOST, PORT))
s.listen(1)

ev3.speaker.beep()
print("Listening for commands on port", PORT)

def handle_command(cmd):
    parts = cmd.strip().lower().split()
    if not parts:
        return "error: empty command"

    try:
        if parts[0] == "move" and len(parts) == 2:
            dist_cm = int(parts[1])
            robot.straight(dist_cm * 10)
            return "done"

        elif parts[0] == "rotate" and len(parts) == 2:
            angle = int(parts[1])
            robot.turn(angle)
            return "done"

        elif parts[0] == "intake" and parts[1] == "on":
            intake_left.run(-600)
            intake_right.run(600)
            return "done"

        elif parts[0] == "intake" and parts[1] == "off":
            intake_left.stop()
            intake_right.stop()
            return "done"

        elif parts[0] == "intake" and parts[1] == "reverse":
            if len(parts) == 3:
                duration = int(parts[2])  # in ms
                intake_left.run_time(600, duration)
                intake_right.run_time(-600, duration)
            else:
                intake_left.run_time(600, 30)
                intake_right.run_time(-600, 30)
            return "done"

        else:
            return "error: unknown command"

    except Exception as e:
        {}

while True:
    client, addr = s.accept()
    print("Connected to", addr)

    while True:
        try:
            data = client.recv(1024)
            if not data:
                break

            command = data.decode()
            print("Command:", command)

            result = handle_command(command)
            client.send((result + "\n").encode())

        except Exception as e:
            print("Fatal error:", e)
            break

    client.close()
