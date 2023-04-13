# -*- coding: utf-8 -*-

import serial
from threading import Thread
import time

port = '/dev/ttyUSB0'
baud = 57600

ser = serial.Serial(port = port, baudrate = baud)

command_queue = []


def clamp(val, low, high):
    if val > high:
        return high
    if val < low:
        return low
    return val


def speedToBytes(speed_val):
    return int16ToBytes(clamp(speed_val, -500, 500))
def radiusToBytes(radius_val):
    return int16ToBytes(clamp(radius_val, -2000, 2000))


def int16ToBytes(val):
    
    first_byte = 0
    second_byte = 0

    if val < 0:
        val = 2**16 + val
    #    first_byte = 255 # FF
    #    second_byte = speed_val - (255 << 8)
    #else: 
    first_byte = val >> 8
    second_byte = val - (first_byte << 8)
    
    return first_byte, second_byte


def start():
    # Opcode 128: Start
    command_queue.append([128])

def passive_mode():
    # Opcode 128: Start, changes any mode to passive

    # Upon sending the Start command or any one of the demo
    # commands (which also starts the specific demo, e.g., Spot
    # Cover, Cover, Cover and Dock, or Demo), the OI enters
    # into Passive mode. When the OI is in Passive mode, you
    # can request and receive sensor data using any of the
    # sensors commands, but you cannot change the current
    # command parameters for the actuators (motors, speaker,
    # lights, low side drivers, digital outputs) to something else.
    # To change how one of the actuators operates, you must
    # switch from Passive mode to Full mode or Safe mode.
    # While in Passive mode, you can read Create’s sensors,
    # watch Create perform any one of its ten built-in demos,
    # and charge the battery.

    command_queue.append([128])

def safe_mode():
    # Opcode 131: Safe
    # This command puts the OI into Safe mode, enabling user
    # control of Create. It turns off all LEDs. The OI can be in
    # Passive, Safe, or Full mode to accept this command.

    # When you send a Safe command to the OI, Create enters into
    # Safe mode. Safe mode gives you full control of Create, with
    # the exception of the following safety-related conditions:
    # • Detection of a cliff while moving forward (or moving
    # backward with a small turning radius, less than one robot
    # radius).
    # • Detection of a wheel drop (on any wheel).
    # • Charger plugged in and powered.
    # Should one of the above safety-related conditions occur
    # while the OI is in Safe mode, Create stops all motors and
    # reverts to the Passive mode.
    # If no commands are sent to the OI when in Safe mode, Create
    # waits with all motors and LEDs off and does not respond to
    # Play or Advance button presses or other sensor input.
    # Note that charging terminates when you enter Safe Mode.
    command_queue.append([131])

def full_mode():
    # Opcode 132: Full
    # This command gives you complete control over Create
    # by putting the OI into Full mode, and turning off the cliff,
    # wheel-drop and internal charger safety features. That is, in
    # Full mode, Create executes any command that you send
    # it, even if the internal charger is plugged in, or the robot
    # senses a cliff or wheel drop.

    # When you send a Full command to the OI, Create enters
    # into Full mode. Full mode gives you complete control over
    # Create, all of its actuators, and all of the safety-related
    # conditions that are restricted when the OI is in Safe mode,
    # as Full mode shuts off the cliff, wheel-drop and internal
    # charger safety features. To put the OI back into Safe mode,
    # you must send the Safe command.
    # If no commands are sent to the OI when in Full mode, Create
    # waits with all motors and LEDs off and does not respond to
    # Play or Advance button presses or other sensor input.
    # Note that charging terminates when you enter Full Mode.
    command_queue.append([132])


def drive(speed = 100, turn_radius = None):
    # Opcode 137: Drive
    velocity_high_byte, velocity_low_byte = speedToBytes(speed)
    if turn_radius is None:
        # radius bytes 0x8000 indicates straight motion
        command_queue.append([137, velocity_high_byte, velocity_low_byte, 128, 0])
    else:
        # positive radius goes left for some reason, flip to right
        radius_high_byte, radius_low_byte = radiusToBytes(-turn_radius)
        command_queue.append([137, velocity_high_byte, velocity_low_byte, radius_high_byte, radius_low_byte])


def turn(speed = 100):
    # Opcode 137: Drive
    # Turn in place clockwise = 0xFFFF
    # Turn in place counter-clockwise = 0x0001
    # Works with negative speed as well

    velocity_high_byte, velocity_low_byte = speedToBytes(speed)
    radius_high_byte, radius_low_byte = (255, 255)
    command_queue.append([137, velocity_high_byte, velocity_low_byte, radius_high_byte, radius_low_byte])


def register_beeps():
    # Opcode 140: Song
    command_queue.append([140, 0, 4, 60, 12, 64, 12, 67, 12, 72, 36])
    command_queue.append([140, 1, 2, 84, 12, 84, 36])
    command_queue.append([140, 2, 5, 60, 12, 64, 12, 67, 12, 72, 6, 72, 36])
    #command_queue.append([140 0 4 62 12 66 12 69 12 74 36])

def beep1():
    # Opcode 141: Play Song
    # Need to register with Opcode 140 first!
    command_queue.append([140, 0])
def beep2():
    command_queue.append([140, 1])
def beep3():
    command_queue.append([140, 2])




stage = -1
spd = 200

tc = 0

stages = []

def plan(action, duration):
    global tc
    global stages
    stages.append({time: tc, action: action})
    tc += duration


#####
plan(lambda: start() or safe_mode() or beep1(), 1)

d = 2
dm = 3
dt = 1
tr = 100
plan(lambda: beep2() or drive(speed = spd, turn_radius = None), d)
plan(lambda: drive(speed = -spd, turn_radius = None), d)
plan(lambda: turn(speed = spd), d)
plan(lambda: turn(speed = -spd), d)
plan(lambda: turn(speed = spd), d)
plan(lambda: turn(speed = -spd), d)
plan(lambda: drive(speed = spd, turn_radius = tr), d)
plan(lambda: drive(speed = -spd, turn_radius = tr), d)
plan(lambda: drive(speed = spd, turn_radius = -tr), d)
plan(lambda: drive(speed = -spd, turn_radius = -tr), d)

for i in range(4):    
    plan(lambda: beep2() or drive(speed = spd, turn_radius = None), dm)
    plan(lambda: turn(speed = spd), dt)

plan(lambda: beep3() or passive_mode(), 1)
####


start = time.time()
while True:
    t = time.time() - start

    if stage >= len(stages):
        break

    next_stage = stages[stage + 1]
    if next_stage.time <= t:
        next_stage.action()
        stage += 1

    for command in command_queue:
        ser.write(bytearray(command))

    time.sleep(0.010)
ser.close()
