import serial
from threading import Thread

port = '/dev/ttyUSB0'
baud = 57600

ser = serial.Serial(port = port, baudrate = baud)

command_queue = []

def read_serial(s):
    print("Reading from " + s.name)
    while True:
        if s.inWaiting() > 0:
            line = s.readline().decode()
	    print(line)
        if len(command_queue) > 0:
            for cmd in command_queue:
                ser.write(cmd)

def speedToByteStr(speed_val):
    if speed_val > 500:
        speed_val = 500
    elif speed_val < -500:
        speed_val = -500
    
    first_byte = 0
    second_byte = 0

    if speed_val < 0:
        speed_val = 2**16 + speed_val # velocity is over 2 bytes
    #    first_byte = 255 # FF
    #    second_byte = speed_val - (255 << 8)
    #else: 
    first_byte = speed_val >> 8
    second_byte = speed_val - (first_byte << 8)
    
    return str(first_byte) + " " + str(second_byte)
        

speed = 100

while True:
    line = raw_input("Enter serial command or 'quit': ")
    if line == 'q' or line == 'quit':
        break
    parts = line.split(" ")
    cmd = line
    if line == "init":
        cmd = "128 131"
    if parts[0] == "speed":
        speed = int(parts[1])
        print("Set speed to " + str(speed) + " millimeters/s") 
        continue
    if line == "stop":
        cmd = "137 0 0 0 0"
    if line == "forward":
        cmd = "137 " + speedToByteStr(speed) + " 128 0" # 128 means straight
    if line == "back":
        cmd = "137 " + speedToByteStr(-speed) + " 128 0"
    if line == "right":
        cmd = "137 " + speedToByteStr(speed) + " 255 255" # 0xFFFF in last 2 mean turn in place clockwise
    if line == "left":
        cmd = "137 " + speedToByteStr(speed) + " 0 1" # 0x0001 means turn in place counterclockwise
    parts = cmd.split(" ")
    print("  Parsing " + cmd)
    try:
        cmd_bytes = bytearray([int(p) for p in parts])
    except Exception,e:
        print(str(e))
        print("  Error: " + p + " is not a number")
        continue
    print("  Sending " + cmd)
    ser.write(cmd_bytes)
    
#t = Thread(target=read_serial, args=(ser,))
#t.start()
#read_serial(ser)
#ser.write(bytearray([128, 132, 139, 2, 0, 0]))
#ser.write(chr(128))
#ser.write(bytearray([128, 131]))
#ser.write(chr(131))
ser.close()
