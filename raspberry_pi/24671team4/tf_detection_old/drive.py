import serial
import time
from new import *

if __name__ == '__main__':
    main()
    ser = serial.Serial('/dev/ttyACM1',115200,timeout=1)
    ser.flush()
    
    while True:
        key = input("E for exit!")
        if key == "w":
            command = b"forward\n"
            print(command)
           # ser.write(b"forward\n")
        elif key == "s":
            command = b"backward\n"
            print(command)
           # ser.write(b"backward\n")
        elif key == "a":
            command = b"ccw\n"
            print(command)
           # ser.write(b"left\n")
        elif key == "d":
            command = b"cw\n"
            print(command)
           # ser.write(b"right\n")
        elif key == "e":
            command = b"stop\n"
            print(command)
           # ser.write(b"stop\n")
        elif key == "t":
            command = b"up\n"
            print(command)
        elif key == "b":
            command = b"down\n"
            print(command)
        elif key == "g":
            command = b"center\n"
            print(command)
        elif key == "j":
            command = b"open\n"
            print(command)
        elif key == "l":
            command = b"close\n"
            print(command)
        elif key == "k":
            command = b"hold\n"
            print(command)
        elif key == "p":
            command = b"path\n"
            print(command)

        ser.write(command)
        line = ser.readline().decode('utf-8').rstrip()
       # serial.Serial.flush()
        print(line)
        time.sleep(0.005)
