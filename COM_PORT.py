import serial

ser = serial.Serial(port="COM3", baudrate= 115200, timeout=2)
ser.open()
while True:
    kp = input(float("enter kp value"))
    ki = input(float("enter ki value"))
    kd = input(float("enter kd value"))
    ser.write(kp)
    ser.write(ki)
    ser.write(kd)

ser.close()