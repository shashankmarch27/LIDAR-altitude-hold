import serial

ser = serial.Serial(port="COM4", baudrate= 1200, timeout=2)
ser.open()
ser.close()