import serial

ser = serial.Serial('COM3', 9600)

def tx(msg):
    encoded_msg = msg.encode("utf-8")
    ser.write(encoded_msg)

try: 
    while True:
        msg = input("Msg: ")
        tx(msg)
except KeyboardInterrupt:
    pass

ser.close()