# Louis Lapp

import serial

class MADLS:
    def __init__(self, com_port, baud_rate, verbose=False):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.verbose = verbose
        self.ser = serial.Serial(com_port, baud_rate)
	
    def tx(self, key, value):
        msg = key + str(value)
        encoded_msg = msg.encode("utf-8")
        self.ser.write(encoded_msg)

    def setPose(self, **kwargs):
        tx_values = {
            "centroid_x": "a",
            "centroid_y": "b",
            "centroid_z": "c",
            "yaw": "d",
            "pitch": "e"
        }
        
        for key, value in kwargs.items():
            try:
                print(key)
                self.tx(tx_values[key], f"{int(round(value*1000)):06}")
            except Exception as e:
                if self.verbose:
                    print(e)
    def terminate(self):
        self.ser.close()

# Example usage (see https://www.desmos.com/3d/p06gkvywrs for info on centroid, yaw, pitch)
madls = MADLS(com_port = "COM3", baud_rate = 9600, verbose=True)
madls.setPose(centroid_x = 0,
                centroid_y = 0,
                centroid_z = 0,
                yaw = 20,
                pitch = 40
                )
madls.terminate()