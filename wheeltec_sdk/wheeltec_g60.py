# coding: utf-8
import time
import serial
import re
import threading

from typing import Tuple

class WheeltecG60:
    def __init__(self) -> None:
        self.ser = serial.Serial("/dev/wheeltec_gps", 9600)

        if self.ser.isOpen():
            print("GPS serial Open!")
        else:
            self.ser = None
            print("GPS serial Open Failed!")

        # Private variables
        self.lat: float = 0.0
        self.ulat = ''
        self.lon: float = 0.0
        self.ulon = ''
        self.numSv: int = 0

        # Thread
        self.t = threading.Thread(target=self._gps_receive)
        self.t.start()

    def _to_deg(self, in_data1, in_data2):
        len_data1 = len(in_data1)
        str_data2 = "%05d" % int(in_data2)
        temp_data = int(in_data1)
        symbol = 1
        if temp_data < 0:
            symbol = -1
        degree = int(temp_data / 100.0)
        str_decimal = str(in_data1[len_data1-2]) + str(in_data1[len_data1-1]) + str(str_data2)
        f_degree = int(str_decimal)/60.0/100000.0
        if symbol > 0:
            result = degree + f_degree
        else:
            result = degree - f_degree
        return result

    def _gps_receive(self):
        while True:
            if self.ser.inWaiting():
                if self.ser.read(1) == b'G':
                    time.sleep(.05) 
                    if self.ser.inWaiting():
                        if self.ser.read(1) == b'N':
                            if self.ser.inWaiting():
                                choice = self.ser.read(1)
                                if choice == b'G':
                                    if self.ser.inWaiting():
                                        if self.ser.read(1) == b'G':
                                            if self.ser.inWaiting():
                                                if self.ser.read(1) == b'A':
                                                    GGA = self.ser.read(70)
                                                    GGA_g = re.findall(r"\w+(?=,)|(?<=,)\w+", str(GGA))
                                                    if len(GGA_g) < 13:
                                                        return 0
                                                    else:
                                                        self.lat = self._to_deg(str(GGA_g[2]), str(GGA_g[3]))
                                                        self.ulat = GGA_g[4]
                                                        self.lon = self._to_deg(str(GGA_g[5]), str(GGA_g[6]))
                                                        self.ulon = GGA_g[7]
                                                        self.numSv = int(GGA_g[9])
                                                        return 1
                                                
    def read(self) -> Tuple[float, float, int]: # Latitude, Longitude, Number of satellites
        return (self.lat, self.lon, self.numSv)

    def __del__(self):
        self.ser.close()
        
        # Thread
        if self.t.is_alive():
            self.t.join()

        print("GPS serial Close!")

if __name__ == '__main__':
    g60 = WheeltecG60()
    while True:
        lat, lon, numSv = g60.read()
        print(f"Latitude: {lat}, Longitude: {lon}, Number of satellites: {numSv}")
        time.sleep(1)