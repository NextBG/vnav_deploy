# coding: utf-8
import serial
import threading

from typing import Tuple

class Gnss:
    def __init__(self) -> None:
        self.ser = serial.Serial("/dev/wit_gps", 115200)

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

    def _to_deg(self, value, dir):
        """Convert NMEA lat/lon to decimal degrees."""
        try:
            d, m = divmod(float(value), 100)
            deg = d + (m / 60)
            if dir in ['S', 'W']:
                deg = -deg
            return deg
        except ValueError:
            return None
    
    def _parse_gga(self, line):
        """Parse GGA line and update attributes."""
        parts = line.split(',')
        if len(parts) < 15:
            return  # Not enough parts to parse

        self.lat = self._to_deg(parts[2], parts[3])
        self.lon = self._to_deg(parts[4], parts[5])
        try:
            self.numSv = int(parts[7])
        except ValueError:
            self.numSv = None

        # print(f"Latitude: {self.lat}, Longitude: {self.lon}, Number of satellites: {self.numSv}")

    def _gps_receive(self):
        while True:
            line = self.ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GNGGA'):
                self._parse_gga(line)
                                                
    def read(self) -> Tuple[float, float, int]: # Latitude, Longitude, Number of satellites
        return (self.lat, self.lon, self.numSv)

    def __del__(self):
        self.ser.close()
        
        # Thread
        if self.t.is_alive():
            self.t.join()

        print("GPS serial Close!")

if __name__ == '__main__':
    gnss = Gnss()