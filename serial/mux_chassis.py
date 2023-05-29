'''
UART communication on Raspberry Pi using Pyhton
http://www.electronicwings.com

NOTE: Run with sudo on LOKI
'''
from enum import Enum
import serial
import threading
from time import sleep

class Chassis(Enum):
    FRONT = 0
    BACK = 1


def main():
    ser = serial.Serial("/dev/ttyS0", 9600)  # Open port with baud rate

    while True:
        print('Select a camera:')
        for chassis_enum in Chassis:
            print(f'[{chassis_enum.value}] {chassis_enum.name}')
    
        data = input('> ')
        
        ser.write(data.encode())  # Encode the string and transmit data serially
        sleep(0.1)  # Delay to allow the Arduino time to process the data

if __name__ == '__main__':
    main()