'''
UART communication on Raspberry Pi using Pyhton
http://www.electronicwings.com

NOTE: Run with sudo on NYX
'''
from enum import Enum
import serial
import threading
from time import sleep

class Arm(Enum):
    UNDER = 0
    ABOVE = 1
    ELBOW = 2

def main():
    ser = serial.Serial("/dev/ttyS0", 9600)  # Open port with baud rate

    r2_state = '0'
    r8_state = '0'
    ser.write((r8_state + r2_state).encode())

    while True:
        print('Select a display:')
        print('[2] Arm (R2)')
        print('[8] Arm (R8)')
        choice = input('> ')

        print('Select a camera:')

        if choice == '2' or choice == '8': 
            for arm_enum in Arm:
                print(f'[{arm_enum.value}] {arm_enum.name}')
        
            data = input('> ')

            if choice == '2':
                r2_state = data
            elif choice == '8':
                r8_state = data

            data = r8_state + r2_state

            ser.write(data.encode())  # Encode the string and transmit data serially
            sleep(0.1)  # Delay to allow the Arduino time to process the data
        
        else:
            print('ERROR')

if __name__ == '__main__':
    main()