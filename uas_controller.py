#!/usr/bin/env python
# import ROS libraries
import rospy
import sys
import signal
        



import time
import serial

class uas_serial_controller:

    def __init__(self):
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.SEVENBITS
        )
        self.throttle = 0
        self.rudder = 50
        self.elevator = 50
        self.roll = 50
    def initialize_serial(self):
        """ser = serial.Serial(
            port = '/dev/ttyUSB1',
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )
        """
        self.ser.isOpen()

        input=1


    # Test section for serial com - WORKING
    """while 1 : 
        
"""
    def read_keyboard(self):
        input = raw_input(">> ")

        # quits the program
        if input == 'exit':
            self.ser.close()
            exit()

        # Increase throttle input
        if input == 'w':
            self.throttle =self.throttle + 1
            if self.throttle >= 100:
                self.throttle == 100
            print(self.throttle)
        # Decrease throttle input
        if input == 's':
            self.throttle = self.throttle - 1
            if self.throttle <= 0:
                self.throttle == 0
            print(self.throttle)
        # Increase rudder - right turn
        if input == 'a':
            self.rudder = self.rudder + 1
            if self.rudder >= 100:
                self.rudder == 100
            print(self.rudder)

        # Decrease rudder - left turn
        if input == 'd':
            self.rudder = self.rudder -1
            if self.rudder == -1:
                self.rudder == 0
            print(self.rudder)

        # Increase elevator - forward
        if input == 'i':
            self.elevator = self.elevator + 1
            if self.elevator >= 100:
                self.elevator == 100
            print(self.elevator)

        # Decrease elevator - backwards
        if input == 'k':
            self.elevator = self.elevator - 1
            if self.elevator <= 0:
                self.elevator == 0
            print(self.elevator)

        # Increase roll - right
        if input == 'l':
            self.roll = self.roll - 1
            if self.roll <= 0:
                self.roll== 0
            print(self.roll)

        # Decrease elevator - backwards
        if input == 'j':
            self.roll = self.roll + 1
            print('roll: ',type(self.roll))
            if self.roll == 100 :
                print('max')
                self.roll == 100
            print(self.roll)

        ###### Sends the characters to the arduino
        else:
            print('type ' , type(input))
            self.ser.write(input)
            out = ''

            time.sleep(1)
            while self.ser.inWaiting() > 0:
                out += self.ser.read(1)

            if out != '':
                print(">>" + out)


    def ptest(self):
        print('test')
    # Simulate stick controlls
if __name__ == '__main__':
    usc = uas_serial_controller()
    #   usc.ptest()

    while True:
        print(usc.read_keyboard())