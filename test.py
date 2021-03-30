# testSerialSimulator.py
# D. Thiebaut
# This program energizes the fakeSerial simulator using example code taken
# from http://pyserial.sourceforge.net/shortintro.html
#

# import the simulator module (it should be in the same directory as this program)
import serial
from time import sleep

# Example 1  from http://pyserial.sourceforge.net/shortintro.html
def Example1():
    counter = 0
    ser = serial.Serial('COM28', 19200, timeout=1)  # open first serial port
    print(ser.name)  # check which port was really used
    while True:
        ser.write(("message number: " +  str(counter) + "\n").encode('utf-8'))  # write a string
        sleep(0.0001)
        print(counter)
        counter  = counter +1
    ser.close()  # close port




Example1()