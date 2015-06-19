#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      Schuyler
#
# Created:     17/06/2015
# Copyright:   (c) Schuyler 2015
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import serial
import argparse
from time import time
from time import sleep
from struct import unpack, calcsize, pack

#DEFINE THE DIFFERENT MESSAGE TYPES - they can be values between 1 and 254 - 0 and 255 are reserved
MSG_CODES = {
'1' : 'MSG_EVENT_START',
'2' : 'MSG_EVENT_END',
'3' : 'MSG_DET1_START',
'4' : 'MSG_DET2_START',
'5' : 'MSG_DET3_START',
'6' : 'MSG_DET1_END',
'7' : 'MSG_DET2_END',
'8' : 'MSG_DET3_END',
'9' : 'MSG_ERROR',
'255' : 'MSG_DATA_END'
}

#DEFINE THE SIZE OF EACH MESSAGE (in bytes)
MSG_SIZE = {
'MSG_EVENT_START' : '>LHHH', #10,    #one uint32_t for millis and three uint16_t for the avg detector values
'MSG_EVENT_END'   : '>L',	    #time just before sleep (1 uint32_t) millis()
'MSG_DET1_START'  : '>L',	    #time
'MSG_DET2_START'  : '>L',	    #time
'MSG_DET3_START'  : '>L',	    #time
'MSG_DET1_END'	   : '>L',     #time
'MSG_DET2_END'	   : '>L',     #time
'MSG_DET3_END'	   : '>L',     #time
'MSG_ERROR'	   : 'B',	    #8-bit error code
'MSG_DATA_END'     : 0
}

def main():
    pass

if __name__ == '__main__':
    print "Starting"
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = 'COM4'
    ser.timeout = 1
    try:
        ser.open()
    except:
        print "Error opening serial port"
        exit (0)
    sleep(3)
    ser.write('!')  #anything - doesn't matter
    sleep(0.1)
    print(ser.read(ser.inWaiting()))

    sleep(1)
    ser.write('R')  #read data command
    sleep(1)
    data = ""
    #read all data
    while(ser.inWaiting() > 0):
        data = "".join((data, ser.read(ser.inWaiting())))
        sleep(0.1)

    ser.write('E')  #exit computer mode

    index = 0

    while index < len(data):
        msg_num = str(unpack('B', data[index])[0])
        if msg_num != '255':
            msg_val = MSG_CODES[msg_num]
            msg_size  = calcsize(MSG_SIZE[msg_val])
            msg_data = unpack(MSG_SIZE[msg_val], data[index+1:index+msg_size+1])
            str_out = msg_val
            for d in msg_data:
                str_out = "%s %d" % (str_out, d)
            print str_out
            index = index + msg_size + 1
        else:
            index += 1
    #ser.close()
