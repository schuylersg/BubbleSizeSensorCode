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
from time import time, sleep, asctime
from struct import unpack, calcsize, pack
from os.path import isfile

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

import argparse

if __name__ == "__main__":
    print "Starting Bubble Sensor Reader"

    PARSER = argparse.ArgumentParser(description='GUI for programming and testing CMT Tags.')
    PARSER.add_argument('--port', type=str, dest='port', required=True, default="Info", help='Serial Com Port (e.g. COM4)')
    PARSER.add_argument('--file', type=str, dest='filename', required=False, default="Info", help='Data output file name')
    ARGS = PARSER.parse_args()

    filename = ARGS.filename
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = ARGS.port
    ser.timeout = 1

    try:
        ser.open()
    except:
        print "Error opening serial port"
        exit (0)
    #sleep(3)
    ser.write('!')  #anything - doesn't matter
    sleep(1)

    ser_data = ser.read(ser.inWaiting())
    print ser_data

    if "Bubble Size Sensor" not in ser_data:
        print "Unable to connect to bubble sensor"
        print "Read: %s" % ser_data
        exit(0)

    while True:
        print "COMMANDS\nR = Read Data\nX = Delete Data\nE = Exit"
        request = raw_input("Enter Option")
        if request == "R":
            ser.write('R')  #read data command
            sleep(1)
            data = ""
            #read all data
            while(ser.inWaiting() > 0):
                data = "".join((data, ser.read(ser.inWaiting())))
                sleep(0.1)

            if filename == None or isfile(filename):
                filename = "Bubble_Data_%s.csv" % asctime().replace(' ', '_').replace(':', '-')

            data_list = list()
            index = 0
            while index < len(data):
                msg_num = str(unpack('B', data[index])[0])
                if msg_num != '255':
                    msg_val = MSG_CODES[msg_num]
                    msg_size  = calcsize(MSG_SIZE[msg_val])
                    msg_data = unpack(MSG_SIZE[msg_val], data[index+1:index+msg_size+1])
                    print msg_val, msg_data
                    index = index + msg_size + 1
                    #print msg_val, msg_data
                    if msg_val == "MSG_EVENT_START":
                        data_list.append(dict())
                        data_list[-1]["start"] = msg_data
                        data_list[-1]["det1start"] = list()
                        data_list[-1]["det2start"] = list()
                        data_list[-1]["det3start"] = list()
                        data_list[-1]["det1end"] = list()
                        data_list[-1]["det2end"] = list()
                        data_list[-1]["det3end"] = list()
                        data_list[-1]["errors"] = list()

                    elif msg_val == "MSG_EVENT_END":
                        data_list[-1]["end"] = msg_data
                    elif msg_val == "MSG_DET1_START":
                        data_list[-1]["det1start"].append(msg_data[0])
                    elif msg_val == "MSG_DET2_START":
                        data_list[-1]["det2start"].append(msg_data[0])
                    elif msg_val == "MSG_DET3_START":
                        data_list[-1]["det3start"].append(msg_data[0])
                    elif msg_val == "MSG_DET1_END":
                        data_list[-1]["det1end"].append(msg_data[0])
                    elif msg_val == "MSG_DET2_END":
                        data_list[-1]["det2end"].append(msg_data[0])
                    elif msg_val == "MSG_DET3_END":
                        data_list[-1]["det3end"].append(msg_data[0])
                    elif msg_val == "MSG_ERROR":
                        data_list[-1]["errors"].append(msg_data[0])
                else:
                    index += 1

            with open(filename, 'w')as fid:
                for event in data_list:
                    max_items = max(len(event["det1start"]), len(event["det2start"]), len(event["det3start"]), len(event["det1end"]), len(event["det2end"]), len(event["det3end"]))
                    for it in range(0, max_items):
                        try:
                            fid.write("%s, " % event['det1start'][it])
                        except:
                            fid.write(" , ")
                        try:
                            fid.write("%s, " % event['det1end'][it])
                        except:
                            fid.write(" , ")
                        try:
                            fid.write("%s, " % event['det2start'][it])
                        except:
                            fid.write(" , ")
                        try:
                            fid.write("%s, " % event['det2end'][it])
                        except:
                            fid.write(" , ")
                        try:
                            fid.write("%s, " % event['det3start'][it])
                        except:
                            fid.write(" , ")
                        try:
                            fid.write("%s, " % event['det3end'][it])
                        except:
                            fid.write(" , ")
                        fid.write("%s, " % str(event["start"]))
                        try:
                            fid.write("%s, " % event["end"])
                        except:
                            fid.write(" , ")
                        try:
                            fid.write("%s, " % str(event["errors"]))
                        except:
                            fid.write(" , ")
                        fid.write('\n')

        elif request == 'X':
            confirm = raw_input("Confirm you want to delete the data (Y/N)")
            if confirm == 'Y':
                ser.write('X')
                sleep(0.1)
                ser.write('Y')
        elif request =='E':
            ser.write('E')  #exit computer mode
            sleep(0.01)
            break
        else:
            print ("Unrecongnized command")

    ser.close()
