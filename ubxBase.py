#!/usr/bin/env python3
"""
Example implementation of UBX Gen 9 Configuration Interface
using CFG-VALSET, CFG-VALDEL and CFG-VALGET messages.
Connects to the receiver's serial port and sets up a
threaded UBXReader process. With the reader process running
in the background, it sends CFG-VALSET and CFG-VALDEL
configuration messages to set the UART1/2 baud rates in 
the BBR memory layer, and uses CFG-VALGET to poll the before and 
after results.
NB: This example will only work on Generation 9 or later devices
(e.g. NEO-9M). You'll get a series of ACK-NAK responses on earlier
devices. It also assumes you're connected via USB rather than UART1/2.
Created on 5 Dec 2020
@author: semuadmin
"""

import os
from sys import platform
from io import BufferedReader
from threading import Thread
from time import sleep, time

from pyubx2 import UBXMessage, UBXReader, VALCKSUM
from serial import Serial, SerialException, SerialTimeoutException
import argparse, math, sys

import pyubx2.exceptions as ube

class DataStore:

    def __init__(self, start_time):
        self.msg_nav_pvt = UBXMessage("NAV","NAV-PVT",0,lat=-1,lon=-1,height=-1,hMSL=-1)
        self.msgNavSvin = UBXMessage("NAV","NAV-SVIN",0,active=False)
        self.fixType = ["No fix", "Dead reackoning", "2D fix", 
                        "3D fix", "GNSS + dead rec", "Time-only fix"]
        self.svin_has_been_active = False
        self.start_time = start_time
        self.num_prints = 0

    def print_status(self, t_diff):
        #clear screen
        os.system('cls' if os.name == 'nt' else 'clear')
        print("Duration: {} [s], print diff {} [s]".format(time() - self.start_time, t_diff))
        print("Num prints:", self.num_prints)
        self.num_prints += 1
        print("Lat {:3.10f} [deg], Lon {:3.10f} [deg], Height {:4.3f} [m] (above ellipsoid), Hmsl {:4.3f} [m]".format(
                float(self.msg_nav_pvt.lat*1e-7),
                float(self.msg_nav_pvt.lon*1e-7),
                float(self.msg_nav_pvt.height*1e-3),
                float(self.msg_nav_pvt.hMSL*1e-3)))
        print("Num Sats {:3d}".format(self.msg_nav_pvt.numSV))
        print("Fix: {}".format(self.fixType[self.msg_nav_pvt.fixType]))

        if self.svin_has_been_active and self.msgNavSvin.active != 1:
            # Was active at some point, but is not anymore => has completed
            svin_status = "DONE"
        elif self.msgNavSvin.active == 1:
            # Is currently active
            svin_status = "In progress"
        else:
            svin_status = "Not started"
        print("Survey-in status: {}, Valid: {}, Used observations: {}, Duration: {} [s]".format(
                svin_status, self.msgNavSvin.valid, self.msgNavSvin.obs, self.msgNavSvin.dur))
        print("SVIN ECEF X {} [m] Y {} [m] Z {} [m] Accuracy {} [m]".format(
                float(self.msgNavSvin.meanX*1e-2) + float(self.msgNavSvin.meanXHP*1e-4),
                float(self.msgNavSvin.meanY*1e-2) + float(self.msgNavSvin.meanYHP*1e-4),
                float(self.msgNavSvin.meanZ*1e-2) + float(self.msgNavSvin.meanZHP*1e-4),
                float(self.msgNavSvin.meanAcc*1e-4)))

class UBXStreamer:
    """
    UBXStreamer class.
    """

    def __init__(self, port, baudrate, timeout=5):
        """
        Constructor.
        """

        self._serial_object = None
        self._serial_thread = None
        self._ubxreader = None
        self._connected = False
        self._reading = False
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout

    def __del__(self):
        """
        Destructor.
        """

        self.stop_read_thread()
        self.disconnect()

    def connect(self):
        """
        Open serial connection.
        """

        try:
            self._serial_object = Serial(
                self._port, self._baudrate, timeout=self._timeout
            )
            self._ubxreader = UBXReader(
                BufferedReader(self._serial_object), validate=VALCKSUM
            )
            self._connected = True
        except (SerialException, SerialTimeoutException) as err:
            print(f"Error connecting to serial port {err}")

    def disconnect(self):
        """
        Close serial connection.
        """

        if self._connected and self._serial_object:
            try:
                self._serial_object.close()
            except (SerialException, SerialTimeoutException) as err:
                print(f"Error disconnecting from serial port {err}")
        self._connected = False

    def start_read_thread(self, recent_val):
        """
        Start the serial reader thread.
        """

        if self._connected:
            self._reading = True
            self._serial_thread = Thread(target=self._read_thread, daemon=True, args=(recent_val,))
            self._serial_thread.start()

    def stop_read_thread(self):
        """
        Stop the serial reader thread.
        """

        if self._serial_thread is not None:
            self._reading = False

    def send(self, data):
        """
        Send data to serial connection.
        """

        self._serial_object.write(data)

    def flush(self):
        """
        Flush input buffer
        """

        self._serial_object.reset_input_buffer()

    def waiting(self):
        """
        Check if any messages remaining in the input buffer
        """

        return self._serial_object.in_waiting

    def _read_thread(self, recent_val):
        """
        THREADED PROCESS
        Reads and parses UBX message data from stream
        """

        sleep(5) #allow the other thread to print init info first
        last_print = 0
        while self._reading and self._serial_object:
            if self._serial_object.in_waiting:
                now = time()
                try:
                    (_, parsed_data) = self._ubxreader.read()
                    if parsed_data:
                        print(parsed_data)
                        if parsed_data.identity == "NAV-SVIN":
                            recent_val.msgNavSvin = parsed_data
                            if (not recent_val.svin_has_been_active) and (parsed_data.active == 1):
                                recent_val.svin_has_been_active = True
                        elif parsed_data.identity == "NAV-PVT":
                            recent_val.msg_nav_pvt = parsed_data
                        if now - last_print > 1:
                            recent_val.print_status(now-last_print)
                            last_print = now
                except (
                    ube.UBXStreamError,
                    ube.UBXMessageError,
                    ube.UBXTypeError,
                    ube.UBXParseError,
                ) as err:
                    print(f"Something went wrong {err}")
                    continue

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--survey_in', nargs=2, type=float,
                        help="Start survey-in of the antenna position for the given minimum duration [minutes] and max allowed accuracy [cm]", 
                        metavar=('duration', 'accuracy'))
    group.add_argument('--fixed_pos_ecef', nargs=4, type=float, 
                        help="ECEF position of the GNSS antenna [m], with accuracy [cm]",
                        metavar=('X', 'Y', 'Z', 'ACC'))
    group.add_argument('--fixed_pos_llh', nargs=4, type=float, 
                        help="Lat [deg] lon [deg] height [m] position of the GNSS antenna, with accuracy [cm]",
                        metavar=('Lat', 'Lon', 'Height', 'LLH ACC'))
    group.add_argument('--mon', type=bool, default=False, help="Only monitor the position, w/o surveying")
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--port', type=str, default="/dev/ttyUSB0")
    args = parser.parse_args()

    PORT = args.port
    BAUDRATE = args.baud
    TIMEOUT = 1
    NMEA = 0
    UBX = 1
    BOTH = 2

    recent_val = DataStore(time())
    print("Instantiating UBXStreamer class...")
    ubp = UBXStreamer(PORT, BAUDRATE, TIMEOUT)
    print(f"Connecting to serial port {PORT} at {BAUDRATE} baud...")
    ubp.connect()
    print("Starting reader thread...")
    ubp.start_read_thread(recent_val)

    
    cfgData = []
    if args.fixed_pos_llh is not None:
        lat = int(math.floor(args.fixed_pos_llh[0]*1e7))
        latHP = int((args.fixed_pos_llh[0]*1e7 - lat)*1e2) # 1e7*1e2 = 1e9
        lon = int(math.floor(args.fixed_pos_llh[1]*1e7))
        lonHP = int((args.fixed_pos_llh[1]*1e7 - lon)*1e2) # 1e7*1e2 = 1e9
        hei = int(math.floor(args.fixed_pos_llh[2]*1e2))
        heiHP = int((args.fixed_pos_llh[2]*1e2 - hei)*1e2) # 1e2*1e2 = 1e4
        acc =  int((args.fixed_pos_llh[3])*1e2) # cm to 0.1mm
        print("\nSetting fixed pos to (LLH) {} [deg] {} [deg] {} [m], acc {} [cm]".format(args.fixed_pos_llh[0],args.fixed_pos_llh[1],args.fixed_pos_llh[2],args.fixed_pos_llh[3]))
        cfgData = [
                ("CFG_TMODE_POS_TYPE", 1), #LLH
                ("CFG_TMODE_MODE", 2),  #Fixed pos
                ("CFG_TMODE_LAT", lat),  #Fixed pos
                ("CFG_TMODE_LAT_HP", latHP),  #Fixed pos
                ("CFG_TMODE_LON", lon),  #Fixed pos
                ("CFG_TMODE_LON_HP", lonHP),  #Fixed pos
                ("CFG_TMODE_HEIGHT", hei),  #Fixed pos
                ("CFG_TMODE_HEIGHT_HP", heiHP),  #Fixed pos
                ("CFG_TMODE_FIXED_POS_ACC", acc)]
    elif args.fixed_pos_ecef is not None:
        x = int(math.floor(args.fixed_pos_ecef[0]*1e2))
        xHP = int((args.fixed_pos_ecef[0]*1e2 - x)*1e2) #1e2*1e2 = 1e4
        y = int(math.floor(args.fixed_pos_ecef[0]*1e2))
        yHP = int((args.fixed_pos_ecef[0]*1e2 - y)*1e2) #1e2*1e2 = 1e4
        z = int(math.floor(args.fixed_pos_ecef[0]*1e2))
        zHP = int((args.fixed_pos_ecef[0]*1e2 - z)*1e2) #1e2*1e2 = 1e4
        acc =  int((args.fixed_pos_ecef[3])*1e2) # cm to 0.1mm
        print("\nSetting fixed pos to (ECEF) {} [m] {} [m] {} [m], acc {} [cm]".format(args.fixed_pos_ecef[0],args.fixed_pos_ecef[1],args.fixed_pos_ecef[2],args.fixed_pos_ecef[3]))
        cfgData = [
                ("CFG_TMODE_POS_TYPE", 1), #ECEF
                ("CFG_TMODE_MODE", 2),  #Fixed pos
                ("CFG_TMODE_ECEF_X", x),
                ("CFG_TMODE_ECEF_X_HP", xHP),
                ("CFG_TMODE_ECEF_Y", y),
                ("CFG_TMODE_ECEF_Y_HP", yHP),
                ("CFG_TMODE_ECEF_Z", z),
                ("CFG_TMODE_ECEF_Z_HP", zHP),
                ("CFG_TMODE_FIXED_POS_ACC", acc)]  
    elif args.survey_in is not None:
        print("\nStarting survey-in for minimum {} [s], accuracy goal {} [cm]".format(args.survey_in[0],args.survey_in[1]))
        cfgData = [
                ("CFG_TMODE_MODE", 1),  #Survey-in
                ("CFG_TMODE_SVIN_MIN_DUR", int(args.survey_in[0])),  
                ("CFG_TMODE_SVIN_ACC_LIMIT", int(args.survey_in[1]*1e2))
                ]
    else:
        print("\nOnly monitoring!")

    layer = 1 #RAM
    transaction = 0 # set cfg immediately, see 5.6 in F9P interface description
    msg = UBXMessage.config_set(layer, transaction, cfgData)
    ubp.send(msg.serialize())

    # We are done with setting up. Run forever, while running read
    poll_svin_msg = UBXMessage("NAV","NAV-SVIN",0)
    poll_pvt_msg = UBXMessage("NAV","NAV-PVT",0)
    try:
        while True:
            # Sending an empty message corresponds to polling that message
            ubp.send(poll_svin_msg.serialize())
            ubp.send(poll_pvt_msg.serialize())
            print("Polling NAV-SVIN")
            if recent_val.msgNavSvin.active:
                sleep(10)
            elif recent_val.svin_has_been_active:
                sleep(100)
            else:
                sleep(20)

    except KeyboardInterrupt:
        print("\n\nStopping reader thread...\n\n")
        ubp.stop_read_thread()
        print("Disconnecting from serial port...")
        ubp.disconnect()
        sys.exit()
