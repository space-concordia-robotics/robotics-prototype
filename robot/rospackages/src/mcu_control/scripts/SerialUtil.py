# setup serial communications by searching for arm teensy if USB, or simply connecting to UART
# baud: baudrate, mcuName
# baudrate & mcuName:
# PdsNode = 9600, 'PDS'
# ArmNode = 115200, 'arm'
# ScienceNode = 115200, 'science'
# RoverNode = 115200, 'Astro'

import rospy
import time
import sys
import serial
import serial.tools.list_ports

PROTOCOL_USB = 1
PROTOCOL_UART = 2

COMMUNICATION_ATTEMPTS = 3
COMM_TIMEOUT = 0.3 # 300 ms

ser = None

def get_serial():
    return ser

def attempt_usb(mcuName):
    startListening = time.time() 
    ser.write(str.encode('who\n'))
    while (time.time()-startListening < COMM_TIMEOUT):
        if ser.in_waiting:
            response = ser.readline().decode()
            rospy.loginfo('response: '+response)
            if mcuName in response:
                rospy.loginfo(mcuName+" MCU identified!")
                rospy.loginfo('timeout: %f ms', (time.time()-startListening)*1000)
                # rospy.loginfo('took %f ms to find the '+mcuName+' MCU', (time.time()-startConnecting)*1000)
                if mcuName == 'Astro':
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                return True
    return False

def attempt_uart(mcuName):
    startListening = time.time()
    ser.write(str.encode('who\n'))
    while (time.time()-startListening < COMM_TIMEOUT):
        if ser.in_waiting:
            dat='';data=None
            try:
                dat = ser.readline().decode()
                if type == arm:
                    data = stripFeedback(dat) #not present in pdsnode
            except:
                rospy.logwarn('trouble reading from serial port')
            if data is not None:
                if mcuName in data:
                    rospy.loginfo(mcuName+" MCU identified!")
                    rospy.loginfo('timeout: %f ms', (time.time()-startListening)*1000)
                    # rospy.loginfo('took %f ms to find the '+mcuName+' MCU', (time.time()-startConnecting)*1000)
                    return True
            else:
                rospy.loginfo('got raw message: '+dat)
    return False

def clear_buffer():
    rospy.loginfo("clearing buffer...")
    while ser.in_waiting:
        ser.readline()

def search_uart(baudrate, mcuName):
    global ser
    port = 'ttySAC0'
    rospy.loginfo('Attempting to connect to /dev/'+ port)
    rospy.loginfo('Using %d baud', baudrate)
    try:
        ser = serial.Serial('/dev/' + port, baudrate)
    except:
        rospy.logerr('No UART device recognized, terminating arm node')
        sys.exit(0)

    clear_buffer()

    rospy.loginfo("identifying MCU by sending 'who' every %d ms", COMM_TIMEOUT*1000)
    for i in range(COMMUNICATION_ATTEMPTS):
        rospy.loginfo('attempt #%d...', i+1)
        if attempt_uart(mcuName):
            return True

    return False

def search_usb(baudrate, ports, mcuName):
    global ser
    rospy.loginfo("%d USB device(s) detected", len(ports))
    rospy.loginfo('Using %d baud', baudrate)
    for p in ports:
        port = p.name
        rospy.loginfo('Attempting to connect to /dev/'+port)
        try:
            ser = serial.Serial('/dev/' + port, baudrate)
        except:
            rospy.logerr('No UART device recognized, terminating arm node')
            sys.exit(0)

        clear_buffer()

        rospy.loginfo("identifying MCU by sending 'who' every %d ms", COMM_TIMEOUT*1000)
        for i in range(COMMUNICATION_ATTEMPTS):
            rospy.loginfo('attempt #%d...', i+1)
            if attempt_usb(mcuName):
                return True
    return False

def init_serial(baudrate, mcuName):
    cmd_args = rospy.myargv(argv=sys.argv)
    if len(cmd_args) == 1: 
        if mcuName == 'PDS':
            protocol = PROTOCOL_UART
            rospy.loginfo('Using UART by default')
        else:
            protocol = PROTOCOL_USB
            rospy.loginfo('Using USB by default')
    elif len(cmd_args) > 1:
        if cmd_args[1] == 'usb':
            protocol = PROTOCOL_USB
            rospy.loginfo('Communication will be via USB')
        elif cmd_args[1] == 'uart':
            protocol = PROTOCOL_UART
            rospy.loginfo('Communication will be via UART')
        else:
            rospy.logerr('Incorrect argument: expecting "usb" or "uart"')
            sys.exit(0)

    ports = list(serial.tools.list_ports.comports())
    if len(ports) == 0:
        rospy.logerr("No devices found, exiting")
        sys.exit(0)

    startConnecting = time.time()

    search_success = False
    if protocol == PROTOCOL_USB:
        search_success = search_usb(19200 if mcuName == 'PDS' else baudrate, ports, mcuName)
    elif protocol == PROTOCOL_UART:
        search_success = search_uart(baudrate, mcuName)
    else:
        rospy.logerr('Incorrect MCU connected, terminating listener')
        sys.exit(0)

    if not search_success:
        print("Search Unsuccessful")
