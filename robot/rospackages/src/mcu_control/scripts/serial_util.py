# setup serial communications by searching for arm teensy if USB, or simply connecting to UART
# baud: baudrate, type: node type
# baudrate & type:
# PdsNode = 9600, 'PDS'
# ArmNode = 115200, 'arm'
# ScienceNode = 115200, 'science'
# RoverNode = 115200, 'Astro'

def attempt_usb():
    return 0

def attempt_uart():
    return 0

PROTOCOL_USB = 1
PROTOCOL_UART = 2

def init_serial(baudrate, type):
    rospy.loginfo('Using %d baud by default', baudrate)
    cmd_args = rospy.myargv(argv=sys.argv)
    if len(cmd_args) == 1:
        protocol = PROTOCOL_UART
        rospy.loginfo('Using UART by default')
    elif len(cmd_args) > 1:
        if cmd_args[1] == 'usb':
            protocol = PROTOCOL_USB
        else:
            rospy.logerr('Incorrect argument: expecting "usb" or "uart"')
            sys.exit(0)

    global ser

    ports = list(serial.tools.list_ports.comports())

    startConnecting = time.time()
    if usb:
        #Change baudrate for PdsNode.py
        if type == 'PDS':
            baudrate = 19200
        if len(ports) > 0:
            rospy.loginfo("%d USB device(s) detected", len(ports))
            for portObj in ports:
                port = portObj.name
                rospy.loginfo('Attempting to connect to /dev/'+port)
                ser = serial.Serial('/dev/' + port, baudrate)

                rospy.loginfo("clearing buffer...")
                while ser.in_waiting:
                    ser.readline()

                attempts = 5
                rospy.loginfo("identifying MCU by sending 'who' every %d ms", timeout*1000)
                for i in range(attempts):
                    rospy.loginfo('attempt #%d...', i+1)
                    startListening = time.time()
                    ser.write(str.encode('who\n'))
                    while (time.time()-startListening < timeout):
                        if ser.in_waiting: # if there is data in the serial buffer
                            response = ser.readline().decode()
                            rospy.loginfo('response: '+response)
                            if mcuName in response:
                                rospy.loginfo(mcuName+" MCU identified!")
                                rospy.loginfo('timeout: %f ms', (time.time()-startListening)*1000)
                                rospy.loginfo('took %f ms to find the '+mcuName+' MCU', (time.time()-startConnecting)*1000)
                                if type == 'Astro':
                                    ser.reset_input_buffer()
                                    ser.reset_output_buffer()
                                return
        else:
            rospy.logerr("No USB devices recognized, exiting")
            sys.exit(0)

    elif uart:
        port = 'ttySAC0'
        rospy.loginfo('Attempting to connect to /dev/'+port)
        try:
            ser = serial.Serial('/dev/' + port, baudrate)
        except:
            rospy.logerr('No UART device recognized, terminating arm node')
            sys.exit(0)

        rospy.loginfo("clearing buffer...")
        while ser.in_waiting:
            ser.readline()

        rospy.loginfo("identifying MCU by sending 'who' every %d ms", timeout*1000)
        attempts = 5
        for i in range(attempts):
            rospy.loginfo('attempt #%d...', i+1)
            startListening = time.time()
            ser.write(str.encode('who\n'))
            while (time.time()-startListening < timeout):
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
                            rospy.loginfo('took %f ms to find the '+mcuName+' MCU', (time.time()-startConnecting)*1000)
                            return
                    else:
                        rospy.loginfo('got raw message: '+dat)

    rospy.logerr('Incorrect MCU connected, terminating listener')
    sys.exit(0)
