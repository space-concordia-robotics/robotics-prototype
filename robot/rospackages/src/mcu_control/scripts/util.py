# setup serial communications by searching for arm teensy if USB, or simply connecting to UART
#baud: initial baudrate, baud: baudrate for usb, type: node type
#Baudrates:
#PdsNode = 9600
#ArmNode = 115200

def init_serial(init_baud, type):
    baudrate = baud
    # in a perfect world, you can choose the baudrate
    rospy.loginfo('Using %d baud by default', baudrate)
    # in a perfect world, usb vs uart will be set by ROS params
    usb = False; uart = True
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) == 1:
        rospy.loginfo('Using UART by default')
    if len(myargv) > 1:
        if myargv[1] == 'uart':
            usb=False; uart=True
            rospy.loginfo('Using UART and 115200 baud by default')
        elif myargv[1] == 'usb':
            usb=True; uart=False
            rospy.loginfo('Using USB and 115200 baud by default')
        else:
            rospy.logerr('Incorrect argument: expecting "usb" or "uart"')
            sys.exit(0)

    global ser #make global so it can be used in other parts of the code

    ports = list(serial.tools.list_ports.comports())

    startConnecting = time.time()
    #Change baudrate for PdsNode.py
    if type = 'PDS':
        baudrate = 19200
    if usb:
        if len(ports) > 0:
            rospy.loginfo("%d USB device(s) detected", len(ports))
            for portObj in ports:
                port = portObj.name
                rospy.loginfo('Attempting to connect to /dev/'+port)
                ser = serial.Serial('/dev/' + port, baudrate)

                rospy.loginfo("clearing buffer...")
                while ser.in_waiting:
                    ser.readline()

                rospy.loginfo("identifying MCU by sending 'who' every %d ms", timeout*1000)
                for i in range(5):
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
        for i in range(1,6):
            rospy.loginfo('attempt #%d...', i)
            startListening = time.time()
            ser.write(str.encode('who\n'))
            while (time.time()-startListening < timeout):
                if ser.in_waiting:
                    dat='';data=None
                    try:
                        dat = ser.readline().decode()
                        if type = arm:
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
