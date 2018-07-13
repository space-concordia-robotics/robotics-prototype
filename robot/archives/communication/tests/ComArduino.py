# 12 Mar 2014

# in case any of this upsets Python purists it has been converted from an equivalent JRuby program

# this is designed to work with ... ArduinoPC.ino ...

# the purpose of this program and the associated Arduino program is to demonstrate a system for sending 
#   and receiving data between a PC and an Arduino.

# The key functions are:
#    sendToArduino(str) which sends the given string to the Arduino. The string may 
#                       contain characters with any of the values 0 to 255
#
#    recvFromArduino()  which returns an array. 
#                         The first element contains the number of bytes that the Arduino said it included in
#                             message. This can be used to check that the full message was received.
#                         The second element contains the message as a string


# the overall process followed by the demo program is as follows
#   open the serial connection to the Arduino - which causes the Arduino to reset
#   wait for a message from the Arduino to give it time to reset
#   loop through a series of test messages
#      send a message and display it on the PC screen
#      wait for a reply and display it on the PC

# to facilitate debugging the Arduino code this program interprets any message from the Arduino
#    with the message length set to 0 as a debug message which is displayed on the PC screen

# the actual process of sending a message to the Arduino involves
#   prefacing the message with a byte value of 254 (startMarker)
#   following that startMarker with a byte whose value is the number of characters in the original message
#   then the message follows
#      any bytes in the message with values of 253, 254 or 255 into a pair of bytes
#          253 0    253 1   or 253 2       as appropriate
#   suffixing the message with a byte value of 255 (endMarker)


# receiving a message from the Arduino involves
#    waiting until the startMarker is detected
#    saving all subsequent bytes until the end marker is detected
#    converting the pairs of bytes (253 0 etc) back into the intended single byte



# NOTES
#       this program does not include any timeouts to deal with delays in communication
#
#       for simplicity the program does NOT search for the comm port - the user must modify the
#         code to include the correct reference.
#         search for the line "ser = serial.Serial("/dev/ttyS80", 57600)"
#
#       the function bytesToString(str) is just a convenience to show the contents of a string as
#          a series of byte values to make it easy to verify data with non-ascii characters
#
#       this program does NOT include a checkbyte that could be used to verify that there are no
#          errors in the message. This could easily be added.
#
#       as written the Arduino program can only receive a maximum of 16 bytes. 
#          This must include the start- and end-markers, the length byte and any extra bytes needed 
#             to encode values of 253 or over
#          the arduino program could easily be modified to accept longer messages by changing
#                #define maxMessage 16
#
#       as written the Arduino program does NOT check for messages that are too long
#         it is assumed that the PC program will ensure compliance
#         extra code could be added to the Arduino program to deal with too-long messages
#           but it would add a lot of code that may confuse this demo.

#=====================================

#  Function Definitions

#=====================================

def sendToArduino(sendStr):
  global startMarker, endMarker
  txLen = chr(len(sendStr))
  adjSendStr = encodeHighBytes(sendStr)
  adjSendStr = chr(startMarker) + txLen + adjSendStr + chr(endMarker)
  ser.write(adjSendStr)


#======================================

def recvFromArduino():
  global startMarker, endMarker
  
  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many
  
  # wait for the start character
  while  ord(x) != startMarker: 
    x = ser.read()
  
  # save data until the end marker is found
  while ord(x) != endMarker:
    ck = ck + x 
    x = ser.read()
    byteCount += 1
    
  # save the end marker byte
  ck = ck + x 
  
  returnData = []
  returnData.append(ord(ck[1]))
  returnData.append(decodeHighBytes(ck))
#  print "RETURNDATA " + str(returnData[0])
  
  return(returnData)

#======================================

def encodeHighBytes(inStr):
  global specialByte
  
  outStr = ""
  s = len(inStr)
  
  for n in range(0, s):
    x = ord(inStr[n])
    
    if x >= specialByte:
       outStr = outStr + chr(specialByte)
       outStr = outStr + chr(x - specialByte)
    else:
       outStr = outStr + chr(x)
       
#  print "encINSTR  " + bytesToString(inStr)
#  print "encOUTSTR " + bytesToString(outStr)

  return(outStr)


#======================================

def decodeHighBytes(inStr):

  global specialByte
  
  outStr = ""
  n = 0
  
  while n < len(inStr):
     if ord(inStr[n]) == specialByte:
        n += 1
        x = chr(specialByte + ord(inStr[n]))
     else:
        x = inStr[n]
     outStr = outStr + x
     n += 1
     
  print "decINSTR  " + bytesToString(inStr)
  print "decOUTSTR " + bytesToString(outStr)

  return(outStr)


#======================================

def displayData(data):

  n = len(data) - 3

  print "NUM BYTES SENT->   " + str(ord(data[1]))
  print "DATA RECVD BYTES-> " + bytesToString(data[2:-1])
  print "DATA RECVD CHARS-> " + data[2: -1]


#======================================

def bytesToString(data):

  byteString = ""
  n = len(data)
  
  for s in range(0, n):
    byteString = byteString + str(ord(data[s]))
    byteString = byteString + "-"
    
  return(byteString)


#======================================

def displayDebug(debugStr):

   n = len(debugStr) - 3
   print "DEBUG MSG-> " + debugStr[2: -1]


#============================

def waitForArduino():

   # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
   # it also ensures that any bytes left over from a previous message are discarded
   
    global endMarker
    
    msg = ""
    while msg.find("Arduino Ready") == -1:

      while ser.inWaiting() == 0:
        x = 'z'

      # then wait until an end marker is received from the Arduino to make sure it is ready to proceed
      x = "z"
      while ord(x) != endMarker: # gets the initial debugMessage
        x = ser.read()
        msg = msg + x


      displayDebug(msg)
      print
      

#======================================

# THE DEMO PROGRAM STARTS HERE

#======================================

import serial
import serial.tools.list_ports
import time

# NOTE: just takes the first available com port, does not give option to select from multiple available ports yet
ports = list(serial.tools.list_ports.comports())
firstPortName = ports[0].name

ser = serial.Serial("/dev/" + firstPortName, 57600)


startMarker = 254
endMarker = 255
specialByte = 253


waitForArduino()

print "Arduino is ready"

testData = []
testData.append("abcde")
testData.append("zxcv1234")
testData.append("a" + chr(16) + chr(32) + chr(0)  + chr(203))
testData.append("b" + chr(16) + chr(32) + chr(253) + chr(255) + chr(254) + chr(253) + chr(0))
testData.append("fghijk")

numLoops = len(testData)
n = 0
waitingForReply = False

while n < numLoops:
  print "LOOP " + str(n)
  teststr = testData[n]

  if ser.inWaiting() == 0 and waitingForReply == False:
    sendToArduino(teststr)
    print "=====sent from PC=========="
    print "LOOP NUM " + str(n)
    print "BYTES SENT -> " + bytesToString(teststr)
    print "TEST STR " + teststr
    print "==========================="
    waitingForReply = True

  if ser.inWaiting > 0:
    dataRecvd = recvFromArduino()

    if dataRecvd[0] == 0:
      displayDebug(dataRecvd[1])

    if dataRecvd[0] > 0:
      displayData(dataRecvd[1])
      print "Reply Received"
      n += 1
      waitingForReply = False

    print
    print "==========="
    print

    time.sleep(0.3)

ser.close

