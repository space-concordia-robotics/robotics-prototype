import serial

# Master device's address. 
# Deserializer will only search for this address. 
DEVADDR = 0x00

# CCITT CRC16, including start, excluding stop
def CRC16(buffer, start, stop):
    poly = 0x1021
    crc = 0xFFFF
    if stop < 0:
        stop = len(buffer)
    for i in range(start, stop):
        crc = crc ^ (buffer[i] << 8)
        for j in range(0, 8):
            if crc & 0x8000 != 0:
                crc = poly ^ ((crc << 1) & 0xFFFF)
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# Represents a packet that can be sent or received on the bus
class Packet:
    def __init__(self, addr = 0, pid = None, payload = []): 
        if isinstance(addr, Packet): 
            self.pid     = addr.pid
            self.addr    = addr.addr
            self.payload = addr.payload
        else:
            if pid == None:
                self.pid = None
            else: 
                self.pid = pid & 0xFF
            self.addr = addr & 0xF
            if payload == None:
                payload = []
            self.payload = payload
    
    def setPID(self, pid): 
        if pid == None:
            self.pid = None
        else: 
            self.pid = pid & 0xFF
        return self

    def getPID(self):
        return self.pid

    def setAddr(self, addr):
        self.addr = addr & 0xF
        return self
    
    def getAddr(self):
        return self.addr

    def setPayload(self, payload):
        if payload == None:
            payload = []
        self.payload = payload
        return self

    def getPayload(self):
        return self.payload

    def getLength(self):
        if self.pid == None:
            return 0
        return len(self.payload) + 1

    def serialize(self):
        length = self.getLength()
        ba = bytearray(length + 6)
        ba[0] = 0x55
        ba[1] = 0x77
        ba[2] = ((~self.addr << 4) & 0xFF) | self.addr
        ba[3] = length
        if self.pid != None:
            ba[4] = self.pid
            for i in range(0, len(self.payload)):
                ba[5 + i] = self.payload[i] & 0xFF
        crc16 = CRC16(ba, 2, length + 4)
        ba[length + 4] = crc16 >> 8
        ba[length + 5] = crc16 & 0xFF
        return ba
    
    def send(self, serialHandle, addr = None):
        if addr != None: 
            self.setAddr(addr)
        return InboundPacketHandler(OutboundPacketHandler(serialHandle, self))
    
    def str(self):
        string = f"Packet @{self.getAddr()}"
        if self.getLength() == 0:
            string = string + " (ZLP)"
        else: 
            string = string + f" PID={self.getPID()}: {self.getPayload()}"
        return string

# Packet Deserializer accepts characters and events(timeout) through its accept() method. 
# The accept method returns a packet object when a valid packet is found. 
class PacketDeserializer:
    def __init__(self, devaddr = None):
        self.state = 0
        self.addr = 0
        self.length = 0
        self.index = 0
        self.payload = None
        if devaddr != None: 
            self.devaddr = devaddr & 0xF
        else: 
            self.devaddr = None

    def _acceptData(self, data):
        if data < 0 or data > 255:
            return None
        valid = False
        if self.state == 0:
            if data == 0x55:
                self.state = 1
        elif self.state == 1:
            if data == 0x77:
                self.state = 2
            else: 
                self.state = 0
        elif self.state == 2:
            self.addr = data
            self.state = 3
        elif self.state == 3:
            self.length = data
            if data == 0:
                self.index = 0
                self.payload = None
                self.state = 5
            else: 
                self.index = 0
                self.payload = bytearray(self.length)
                self.state = 4
        elif self.state == 4:
            self.payload[self.index] = data
            self.index = self.index + 1
            if self.index >= self.length:
                self.state = 5
        elif self.state == 5:
            self.crc = data << 8
            self.state = 6
        elif self.state == 6:
            self.crc = self.crc | data;
            self.state = 0
            valid = True
        else: 
            self.state = 0
        if valid == False:
            return None
        if ((~self.addr) >> 4) & 0xF != self.addr & 0xF:
            return None
        if self.devaddr != None and self.devaddr != self.addr & 0xF: 
            return None
        header = bytearray(2)
        header[0] = self.addr
        header[1] = self.length
        if self.payload != None:
            header = header + self.payload
        crc = CRC16(header, 0, -1)
        if crc != self.crc:
            return None
        packet = Packet(self.addr, None, [])
        if self.length > 0:
            packet.setPID(self.payload[0])
            packet.setPayload(self.payload[1:])
        return packet

    def _acceptTimeout(self):
        self.state = 0
    
    def accept(self, data = None):
        if data == None: 
            self._acceptTimeout()
            return None
        if isinstance(data, int): 
            return self._acceptData(data)
        if len(data) == 0: 
            self._acceptTimeout()
            return None
        results = []
        for i in range(0, len(data)):
            result = self._acceptData(data[i])
            if result != None:
                results.append(result)
        if len(results) == 0:
            return None
        if len(results) == 1:
            return results[0]
        return results
    
    def str(self):
        string = f"PacketDeserializer at state {self.state}"
        return string

# Specific packet implementations

#define PacketOutPing_ID 1
#define PacketOutNVM_ID 2
#define PacketOutSetLED_ID 3
#define PacketOutSetSwitchChannel_ID 4
#define PacketOutSetFan_ID 5
#define PacketOutReadSwitchChannel_ID 6
#define PacketOutReadTempChannel_ID 7
#define PacketOutReadBatteryChannel_ID 8
#define PacketOutSetChannelOCPThresh_ID 9
#define PacketOutSetBatteryODPThresh_ID 10

#define PacketInStatus_ID 0
#define PacketInPing_ID 1
#define PacketInNVM_ID 2
#define PacketInReadSwitchChannel_ID 6
#define PacketInReadTempChannel_ID 7
#define PacketInReadBatteryChannel_ID 8

# OUTBOUND PACKETS
# PacketOutPoll -> PacketInStatus
# Reads the device's fault flags, 
# including 6 overcurrent flags and 1 undervoltage flag. 
# On the device side flags are reset after read out. 
# The packet has zero payload. 
class PacketOutPoll(Packet):
    PID = None
    def __init__(self, addr = 0): 
        super().__init__(addr)

# PacketOutPing -> PacketInPing
# Testing device's presence and responsiveness. 
# Device will respond with the same payload in an inbound ping packet. 
class PacketOutPing(Packet):
    PID = 1
    def __init__(self, addr = 0):
        super().__init__(addr, PacketOutPing.PID, [])

    def setPingContent(self, content):
        if content == None:
            content = []
        super().setPayload(content)
        return self

# PacketOutNVMRead/Write -> PacketInNVM
# The two outbound packets are essentially a same packet with same PID. 
# Reads or writes the device's EEPROM (NVM) at a specific address
# If the address is negative, it accesses the X last element in EEPROM
class PacketOutNVMRead(Packet): 
    PID = 2
    def __init__(self, addr = 0): 
        super().__init__(addr, PacketOutNVMRead.PID, [0, 0])
    
    def read(self, addr): 
        super().getPayload()[0] = (addr >> 8) & 0xFF
        super().getPayload()[1] = (addr     ) & 0xFF
        return self

class PacketOutNVMWrite(Packet): 
    PID = 2
    def __init__(self, addr = 0): 
        super().__init__(addr, PacketOutNVMWrite.PID, [0, 0, 0])
    
    def write(self, addr, data): 
        super().getPayload()[0] = (addr >> 8) & 0xFF
        super().getPayload()[1] = (addr     ) & 0xFF
        super().getPayload()[2] = data & 0xFF
        return self

# PacketOutSetLED -> PacketInAcknowledgement
# Sets the device's status LED
class PacketOutSetLED(Packet): 
    PID = 3
    def __init__(self, addr = 0): 
        super().__init__(addr, PacketOutSetLED.PID, [0])
    
    def setOn(self): 
        super().getPayload()[0] = 1
        return self
    
    def setOff(self): 
        super().getPayload()[0] = 0
        return self
    
    def setToggle(self): 
        super().getPayload()[0] = 2
        return self

# PacketOutSetSwitchChannel -> PacketInAcknowledgement
# Sets the device's power switch channels
# Some channels will be set ON, some set OFF, and the remaining STAYs at its previous value. 
# By default all channels STAYs and does not change. 
# The set methods also accept lists. 
class PacketOutSetSwitchChannel(Packet):
    PID = 4
    def __init__(self, addr = 0):
        super().__init__(addr, PacketOutSetSwitchChannel.PID, [0, 0])

    def _setOn(self, channel):
        if channel < 0 or channel > 6:
            return self
        super().getPayload()[0] = super().getPayload()[0] | (1 << channel)
        super().getPayload()[1] = super().getPayload()[1] &~(1 << channel)
        return self

    def _setOff(self, channel):
        if channel < 0 or channel > 6:
            return self
        super().getPayload()[0] = super().getPayload()[0] &~(1 << channel)
        super().getPayload()[1] = super().getPayload()[1] | (1 << channel)
        return self

    def _setStay(self, channel): 
        if channel < 0 or channel > 6:
            return self
        super().getPayload()[0] = super().getPayload()[0] &~(1 << channel)
        super().getPayload()[1] = super().getPayload()[1] &~(1 << channel)
        return self

    def setOn(self, channel):
        if channel == None:
            return self
        if isinstance(channel, int):
            self._setOn(channel)
            return self
        for i in range(0, len(channel)):
            self._setOn(channel[i])
        return self

    def setOff(self, channel):
        if channel == None:
            return self
        if isinstance(channel, int):
            self._setOff(channel)
            return self
        for i in range(0, len(channel)):
            self._setOff(channel[i])
        return self

    def setStay(self, channel):
        if channel == None:
            return self
        if isinstance(channel, int):
            self._setStay(channel)
            return self
        for i in range(0, len(channel)):
            self._setStay(channel[i])
        return self

# PacketOutSetFan -> PacketInAcknowledgement
# Sets the device's fan to ON or OFF. 
class PacketOutSetFan(Packet): 
    PID = 5
    def __init__(self, addr = 0): 
        super().__init__(addr, PacketOutSetFan.PID, [0])
    
    def setOn(self): 
        super().getPayload()[0] = 1
        return self
    
    def setOff(self): 
        super().getPayload()[0] = 0
        return self

# PacketOutReadSwitchChannel -> PacketInReadSwitchChannel
# Reads the device's six power switch channels
# Currently the value is RAW and needs to be processed on the master side
class PacketOutReadSwitchChannel(Packet): 
    PID = 6
    def __init__(self, addr = 0): 
        super().__init__(addr, PacketOutReadSwitchChannel.PID, [])

# PacketOutReadTempChannel -> PacketInReadTempChannel
# Reads the device's five temperature channels
# Currently the value is RAW and needs to be processed on the master side
class PacketOutReadTempChannel(Packet): 
    PID = 7
    def __init__(self, addr = 0): 
        super().__init__(addr, PacketOutReadTempChannel.PID, [])

# PacketOutReadBatteryChannel -> PacketInReadBatteryChannel
# Reads the device's battery level
# Currently the value is RAW and needs to be processed on the master side
class PacketOutReadBatteryChannel(Packet): 
    PID = 8
    def __init__(self, addr = 0): 
        super().__init__(addr, PacketOutReadBatteryChannel.PID, [])

# PacketOutSetChannelOCPThresh -> PacketInAcknowledgement
# Sets the device's power switch protection threshold. 
# Only one channel can be set at a time with one packet (unlike setting power switches)
# Set to 0 to disable protection. 
# The value is also RAW and needs preprocessing. 
class PacketOutSetChannelOCPThresh(Packet): 
    PID = 9
    def __init__(self, addr = 0): 
        super().__init__(addr, PacketOutSetChannelOCPThresh.PID, [0, 0, 0])
    
    def clearLimit(self, channel): 
        self.setLimit(channel, 0)
    
    def setLimit(self, channel, limit): 
        super().getPayload()[0] = channel & 0xFF
        super().getPayload()[1] = (limit >> 8) & 0xFF
        super().getPayload()[2] =  limit       & 0xFF
        return self

# PacketOutSetBatteryODPThresh -> PacketInAcknowledgement
# Sets the device's over discharge protection threshold. 
# Set to 0 to disable protection. 
# The value is also RAW and needs preprocessing. 
class PacketOutSetBatteryODPThresh(Packet): 
    PID = 10
    def __init__(self, addr = 0): 
        super().__init__(addr, PacketOutSetBatteryODPThresh.PID, [0, 0])

    def setLimit(self, limit): 
        super().getPayload()[0] = (limit >> 8) & 0xFF
        super().getPayload()[1] =  limit       & 0xFF
        return self

# INBOUND PACKETS
# General acknowledgement packet with zero payload
class PacketInAcknowledgement(Packet): 
    PID = None
    def __init__(self, packet): 
        if packet.getPID() != PacketInAcknowledgement.PID or packet.getLength() != 0: 
            raise Exception()
        super().__init__(packet)

    def isOkay(self):
        return True

# Response to PacketOutPoll, contains device fault flags. 
class PacketInStatus(Packet): 
    PID = 0
    LEN = 2
    def __init__(self, packet): 
        if packet.getPID() != PacketInStatus.PID or len(packet.getPayload()) != PacketInStatus.LEN: 
            raise Exception()
        super().__init__(packet)
        faultyChannel = []
        for i in range(0, 6): 
            if (packet.getPayload()[0] & (1 << i)) != 0: 
                faultyChannel.append(i)
        self.faultyChannel = faultyChannel
        self.faultyBattery = packet.getPayload()[1] != 0
    
    def getFaultyChannel(self): 
        return self.faultyChannel
    
    def isFaultyBattery(self): 
        return self.faultyBattery

# Response to PacketOutPing, echos the payload back. 
class PacketInPing(Packet): 
    PID = 1
    def __init__(self, packet): 
        if packet.getPID() != PacketInPing.PID: 
            raise Exception()
        super().__init__(packet)
    
    def getPingContent(self): 
        return list(super().getPayload())

# Response to PacketOutNVMRead/Write, containing operation outcome and accessed data. 
class PacketInNVM(Packet): 
    PID = 2
    LEN = 4
    def __init__(self, packet): 
        if packet.getPID() != PacketInNVM.PID or len(packet.getPayload()) != PacketInNVM.LEN: 
            raise Exception()
        super().__init__(packet)
        self.success = packet.getPayload()[0] != 0
        self.nvmaddr = packet.getPayload()[1] << 8 | packet.getPayload()[2]
        self.nvmdata = packet.getPayload()[3]

    def isSuccessful(self): 
        return self.success
    
    def accessAddr(self): 
        return self.nvmaddr
    
    def accessData(self): 
        return self.nvmdata

# Response to PacketOutReadSwitchChannel, containing 6 current readings. 
# The readings are currently RAW and need to be processed. 
class PacketInReadSwitchChannel(Packet): 
    PID = 6
    LEN = 12
    def __init__(self, packet): 
        if packet.getPID() != PacketInReadSwitchChannel.PID or len(packet.getPayload()) != PacketInReadSwitchChannel.LEN: 
            raise Exception()
        super().__init__(packet)
        meas = []
        for i in range(0, 6): 
            meas.append(packet.getPayload()[2 * i] << 8 | packet.getPayload()[2 * i + 1])
        self.measurement = meas
    
    def getMeasurement(self, channel = None): 
        if channel == None:
            return self.measurement
        else: 
            return self.measurement[channel]

# Response to PacketOutReadTempChannel, containing 5 temperature readings. 
# Channel 0 is board temperature. 
# The readings are currently RAW and need to be processed. 
class PacketInReadTempChannel(Packet): 
    PID = 7
    LEN = 10
    def __init__(self, packet): 
        if packet.getPID() != PacketInReadTempChannel.PID or len(packet.getPayload()) != PacketInReadTempChannel.LEN: 
            raise Exception()
        super().__init__(packet)
        meas = []
        for i in range(0, 5): 
            meas.append(packet.getPayload()[2 * i] << 8 | packet.getPayload()[2 * i + 1])
        self.measurement = meas
    
    def getMeasurement(self, channel = None): 
        if channel == None:
            return self.measurement
        else: 
            return self.measurement[channel]

# Response to PacketOutReadBatteryChannel, containing 1 voltage reading. 
# The readings are currently RAW and need to be processed. 
class PacketInReadBatteryChannel(Packet): 
    PID = 8
    LEN = 2
    def __init__(self, packet): 
        if packet.getPID() != PacketInReadBatteryChannel.PID or len(packet.getPayload()) != PacketInReadBatteryChannel.LEN: 
            raise Exception()
        super().__init__(packet)
        self.measurement = packet.getPayload()[0] << 8 | packet.getPayload()[1]
    
    def getMeasurement(self): 
        return self.measurement

# Transaction logic manager
def OutboundPacketHandler(serialHandle, outPacket):
    ps = PacketDeserializer(DEVADDR)
    serialHandle.reset_input_buffer()
    serialHandle.reset_output_buffer()
#   Enable Transmitter
    serialHandle.write(outPacket.serialize())
    serialHandle.flush()
#   Disable transmitter
    inPacket = None
    while True:
        data = serialHandle.read()
        if len(data) == 0:
            break
        inPacket = ps.accept(data)
        if inPacket != None: 
            break
    if inPacket == None: 
        return None
    return inPacket

# Inbound packet factory
def InboundPacketHandler(packet):
    if packet == None:
        return None
    list = [
        PacketInAcknowledgement, 
        PacketInStatus, 
        PacketInPing, 
        PacketInNVM, 
        PacketInReadSwitchChannel, 
        PacketInReadTempChannel, 
        PacketInReadBatteryChannel, 
    ]
    pid = packet.getPID()
    packet0 = None
    for clazz in list: 
        if clazz.PID != pid:
            continue
        try: 
            packet0 = clazz(packet)
            break
        except:
            packet0 = None
            continue
    return packet0


# Your application code

# pySerial port object
# The timeout here is the actual serial packet timeout and cannot be omitted
port = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1) 

# Usage: 
# outPacket = PacketOutXXXX()
# outPacket.method(..)
# outPacket.method(..)
# inPacket = outPacket.send(port, devaddr)
# isinstance(inPacket, PacketInXXXX) # optional
# .. = inPacket.method(..)
# .. = inPacket.method(..)

# Simplified syntax: 
# PacketOutXXXX().method(..).method(..).send(port, devaddr).method(..)

# Try these examples: 

# PacketOutSetLED().setToggle().send(port, 1).isOkay()

# PacketOutSetSwitchChannel().setOn([0,1,2,3,4,5]).send(port, 1).isOkay()

# PacketOutReadSwitchChannel().send(port, 1).getMeasurement()

# PacketOutNVMRead().read(-1).send(port, 1).accessData()

# PacketOutPing().setPingContent([12, 34, 56, 78]).send(port, 1).getPingContent()

# PacketOutPoll().send(port, 1).getFaultyChannel()

