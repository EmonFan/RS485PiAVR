import RPi.GPIO as GPIO
import time, serial, array, struct

PKT_LEADIN_BYTE = 0x99
PKT_START_BYTE  = 0xaa
SLAVE_BASE      = 0xa0
MASTER_ID       = 0x0b
PKTF_ACK        = 0x01
PKTF_NACK       = 0x02
PKTF_REQACK     = 0x04
PKTF_REQID      = 0x08
PKTF_COMMAND    = 0x10
PKTF_RESERVED   = 0x20
PKTF_SENSOR_BAD = 0x40

PKT_N_LEADIN    = 4     #Bytes before PKT_FLAG

PKT_LEADIN      = 0x00
PKT_START       = 0x01
PKT_DEST        = 0x02
PKT_SOURCE      = 0x03
PKT_FLAG        = 0x04
PKT_LENGTH      = 0x05
PKT_DATA        = 0x06
PKT_CHECKSUM    = 0x07
PKT_VALID       = 0x08
PKT_NOT_VALID   = 0x09

#Define the supported slave commands
ENUMERATE_COUNT = 0xD0
ENUMERATE_ITEM  = 0xD1
TRANSMIT_ITEM   = 0xD2

READ_ENABLE = 7
BUFFER_MAX = 60

# to use Raspberry Pi board pin numbers  
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(READ_ENABLE, GPIO.OUT) 

#Slave is running at DOUBLE SPEED
#sp = serial.Serial('/dev/ttyAMA0', 230400, timeout = .05)
#sp = serial.Serial('/dev/ttyAMA0', 115200, timeout = .01)
sp = serial.Serial('/dev/ttyAMA0', 57600, timeout = .1)

#create an unsigned byte array for data reception
receiveBuffer = array.array('B')

#The sensor class represents a single sensor with it's current value
class sensor:
  sensorCount = 0
  
  def __init__(self):
    self.sensorID = array.array('B')
    self.currentValue = ""
    sensor.sensorCount += 1

  def clear(self):
    print "Delete sensor"
    del self.sensorID
    sensor.sensorCount -= 1

#The slave class represents a network slave along with it's associated sensor list
class slave:
  slaveCount = 0
  
  def __init__(self):
    self.slaveID = 0
    self.sensorList = []
    slave.slaveCount += 1

  def clear(self):
    for sensor in self.sensorList:
      sensor.clear()
      
    del self.sensorList[:]
    
slaveList = []

#The data received could be padded with crap. The packetOffset variable will ensure we have the correct start point for valid data
packetOffset = 0  #Needed to skip any bogus bytes ahead of our payload

def checkSum(buffer):
  global packetOffset

  #Data length is the actual length of the packed data + the lead in bytes + any crap data ahead of the lot
  plen = buffer[PKT_LENGTH + packetOffset] + PKT_N_LEADIN + packetOffset

  ckSum = buffer[PKT_DEST + packetOffset]
  i = PKT_DEST + packetOffset + 1

  while (i < plen):
    ckSum += buffer[i]
    i += 1

  #print "CS Raw: " + str(ckSum)
  #our checksum must be a byte
  multiple = ckSum / 255;
  if (multiple >= 1):
    ckSum = ckSum - (255 * multiple)
    ckSum -= multiple

  if (ckSum < 0):
    ckSum = 256 + ckSum
    print "CkSum = " + str(ckSum)
  return ckSum

#Validate a received packet is valid
def validate(slaveID):
  global packetOffset
  global receiveBuffer
  
  length = len(receiveBuffer)

  if (len < 5): #minimum length
    return PKT_NOT_VALID

  try:
    packetOffset = 0  #Needed to skip any bogus bytes ahead of our payload
    while (packetOffset < length):
      if (receiveBuffer[packetOffset+PKT_LEADIN] == PKT_LEADIN_BYTE):
        if (receiveBuffer[packetOffset+PKT_START] == PKT_START_BYTE):
          if (receiveBuffer[packetOffset+PKT_DEST] == MASTER_ID):
            if (receiveBuffer[packetOffset+PKT_SOURCE] == slaveID):
              if (receiveBuffer[packetOffset+PKT_FLAG] == PKTF_SENSOR_BAD): #Hmm, looks like we lost a sensor
                return PKTF_SENSOR_BAD
              #So far so good. Let's verify the checksum
              if (receiveBuffer[ packetOffset+PKT_LENGTH + int(receiveBuffer[packetOffset+PKT_LENGTH]) + 1]== checkSum(receiveBuffer)):
#                print "CS OK"
                return PKT_VALID
              else:
#                print "Expected: " + str(receiveBuffer[ packetOffset+PKT_LENGTH + int(receiveBuffer[packetOffset+PKT_LENGTH]) + 1])
                return PKT_NOT_VALID
            else:
              return PKT_NOT_VALID
          else:
            return PKT_NOT_VALID
        else:
          return PKT_NOT_VALID
              
      packetOffset += 1
    return PKT_NOT_VALID
  except:
    return PKT_NOT_VALID

#We can have up to 8 slaves on bus (for now) ping them all and see which ones exist
def identifySlaves():
  global packetOffset
  global receiveBuffer
  #Let's call them all individually to see who is alive
  index = 5
  while (index < 6):
    #Set the slave address we will ping
    slaveID = SLAVE_BASE + index
    
    #reset our packet offset variable
    packetOffset = 0
    #Ask the slaves to send their ID's
    #Payload in the request is a single byte just to keep things clean
    #Slaves will respond with their ID's but with random delays to prevent collisions
    transmitBuffer = array.array('B', [PKT_LEADIN_BYTE, PKT_START_BYTE, slaveID, MASTER_ID, PKTF_REQID, 0x01, 0x00])
    #transmitBuffer.append(checkSum(transmitBuffer))

    #Enable transmit mode and send the data
    GPIO.output(READ_ENABLE, GPIO.HIGH) 
    print 'SlaveID: ' + str(slaveID)
    #print transmitBuffer
    sp.write(transmitBuffer)
    checkSumBuffer = bytearray(1)
    checkSumBuffer[0] = checkSum(transmitBuffer)
    sp.write(checkSumBuffer)
    sp.flush()

    #enable receive mode
    GPIO.output(READ_ENABLE, GPIO.LOW)
    time.sleep(.02)
    #read up to 60 bytes or timeout. 60 is the max packet length
    response = sp.read(BUFFER_MAX)

    #print response.encode('hex')

    #the response will be a string, we need a byte array.
    receiveBuffer.fromstring(response)

    if (validate(slaveID) == PKT_VALID):
      newSlave = slave()
      newSlave.slaveID = int(response[packetOffset+PKT_DATA].encode('hex'), 16)
      slaveList.append(newSlave)
      print "Slave ID: " + str(newSlave.slaveID)
    else:
      print "Slave not present."
    
    #reset our receive buffer to empty state
    del receiveBuffer[:]
    #delete our transmit buffer, it will be recreated
    del transmitBuffer
    index += 1

#Ask the slave how many sensors are attached
def itemCount(slave):
  global packetOffset
  global receiveBuffer

  #reset our packet offset variable
  packetOffset = 0
  #reset our receive buffer to empty state
  del receiveBuffer[:]

  transmitBuffer = array.array('B', [PKT_LEADIN_BYTE, PKT_START_BYTE,  slave.slaveID, MASTER_ID, PKTF_COMMAND, 0x01, ENUMERATE_COUNT])
  #transmitBuffer.append(checkSum(transmitBuffer))

  #Enable transmit mode and send the data
  GPIO.output(READ_ENABLE, GPIO.HIGH) 
  sp.write(transmitBuffer)
  checkSumBuffer = bytearray(1)
  checkSumBuffer[0] = checkSum(transmitBuffer)
  sp.write(checkSumBuffer)
  sp.flush()

  #enable receive mode
  GPIO.output(READ_ENABLE, GPIO.LOW)
  time.sleep(.02)

  #read up to 60 bytes or timeout. 60 is the max packet length
  response = sp.read(BUFFER_MAX)
  
  print "response = " + response.encode('hex')

  #the response will be a string, we need a byte array.
  receiveBuffer.fromstring(response)

  if (validate(slave.slaveID) == PKT_VALID):
    return receiveBuffer[packetOffset+PKT_DATA]
  else:
    return -1

#Ask the slave to send the sensorID if the command is ENUMERATE_ITEM, else we are asking for the current value of
#a specific sensorID using the sensorID
def getItemData(slave, command, itemIndex):
  global packetOffset
  global receiveBuffer

  #reset our packet offset variable
  packetOffset = 0
  #reset our receive buffer to empty state
  del receiveBuffer[:]

  #print ("Fetch itemIndex = " + str(itemIndex))
  length = 2
  if (command == ENUMERATE_ITEM):
    #print "Enumerate"
    transmitBuffer = array.array('B', [PKT_LEADIN_BYTE, PKT_START_BYTE, slave.slaveID, MASTER_ID, PKTF_COMMAND, length, command, itemIndex])
  else: #TRANSMIT_ITEM
    #print "Transmit"
    length = 9 #Command byte plus 8 byte sensor address
    transmitBuffer = array.array('B', [PKT_LEADIN_BYTE, PKT_START_BYTE, slave.slaveID, MASTER_ID, PKTF_COMMAND, length, command])
    for byte in slave.sensorList[itemIndex].sensorID:
      transmitBuffer.append(byte)
    
  #transmitBuffer.append(checkSum(transmitBuffer))
  
  #Enable transmit mode and send the data
  GPIO.output(READ_ENABLE, GPIO.HIGH) 
  sp.write(transmitBuffer)
  checkSumBuffer = bytearray(1)
  checkSumBuffer[0] = checkSum(transmitBuffer)
  sp.write(checkSumBuffer)
  sp.flush()

  #enable receive mode
  GPIO.output(READ_ENABLE, GPIO.LOW)
  time.sleep(.02)
  #read up to 60 bytes or timeout. 60 is the max packet length
  response = sp.read(BUFFER_MAX)
  
  #the response will be a string, we need a byte array.
  receiveBuffer.fromstring(response)

  newSensor = sensor()
  result = ""
  validation = validate(slave.slaveID)
  if ( validation == PKTF_SENSOR_BAD): 
    return PKTF_SENSOR_BAD  #Hmm, we lost a sensor. We need to re-inventory
    
  if (validation == PKT_VALID):
    #How many bytes are we expecting?
    lastPos = receiveBuffer[packetOffset+PKT_LENGTH] + packetOffset + PKT_DATA
    #Get our payload start position
    index = packetOffset + PKT_DATA

    if (command != ENUMERATE_ITEM): #We are asking for a sensor value. Initialize it to empty
      slave.sensorList[itemIndex].currentValue = ""
      
    while (index < lastPos):
      if (command == ENUMERATE_ITEM): #we are receiving back a sensorID, copy it byte per byte
        newSensor.sensorID.append(receiveBuffer[index])
      else: #we are receiving a sensor value, copy it as a string
        slave.sensorList[itemIndex].currentValue += chr(receiveBuffer[index])
        
      index += 1

    if (command == ENUMERATE_ITEM): #We were enumerating, so add the sensor object to our list
      slave.sensorList.append(newSensor)
      print "Added new sensor: " + newSensor.sensorID.tostring().encode('hex')
  
  return 0 #All good

def slaveSensors(slave):
  slaveItems = itemCount(slave) #How many sensors?
  print "Item count: " + str(slaveItems)
  itemIndex = 0
  while (itemIndex < slaveItems):
    getItemData(slave, ENUMERATE_ITEM, itemIndex)
    #print "ID: " + slave.sensorList[itemIndex].sensorID.tostring().encode('hex')
    #currentTemp = getItemData(slave, TRANSMIT_ITEM, itemIndex)
    #print "Temp: " + str(currentTemp)
    itemIndex += 1

while (len(slaveList) == 0):
  identifySlaves()  #Get the slaves that exist on the network
  time.sleep(1)     #We need at least one slave on the network. Try again in a moment

for slave in slaveList:
  slaveSensors(slave) #Get the sensors that exist on each slave

while (1):
  for slave in slaveList:
    slaveItems = itemCount(slave) #How many sensors?
    #print ("we have " + str(slaveItems) + " sensors" + " for slaveId ")
    itemIndex = 0
    while (itemIndex < slaveItems):
      print "Get Item:" + str(itemIndex)
      try:
        if (getItemData(slave, TRANSMIT_ITEM, itemIndex) == PKTF_SENSOR_BAD):
          print "Bad sensor"
        else:
          currentTemp = slave.sensorList[itemIndex].currentValue
          print "Sensor: " + slave.sensorList[itemIndex].sensorID.tostring().encode('hex') + " = " + currentTemp + "C"
      except Exception, e:
        #print "Exception with sensor: " + slave.sensorList[itemIndex].sensorID.tostring().encode('hex')
        print str(e)
        pass
      
      time.sleep(.5)
      itemIndex += 1
    print ""
