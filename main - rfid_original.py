# native libraries
import time, threading
import sys, signal

# 3rd party libraries
import RPi.GPIO as GPIO
from Adafruit_LED_Backpack import SevenSegment
import Adafruit_CharLCD as LCD
import Adafruit_DHT
import spi, smbus


# Global Variables
# distance - cm
# humidity - %
# temperature - K

INTERVAL = 2
LIGHT_INTERVAL = 2
WEATHER_INTERVAL = 4

MIN_DISTANCE = 10
CAR_SPEED = 30  # cm/m

MIN_LIGHT_INTENSITY = 100

SPEED_OF_SOUND = 17150


# GPIO PINS
TOUCH_PIN = 17
BUZZER_PIN = 18
VIBRATION_PIN = 27
LED_PIN = 26
TILT_PIN = 22
MOTION_PIN = 23

TRIG = 16
ECHO = 12

# car variables
is_car_moving = True
light_intensity = 0
authentication_attempts = 3

start_time = 0
end_time = 0

# total_travel variables
total_travel = {
    "distance": 0,
    "time": 0,
    "turned_left": 0,
    "turned_right": 0,
    "brakes": 0,
    "humidity": {
        "unit": "%",
        "min": 1000,
        "max": 0
    },
    "temperature": {
        "unit": "K",
        "min": 1000,
        "max": 0
    },
    "light": {
        "unit": "lx",
        "min": 1000,
        "max": 0
    }
}

# Define columns and rows of lcd
lcd_columns = 16
lcd_rows = 2

# Initialize LCD and Seven Segment Display
lcd = LCD.Adafruit_CharLCDBackpack(address=0x21)
segment = SevenSegment.SevenSegment(address=0x70)

segment.begin()

# Set up GPIO
GPIO.setmode(GPIO.BCM)

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(TOUCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(VIBRATION_PIN, GPIO.OUT)
GPIO.setup(TILT_PIN, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(MOTION_PIN, GPIO.IN)

# ------------------- MFRC522 -------------------------- #

class MFRC522:
  NRSTPD = 25

  MAX_LEN = 16

  PCD_IDLE       = 0x00
  PCD_AUTHENT    = 0x0E
  PCD_RECEIVE    = 0x08
  PCD_TRANSMIT   = 0x04
  PCD_TRANSCEIVE = 0x0C
  PCD_RESETPHASE = 0x0F
  PCD_CALCCRC    = 0x03

  PICC_REQIDL    = 0x26
  PICC_REQALL    = 0x52
  PICC_ANTICOLL  = 0x93
  PICC_SElECTTAG = 0x93
  PICC_AUTHENT1A = 0x60
  PICC_AUTHENT1B = 0x61
  PICC_READ      = 0x30
  PICC_WRITE     = 0xA0
  PICC_DECREMENT = 0xC0
  PICC_INCREMENT = 0xC1
  PICC_RESTORE   = 0xC2
  PICC_TRANSFER  = 0xB0
  PICC_HALT      = 0x50

  MI_OK       = 0
  MI_NOTAGERR = 1
  MI_ERR      = 2

  Reserved00     = 0x00
  CommandReg     = 0x01
  CommIEnReg     = 0x02
  DivlEnReg      = 0x03
  CommIrqReg     = 0x04
  DivIrqReg      = 0x05
  ErrorReg       = 0x06
  Status1Reg     = 0x07
  Status2Reg     = 0x08
  FIFODataReg    = 0x09
  FIFOLevelReg   = 0x0A
  WaterLevelReg  = 0x0B
  ControlReg     = 0x0C
  BitFramingReg  = 0x0D
  CollReg        = 0x0E
  Reserved01     = 0x0F

  Reserved10     = 0x10
  ModeReg        = 0x11
  TxModeReg      = 0x12
  RxModeReg      = 0x13
  TxControlReg   = 0x14
  TxAutoReg      = 0x15
  TxSelReg       = 0x16
  RxSelReg       = 0x17
  RxThresholdReg = 0x18
  DemodReg       = 0x19
  Reserved11     = 0x1A
  Reserved12     = 0x1B
  MifareReg      = 0x1C
  Reserved13     = 0x1D
  Reserved14     = 0x1E
  SerialSpeedReg = 0x1F

  Reserved20        = 0x20
  CRCResultRegM     = 0x21
  CRCResultRegL     = 0x22
  Reserved21        = 0x23
  ModWidthReg       = 0x24
  Reserved22        = 0x25
  RFCfgReg          = 0x26
  GsNReg            = 0x27
  CWGsPReg          = 0x28
  ModGsPReg         = 0x29
  TModeReg          = 0x2A
  TPrescalerReg     = 0x2B
  TReloadRegH       = 0x2C
  TReloadRegL       = 0x2D
  TCounterValueRegH = 0x2E
  TCounterValueRegL = 0x2F

  Reserved30      = 0x30
  TestSel1Reg     = 0x31
  TestSel2Reg     = 0x32
  TestPinEnReg    = 0x33
  TestPinValueReg = 0x34
  TestBusReg      = 0x35
  AutoTestReg     = 0x36
  VersionReg      = 0x37
  AnalogTestReg   = 0x38
  TestDAC1Reg     = 0x39
  TestDAC2Reg     = 0x3A
  TestADCReg      = 0x3B
  Reserved31      = 0x3C
  Reserved32      = 0x3D
  Reserved33      = 0x3E
  Reserved34      = 0x3F

  serNum = []

  def __init__(self, dev='/dev/spidev0.0', spd=1000000):
    spi.openSPI(device=dev,speed=spd)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.NRSTPD, GPIO.OUT)
    GPIO.output(self.NRSTPD, 1)
    self.MFRC522_Init()

  def MFRC522_Reset(self):
    self.Write_MFRC522(self.CommandReg, self.PCD_RESETPHASE)

  def Write_MFRC522(self, addr, val):
    spi.transfer(((addr<<1)&0x7E,val))

  def Read_MFRC522(self, addr):
    val = spi.transfer((((addr<<1)&0x7E) | 0x80,0))
    return val[1]

  def SetBitMask(self, reg, mask):
    tmp = self.Read_MFRC522(reg)
    self.Write_MFRC522(reg, tmp | mask)

  def ClearBitMask(self, reg, mask):
    tmp = self.Read_MFRC522(reg);
    self.Write_MFRC522(reg, tmp & (~mask))

  def AntennaOn(self):
    temp = self.Read_MFRC522(self.TxControlReg)
    if(~(temp & 0x03)):
      self.SetBitMask(self.TxControlReg, 0x03)

  def AntennaOff(self):
    self.ClearBitMask(self.TxControlReg, 0x03)

  def MFRC522_ToCard(self,command,sendData):
    backData = []
    backLen = 0
    status = self.MI_ERR
    irqEn = 0x00
    waitIRq = 0x00
    lastBits = None
    n = 0
    i = 0

    if command == self.PCD_AUTHENT:
      irqEn = 0x12
      waitIRq = 0x10
    if command == self.PCD_TRANSCEIVE:
      irqEn = 0x77
      waitIRq = 0x30

    self.Write_MFRC522(self.CommIEnReg, irqEn|0x80)
    self.ClearBitMask(self.CommIrqReg, 0x80)
    self.SetBitMask(self.FIFOLevelReg, 0x80)

    self.Write_MFRC522(self.CommandReg, self.PCD_IDLE);

    while(i<len(sendData)):
      self.Write_MFRC522(self.FIFODataReg, sendData[i])
      i = i+1

    self.Write_MFRC522(self.CommandReg, command)

    if command == self.PCD_TRANSCEIVE:
      self.SetBitMask(self.BitFramingReg, 0x80)

    i = 2000
    while True:
      n = self.Read_MFRC522(self.CommIrqReg)
      i = i - 1
      if ~((i!=0) and ~(n&0x01) and ~(n&waitIRq)):
        break

    self.ClearBitMask(self.BitFramingReg, 0x80)

    if i != 0:
      if (self.Read_MFRC522(self.ErrorReg) & 0x1B)==0x00:
        status = self.MI_OK

        if n & irqEn & 0x01:
          status = self.MI_NOTAGERR

        if command == self.PCD_TRANSCEIVE:
          n = self.Read_MFRC522(self.FIFOLevelReg)
          lastBits = self.Read_MFRC522(self.ControlReg) & 0x07
          if lastBits != 0:
            backLen = (n-1)*8 + lastBits
          else:
            backLen = n*8

          if n == 0:
            n = 1
          if n > self.MAX_LEN:
            n = self.MAX_LEN

          i = 0
          while i<n:
            backData.append(self.Read_MFRC522(self.FIFODataReg))
            i = i + 1;
      else:
        status = self.MI_ERR

    return (status,backData,backLen)


  def MFRC522_Request(self, reqMode):
    status = None
    backBits = None
    TagType = []

    self.Write_MFRC522(self.BitFramingReg, 0x07)

    TagType.append(reqMode);
    (status,backData,backBits) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, TagType)

    if ((status != self.MI_OK) | (backBits != 0x10)):
      status = self.MI_ERR

    return (status,backBits)


  def MFRC522_Anticoll(self):
    backData = []
    serNumCheck = 0

    serNum = []

    self.Write_MFRC522(self.BitFramingReg, 0x00)

    serNum.append(self.PICC_ANTICOLL)
    serNum.append(0x20)

    (status,backData,backBits) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE,serNum)

    if(status == self.MI_OK):
      i = 0
      if len(backData)==5:
        while i<4:
          serNumCheck = serNumCheck ^ backData[i]
          i = i + 1
        if serNumCheck != backData[i]:
          status = self.MI_ERR
      else:
        status = self.MI_ERR

    return (status,backData)

  def CalulateCRC(self, pIndata):
    self.ClearBitMask(self.DivIrqReg, 0x04)
    self.SetBitMask(self.FIFOLevelReg, 0x80);
    i = 0
    while i<len(pIndata):
      self.Write_MFRC522(self.FIFODataReg, pIndata[i])
      i = i + 1
    self.Write_MFRC522(self.CommandReg, self.PCD_CALCCRC)
    i = 0xFF
    while True:
      n = self.Read_MFRC522(self.DivIrqReg)
      i = i - 1
      if not ((i != 0) and not (n&0x04)):
        break
    pOutData = []
    pOutData.append(self.Read_MFRC522(self.CRCResultRegL))
    pOutData.append(self.Read_MFRC522(self.CRCResultRegM))
    return pOutData

  def MFRC522_SelectTag(self, serNum):
    backData = []
    buf = []
    buf.append(self.PICC_SElECTTAG)
    buf.append(0x70)
    i = 0
    while i<5:
      buf.append(serNum[i])
      i = i + 1
    pOut = self.CalulateCRC(buf)
    buf.append(pOut[0])
    buf.append(pOut[1])
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buf)

    if (status == self.MI_OK) and (backLen == 0x18):
      print("Size: " + str(backData[0]))
      return    backData[0]
    else:
      return 0

  def MFRC522_Auth(self, authMode, BlockAddr, Sectorkey, serNum):
    buff = []

    # First byte should be the authMode (A or B)
    buff.append(authMode)

    # Second byte is the trailerBlock (usually 7)
    buff.append(BlockAddr)

    # Now we need to append the authKey which usually is 6 bytes of 0xFF
    i = 0
    while(i < len(Sectorkey)):
      buff.append(Sectorkey[i])
      i = i + 1
    i = 0

    # Next we append the first 4 bytes of the UID
    while(i < 4):
      buff.append(serNum[i])
      i = i +1

    # Now we start the authentication itself
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_AUTHENT,buff)

    # Check if an error occurred
    if not(status == self.MI_OK):
      print("AUTH ERROR!!")
    if not (self.Read_MFRC522(self.Status2Reg) & 0x08) != 0:
      print("AUTH ERROR(status2reg & 0x08) != 0")

    # Return the status
    return status

  def MFRC522_StopCrypto1(self):
    self.ClearBitMask(self.Status2Reg, 0x08)

  def MFRC522_Read(self, blockAddr):
    recvData = []
    recvData.append(self.PICC_READ)
    recvData.append(blockAddr)
    pOut = self.CalulateCRC(recvData)
    recvData.append(pOut[0])
    recvData.append(pOut[1])
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, recvData)
    if not(status == self.MI_OK):
      print("Error while reading!")
    i = 0
    if len(backData) == 16:
      print("Sector "+str(blockAddr)+" "+str(backData))
      return backData

  def MFRC522_Write(self, blockAddr, writeData):
    buff = []
    buff.append(self.PICC_WRITE)
    buff.append(blockAddr)
    crc = self.CalulateCRC(buff)
    buff.append(crc[0])
    buff.append(crc[1])
    (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buff)
    if not(status == self.MI_OK) or not(backLen == 4) or not((backData[0] & 0x0F) == 0x0A):
        status = self.MI_ERR

    print("%s backdata &0x0F == 0x0A %s" % (backLen, backData[0]&0x0F))
    if status == self.MI_OK:
        i = 0
        buf = []
        while i < 16:
            buf.append(writeData[i])
            i = i + 1
        crc = self.CalulateCRC(buf)
        buf.append(crc[0])
        buf.append(crc[1])
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE,buf)
        if not(status == self.MI_OK) or not(backLen == 4) or not((backData[0] & 0x0F) == 0x0A):
            print("Error while writing")
        if status == self.MI_OK:
            print("Data written")

  def MFRC522_DumpClassic1K(self, key, uid):
    i = 0
    while i < 64:
        status = self.MFRC522_Auth(self.PICC_AUTHENT1A, i, key, uid)
        # Check if authenticated
        if status == self.MI_OK:
            self.MFRC522_Read(i)
        else:
            print("Authentication error")
        i = i+1

  def MFRC522_Init(self):
    GPIO.output(self.NRSTPD, 1)

    self.MFRC522_Reset();


    self.Write_MFRC522(self.TModeReg, 0x8D)
    self.Write_MFRC522(self.TPrescalerReg, 0x3E)
    self.Write_MFRC522(self.TReloadRegL, 30)
    self.Write_MFRC522(self.TReloadRegH, 0)

    self.Write_MFRC522(self.TxAutoReg, 0x40)
    self.Write_MFRC522(self.ModeReg, 0x3D)
    self.AntennaOn()


# ------------------------RFID Read------------------------------ #
def is_authenticated():
    continue_reading = True

    # Create an object of class MFRC522
    MIFAREReader = MFRC522()

    while continue_reading:
        # Detect touch of the card, get status and tag type
        print("Waiting for card...")
        (status, TagType) = MIFAREReader.MFRC522_Request(MIFAREReader.PICC_REQIDL)

        # Check if card detected
        if status == MIFAREReader.MI_OK:
            print("Card detected")

        # Get the RFID card UID and status
        (status, uid) = MIFAREReader.MFRC522_Anticoll()

        # If status is alright, continue to the next stage
        if status == MIFAREReader.MI_OK:
            # Print UID
            print("Card read UID: %s,%s,%s,%s" % (uid[0], uid[1], uid[2], uid[3]))
            # Standard key for RFID tags
            key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
            # Select the scanned tag
            MIFAREReader.MFRC522_SelectTag(uid)
            # Authenticate
            status = MIFAREReader.MFRC522_Auth(MIFAREReader.PICC_AUTHENT1A, 8, key, uid)
            # Check if authenticated successfully, read the data
            if status == MIFAREReader.MI_OK:
                data = MIFAREReader.MFRC522_Read(8)
                MIFAREReader.MFRC522_StopCrypto1()
                
                print(f"Data read: {data}")
                if data[0] == 27 and data[1] == 30:
                    continue_reading = False
                    return True
                return False
            else:
                continue_reading = False
                return False

# -------------------------Seven Segment----------------------------- #
def show_countdown(number):
    if number > 99:
        return

    segment.clear()

    segment.set_digit(0, int(number / 10) % 10)
    segment.set_digit(1, number % 10)

    segment.write_display()


def clear_segment():
    segment.clear()
    segment.write_display()


# -------------------------Ultra Sonic-------------------------------------- #
def get_ultrasonic_distance():
    """
    Sends a sonic wave and measures the time the echo takes to return
    """
    # Prepare Trigger
    GPIO.output(TRIG, False)
    print("Preparing sonic wave...")
    time.sleep(2.0)

    # Send Sonic Wave
    GPIO.output(TRIG, True)
    print("Sending sonic wave...")
    time.sleep(0.00001)

    # Stop Sonic Wave
    GPIO.output(TRIG, False)
    
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start

    distance = round(pulse_duration * SPEED_OF_SOUND, 2) # distance in cm

    return distance

# -------------------------GPIO MISC-------------------------------------- #
def touch_sensor_clicked():
    return GPIO.input(TOUCH_PIN)


def stop_buzz():
    GPIO.output(BUZZER_PIN, GPIO.LOW)


def buzz_alarm(n=1):
    for _ in range(n):
        GPIO.output(BUZZER_PIN, GPIO.HIGH)
        time.sleep(2)
        stop_buzz()


def start_vibration():
    GPIO.output(VIBRATION_PIN, GPIO.HIGH)


def stop_vibration():
    GPIO.output(VIBRATION_PIN, GPIO.LOW)


def turn_on_light():
    GPIO.output(LED_PIN, GPIO.HIGH)


def turn_off_light():
    GPIO.output(LED_PIN, GPIO.LOW)


def detect_motion():
    if (GPIO.input(MOTION_PIN) == 1):
        return True
    else:
        return False


def get_turning_direction():
    """
    Checks if the tilt sensor is turning left or right
    """
    print("checking tilt")
    if (GPIO.input(TILT_PIN)):
        # tilting left
        return "left"
    else:
        #tilting right
        return "right"
    
    return None

# -------------------------DH11-------------------------------------- #
def convert_to_kelvin(temp):
    return temp + 273.15


def get_humidity_temperature():
     #Try to grab a sensor reading.  Use the read_retry method which will retry up
    # to 15 times to get a sensor reading (waiting 2 seconds between each retry).
    humidity, temperature = Adafruit_DHT.read_retry(11, 4)

    if humidity is not None and temperature is not None:
        return humidity, convert_to_kelvin(temperature)
    else:
        pass


# -------------------------LCD-------------------------------------- #

def show_lcd_message(message, blink = False):
    lcd.clear()
    lcd.blink(True) if blink is True else lcd.blink(False)
    lcd.set_backlight(0)
    lcd.message(message)


def clear_lcd_message():
    """
    Turns off LCD
    """
    lcd.set_backlight(1)
    lcd.clear()


# -------------------------Light-------------------------------------- #
# Find the right revision for bus driver
if(GPIO.RPI_REVISION == 1):
    bus = smbus.SMBus(0)
else:
    bus = smbus.SMBus(1)


class LightSensor():

    def __init__(self):

        # Define some constants from the datasheet

        self.DEVICE = 0x5c # Default device I2C address

        self.POWER_DOWN = 0x00 # No active state
        self.POWER_ON = 0x01 # Power on
        self.RESET = 0x07 # Reset data register value

        # Start measurement at 4lx resolution. Time typically 16ms.
        self.CONTINUOUS_LOW_RES_MODE = 0x13
        # Start measurement at 1lx resolution. Time typically 120ms
        self.CONTINUOUS_HIGH_RES_MODE_1 = 0x10
        # Start measurement at 0.5lx resolution. Time typically 120ms
        self.CONTINUOUS_HIGH_RES_MODE_2 = 0x11
        # Start measurement at 1lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        self.ONE_TIME_HIGH_RES_MODE_1 = 0x20
        # Start measurement at 0.5lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        self.ONE_TIME_HIGH_RES_MODE_2 = 0x21
        # Start measurement at 1lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        self.ONE_TIME_LOW_RES_MODE = 0x23

    def convertToNumber(self, data):

        # Simple function to convert 2 bytes of data
        # into a decimal number
        return ((data[1] + (256 * data[0])) / 1.2)

    def readLight(self):

        data = bus.read_i2c_block_data(self.DEVICE,self.ONE_TIME_HIGH_RES_MODE_1)
        return self.convertToNumber(data)


# -------------------------Car-------------------------------------- #

def get_total_time():
    global start_time, end_time
    return round(end_time - start_time)


def is_plural(number):
    return number > 1


# ------------- CAR OPERATIONS ------------- #
def start_car():
    """
        Start the car by initializing the start time and performing necessary actions.
    """

    global start_time
    start_time = time.perf_counter()

    clear_segment()

    show_lcd_message("Welcome! Moon!")
    start_vibration()


def restart_car(duration=3):
    """
        Stops the car movement, wait a few seconds and starts the car again
    """

    show_lcd_message("Restarting car\n..........")
    stop_vibration()
    time.sleep(duration)
    show_lcd_message("Car Moving !!", True)
    start_vibration()


def clean_resource(clean_gpio=False):
    """
        Clean up resources used by script
    """
    clear_segment()
    clear_lcd_message()
    turn_off_light()
    stop_vibration()

    if clean_gpio:
        pass
        GPIO.cleanup()


def shutdown_car(signal, frame):
    """
        Stops the car movement and clean up the resources
    """
    global is_car_moving, end_time

    end_time = time.perf_counter()
    is_car_moving = False

    clean_resource(True)
    print(show_travel_stats())


def handle_authentication_failed():
    """
        Keeps track of the amount of attempts left 
        and shuts the car down if attempt is exhausted   
    """

    global authentication_attempts

    # reduce attempts
    authentication_attempts -= 1

    # Don't start the car if attempt is exhausted
    if authentication_attempts == 0:
        show_lcd_message("Shutting down...")
        buzz_alarm(3)
        clean_resource()
        return True
    
    # Let the user know the current trial failed and there is a number of attempt left
    show_lcd_message(
        f"Attempt failed!!\n{'Countdown started' if authentication_attempts == 2 else ''}"
    )

    show_countdown(authentication_attempts)
    buzz_alarm()


def show_travel_stats():
    """
        Generates a statistics report for the travel data.
    """

    global total_travel

    info = "\n\n========================= STATISTICS =========================\n"

    # calculate time
    total_travel["time"] = get_total_time()
    minute = total_travel["time"] / 60
    seconds = total_travel["time"]

    info += f"\nTotal uptime was {int(minute)}m {seconds}s\n"

    # calculate distance
    total_travel["distance"] = CAR_SPEED * minute
    info += f"\nTotal distance traveled was {total_travel['distance']}cm\n"

    #get total number of turns
    left_turns = total_travel["turned_left"]
    info += f"\n You turned to left {left_turns} time{'s' if is_plural(left_turns) else ''}\n"

    right_turns = total_travel["turned_right"]
    info += f"\n You turned to right {right_turns} time{'s' if is_plural(right_turns) else ''}\n"

    # get total number of brakes
    info += f"\n You applied brakes {total_travel['brakes']} time{'s' if is_plural(total_travel['brakes']) else ''}\n"

    for key, value in total_travel.items():
        if type(value) is dict:
            info += f"\n{key.capitalize()} was within {value['min']}{value['unit']} and {value['max']}{value['unit']}\n"

    
    info += "\n==============================================================\n"

    return info


# ------------- MONITORING FUNCTIONS ------------- #

def handle_car_turning():
    """
        Monitor the car for when it turns and perform actions when this happens
    """

    global is_car_moving


    while is_car_moving:
        # get car turning direction
        car_turning_direction = get_turning_direction()

        # handle car turning left
        if car_turning_direction == "left":
            total_travel["turned_left"] += 1
            print("Car turning left")

        # handle car turning right
        elif car_turning_direction == "right":
            total_travel["turned_right"] += 1
            print("Car turning right")
        
        time.sleep(15)
    


def handle_brakes():
    """
        Stops the car when an object with a distance less than 10cm is detected
    """

    global is_car_moving, MIN_DISTANCE

    while is_car_moving:
        distance = get_ultrasonic_distance()

        if distance < MIN_DISTANCE:
            total_travel["brakes"] += 1

            print("Stopping car because distance is less than 10cm")
            show_lcd_message("Applying brakes")

            # this would stop the vibration of the car
            restart_car()    

        time.sleep(INTERVAL)
    


def handle_front_lightning():
    """
        Turns on led when the light is below a certain level
    """
    global is_car_moving, LIGHT_INTERVAL, MIN_LIGHT_INTENSITY, light_intensity


    light_sensor = LightSensor()

    while is_car_moving:
        light_intensity = light_sensor.readLight()

        total_travel["light"]["min"] = min(light_intensity, total_travel["light"]["min"])
        total_travel["light"]["max"] = max(light_intensity, total_travel["light"]["max"])

        if light_intensity < MIN_LIGHT_INTENSITY:
            print("Turning on light")
            turn_off_light()
        else:
            print("Turning off light")
            turn_on_light()

        #Checks light at a certain interval
        time.sleep(LIGHT_INTERVAL)
    


def handle_weather_report():
    """
        Gets the humidity and temperature and displays it on the LCD
    """

    global is_car_moving

    while is_car_moving:
        if touch_sensor_clicked():
            humidity, temperature = get_humidity_temperature()

            total_travel["temperature"]["min"] = min(temperature, total_travel["temperature"]["min"])
            total_travel["temperature"]["max"] = max(temperature, total_travel["temperature"]["max"])

            total_travel["humidity"]["min"] = min(humidity, total_travel["humidity"]["min"])
            total_travel["humidity"]["max"] = max(humidity, total_travel["humidity"]["max"])

            show_lcd_message(f"T{temperature}K,H{humidity}%\nL{round(light_intensity, 2)}lx")

        #Checks weather at a certain interval
        time.sleep(WEATHER_INTERVAL)

# --------------------------------------------------------- #

if __name__ == "__main__":

    #Gracefully exit program when Ctrl+c is pressed
    signal.signal(signal.SIGINT, shutdown_car)
    
    #shutdown_car(1, 2)
    #sys.exit(0)
    # Authenticate user with on RFID card
    while True:
        auth = is_authenticated()
        print("authenticated" + str(auth))
        if auth:
            start_car()
            break
        else:
            if handle_authentication_failed():
                sys.exit()
                break


    # Create a thread pool of monitoring functions
    monitoring_functions = [
        handle_front_lightning,
        handle_car_turning,
        handle_weather_report,
        handle_brakes,
    ]

    threads = []

    # Creating a thread for each monitoring function
    for handler in monitoring_functions:
        thread = threading.Thread(target=handler, name=handler.__name__)

        print(f"--Thread created: {thread.name}\n")

        thread.start()
        threads.append(thread)


    # Wait for all threads to finish before finishing script
    for thread in threads:
        print(f"---Thread joined: {thread.name}\n")
        thread.join()


