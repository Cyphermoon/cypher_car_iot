# native libraries
import time, threading
import sys, signal

# 3rd party libraries
import RPi.GPIO as GPIO
from Adafruit_LED_Backpack import SevenSegment
import Adafruit_CharLCD as LCD
import Adafruit_DHT
import spi, smbus

# Todo: remove motion if RFID card works


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


def is_authenticated():
    while True:
        print("Waiting for motion")
        if detect_motion():
            print("Waiting for motion")
            message = input("What is your name: ")

            if message.lower().strip().replace(" ", "_") == "cypher_moon":
                return True
            else:
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
