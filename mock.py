import random
import time

#------------ Mock Functions ------------#
def clear_segment():
    print("Clearing Segment")


def show_countdown(n):
    print(f"Countdown: {n}")


def clear_lcd_message():
    print("Clearing LCD")


def start_vibration():
    print("Starting vibration\n")

def stop_vibration():
    print("Stopping vibration\n")


def show_lcd_message(message):
    print(f"\n--------------\nLCD Message\n{message} \n------------------\n")

def turn_on_light():
    print("Turning on light\n")

def turn_off_light():
    print("Turning off light\n")


def buzz_alarm(n=1):
    for i in range(n):
        print(f"Starting buzz --- {i} times")
        time.sleep(0.5)
        print(f"Stopping buzz --- {i} times")


def is_authenticated():
    while True:
        if detect_motion():
            print("Waiting for motion")
            message = input("What is your name: ")

            if message:
                return True
            else:
                return False


def detect_motion():
    return random.choice([True, False])


def get_turning_direction():
    direction = random.choice([None, "left", "right"])
    print(f"Car turning direction: {direction}")
    return direction

def get_humidity_temperature():
    humidity = random.randint(0, 100)
    temperature = random.randint(-50, 50)
    print(f"Humidity: {humidity} -------------------------------:= Temperature: {temperature}")
    return humidity, temperature

def get_ultrasonic_distance():
    distance = random.uniform(0, 100)
    print(f"Ultrasonic distance reading --------------- {distance}\n")
    return distance

def touch_sensor_clicked():
    clicked = random.choice([True, False])
    print(f"Touch sensor clicked: ---------------{clicked}\n")
    return clicked

class LightSensor:
    def readLight(self):
        choice = random.randint(0, 100)
        print(f"Light sensor reading: --------------- {choice}")
        return choice
#----------------------------------------#