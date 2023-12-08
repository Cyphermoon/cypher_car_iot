"""
Utility functions that provide abstractions for each sensors
"""
import Rpi.GPIO as GPIO
import time

# Pins
TOUCH_PIN = 17
BUZZER_PIN = 18
VIBRATION_PIN = 27
LED_PIN = 26
TILT_PIN = 22
MOTION_PIN = 23


GPIO.setmode(GPIO.BCM)

GPIO.setup(TOUCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(VIBRATION_PIN, GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT)


def touch_sensor_clicked():
    return GPIO.input(TOUCH_PIN)

def stop_buzz():
    GPIO.output(BUZZER_PIN, GPIO.LOW)


def buzz_alarm(n=1):
    for _ in range(n):
        GPIO.output(BUZZER_PIN, GPIO.HIGH)
        time.sleep(0.5)
        stop_buzz()

def start_vibration():
    GPIO.output(VIBRATION_PIN, GPIO.HIGH)


def stop_vibration():
    GPIO.output(VIBRATION_PIN, GPIO.LOW)


def turn_on_light():
    GPIO.output(LED_PIN, GPIO.HIGH)


def turn_off_light():
    GPIO.output(LED_PIN, GPIO.LOW)


def get_turning_direction():
    """
    Checks if the tilt sensor is turning left or right
    """

    if (GPIO.input(TILT_PIN)):
        # tilting left
        return "left"
    else:
        #tilting right
        return "right"
    
    return None


def detect_motion():
    if (GPIO.input(MOTION_PIN) == 1):
        return True
    else:
        return False