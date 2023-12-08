import RPi.GPIO as GPIO
import time

# sonic parameters
TRIG = 16
ECHO = 12
speed_of_sound = 17150

GPIO.setmode(GPIO.BCM)

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)


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

    distance = round(pulse_duration * speed_of_sound, 2) # distance in cm

    return distance



if __name__ == "__main__":
        print(f"Distance is {get_ultrasonic_distance()}cm")


