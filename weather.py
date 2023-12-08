import Adafruit_DHT

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