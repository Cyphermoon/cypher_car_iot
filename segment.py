from Adafruit_LED_Backpack import SevenSegment

segment = SevenSegment.SevenSegment(address=0x70)

#initialize the segment once
segment.begin()

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