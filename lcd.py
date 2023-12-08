import Adafruit_CharLCD as LCD
import time

#Define columns and rows of lcd
lcd_columns = 16
lcd_rows    = 2

# Initialize LCD using the pins
lcd = LCD.Adafruit_CharLCDBackpack(address=0x21)


def show_lcd_message(message):
    lcd.clear()
    lcd.set_backlight(0)
    lcd.message(message)


def clear_lcd_message():
    """
    Turns off LCD
    """
    lcd.set_backlight(1)
    lcd.clear()
