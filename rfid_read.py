import RPi.GPIO as GPIO
import MFRC522
import signal

def is_authenticated():
    continue_reading = True

    # Create an object of class MFRC522
    MIFAREReader = MFRC522.MFRC522()

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

                continue_reading = False
                return True
            else:
                continue_reading = False
                return False
