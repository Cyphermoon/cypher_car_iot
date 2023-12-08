import RPi.GPIO as GPIO
import MFRC522

def write_to_rfid(data_to_write):
    continue_writing = True

    # Create an object of class MFRC522
    MIFAREReader = MFRC522.MFRC522()

    print("Welcome to MFRC522 RFID Write function")
    print("Press CTRL+C anytime to quit.")

    while continue_writing:
        # Detect touch of the card, get status and tag type
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
            # Standard encryption key for RFID cards (default)
            key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
            # Select the scanned tag
            MIFAREReader.MFRC522_SelectTag(uid)
            # Authenticate
            status = MIFAREReader.MFRC522_Auth(MIFAREReader.PICC_AUTHENT1A, 8, key, uid)
            # Check if authenticated successfully, write data
            if status == MIFAREReader.MI_OK:
                print("Writing data to RFID card:")
                MIFAREReader.MFRC522_Write(8, data_to_write)
                MIFAREReader.MFRC522_StopCrypto1()
                return True
            else:
                print("Authentication error")
                return False

# Uncomment the line below to test the function
# data_to_write = [99, 11, 55, 66, 44, 111, 222, 210, 125, 153, 136, 199, 144, 177, 166, 188]
# write_result = write_to_rfid(data_to_write)
# print("Write result:", write_result)
