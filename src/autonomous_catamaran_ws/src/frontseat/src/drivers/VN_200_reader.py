import serial

def read_serial_data():
    try:
        # Open the serial port
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        print(f"Connected to {ser.name}")

        while True:
            # Read a line of data from the serial port
            data = ser.readline()
            if data:
                print(f"Received: {data}")

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Program interrupted. Exiting...")
    finally:
        # Close the serial port
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    read_serial_data()
