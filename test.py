import time
import serial

# Configure the serial connection
instrument = serial.Serial(
    port="COM4",  # Change to your specific COM port (e.g., '/dev/ttyUSB0' on Linux)
    baudrate=9600,  # Must match the front panel setting
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1.0,  # Read timeout in seconds
)

try:
    # 1. Identify the device
    # Every command must end with \n
    instrument.write(b"*IDN?\n")
    # Read the response ending with \n
    response = instrument.readline().decode("utf-8").strip()
    print(f"Connected to: {response}")

    # 2. Query Temperature from Channel A
    # The Cryo-con 22C expects standard SCPI format
    instrument.write(b"INP? A\n")
    temp_a = instrument.readline().decode("utf-8").strip()
    print(f"Channel A Temperature: {temp_a} K")

except Exception as e:
    print(f"Communication error: {e}")

finally:
    # Always close the port when done
    instrument.close()
