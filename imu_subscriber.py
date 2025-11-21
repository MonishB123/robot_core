import serial
import time

# Change this if needed:
# On Jetson/Ubuntu, Arduino is usually /dev/ttyACM0 or /dev/ttyUSB0
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

def main():
    print(f"Connecting to Arduino on {SERIAL_PORT} at {BAUD_RATE} baud...")

    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=1  # seconds
    )

    # Give Arduino a moment to reset after opening the port
    time.sleep(2)

    print("Connected. Reading pitch,roll,yaw from serial...\n")

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Expecting: "pitch,roll,yaw"
            parts = line.split(",")
            if len(parts) != 3:
                print(f"Raw: {line}")  # something unexpected; just show it
                continue

            try:
                pitch = float(parts[0])
                roll  = float(parts[1])
                yaw   = float(parts[2])

                # Print nicely formatted
                print(f"Pitch: {pitch:7.2f}  Roll: {roll:7.2f}  Yaw: {yaw:7.2f}")
            except ValueError:
                # Couldn’t parse the numbers; print raw line for debugging
                print(f"Parse error, raw line: {line}")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
