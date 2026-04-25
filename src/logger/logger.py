import serial
import threading
import sys
import os
import time

# --- CHANGE THIS TO YOUR ARDUINO'S PORT ---
COM_PORT = 'COM3'  # Example: 'COM3' for Windows, '/dev/tty.usbmodem...' for Mac
BAUD_RATE = 115200
FILE_NAME = 'logs/tvc_flight_data.csv'

try:
    # Connect to the Arduino
    ser = serial.Serial(COM_PORT, BAUD_RATE)
    print(f"Connected to {COM_PORT}.")
    print(f"Recording data to {FILE_NAME}... Press Ctrl+C to stop.")
    print("Type 'arm' to start/resume the EDF, and 'off' to turn it off.")

    # Ensure the directory exists
    os.makedirs(os.path.dirname(FILE_NAME), exist_ok=True)

    def read_keyboard():
        while True:
            try:
                cmd = sys.stdin.readline()
                if cmd:
                    ser.write(cmd.encode('utf-8'))
            except:
                break

    # Start a background thread to listen for keyboard input
    input_thread = threading.Thread(target=read_keyboard, daemon=True)
    input_thread.start()

    # Open the CSV file and start writing
    with open(FILE_NAME, 'w') as f:
        while True:
            if ser.in_waiting > 0:
                # Read the line, decode the bytes to text, and strip extra spaces
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Print to screen so you can watch it
                print(line) 
                
                # Write to the CSV file if it contains comma (CSV data or header)
                if ',' in line:
                    f.write(line + '\n')

except KeyboardInterrupt:
    # This runs when you press Ctrl+C to end the test
    print(f"\nTest Complete. Data safely saved to {FILE_NAME}")
    try:
        print("Sending 'off' command to Arduino to stop the EDF...")
        ser.write(b'off\n')
        time.sleep(0.5) # Give it a moment to send before closing the port
    except Exception as e:
        print(f"Failed to send 'off' command: {e}")
    ser.close()
except Exception as e:
    print(f"Error: {e}")