import csv
import datetime
from serial import Serial
from pynmeagps import NMEAReader
import time

# File Settings
COMMON_FILE_NAME = "gps_readings"

# Read Current Time for Filename
current_time = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
file_path = f"{COMMON_FILE_NAME}_{current_time}.csv"

# Initialize CSV File
with open(file_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow([
        "GPS_Timestamp", "Latitude", "Longitude"
    ])

# Function to read GPS data
def read_gps(ser):
    try:    
        if ser.in_waiting > 0:  
            data = ser.readline()
            if data.startswith(b'$GNRMC'): 
                gps_data = NMEAReader.parse(data)
                if gps_data.status == "A":        
                    return [
                        gps_data.time, gps_data.lon, gps_data.lat
                    ]
    except Exception as e:
        print(f"GPS Read Error: {e}")



def main():
    count = 1 
    gps_data = [None, None, None] 
    print(f"Logging data to {file_path}...")
    ser = Serial('/dev/ttyAMA0', 115200, timeout=1) 
    print("Serial opened") 
    try:
        with open(file_path, "a", newline="") as csvfile:
            print("Writing to file.") 
            writer = csv.writer(csvfile)
            while True: 
                # Read GPS Data if available
                # print("Reading GPS data")
                new_gps_data = read_gps(ser)
                if new_gps_data: 
                    gps_data = new_gps_data          
                    writer.writerow(gps_data)
                    print(f'data: {gps_data}')

    except KeyboardInterrupt:
        print("\nData collection stopped.")
        ser.close()

if __name__ == "__main__":
    main()
