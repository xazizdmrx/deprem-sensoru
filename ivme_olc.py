import serial
import csv
import time
from datetime import datetime

PORT = 'COM3'
BAUDRATE = 9600
RECONNECT_DELAY = 5  # saniye

filename = "tilt_data.csv"

def connect_serial():
    while True:
        try:
            ser = serial.Serial(PORT, BAUDRATE, timeout=1)
            time.sleep(2)  # Arduino'nun açılması için bekleme
            print(f"[{datetime.now()}] {PORT} bağlantısı başarılı.")
            return ser
        except serial.SerialException:
            print(f"[{datetime.now()}] {PORT} bağlanılamadı. {RECONNECT_DELAY}s sonra tekrar denenecek...")
            time.sleep(RECONNECT_DELAY)

def main():
    ser = connect_serial()

    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp_epoch", "timestamp_readable", "roll", "pitch"])

        print("Veri kaydı başladı. Ctrl+C ile durdurabilirsin.")

        try:
            while True:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if "roll" in line and "pitch" in line:
                        parts = line.replace("roll = ", "").replace("pitch = ", "").split(",")
                        if len(parts) == 2:
                            roll = float(parts[0].strip())
                            pitch = float(parts[1].strip())
                            ts_epoch = time.time()
                            ts_readable = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                            writer.writerow([ts_epoch, ts_readable, roll, pitch])
                            print(f"[{ts_readable}] roll: {roll:.2f}, pitch: {pitch:.2f}")
                except (ValueError, UnicodeDecodeError):
                    continue  # Hatalı satır varsa atla
                except serial.SerialException:
                    print(f"[{datetime.now()}] COM port bağlantısı koptu. Tekrar bağlanılıyor...")
                    ser.close()
                    ser = connect_serial()
        except KeyboardInterrupt:
            print("\nVeri kaydı durduruldu.")
            ser.close()

if __name__ == "__main__":
    main()
