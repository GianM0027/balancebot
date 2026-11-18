"""Save the CSV printed by the ESP32 on the serial port to a file.

Run on WINDOWS (not WSL: WSL does not see the USB serial ports without extra setup),
with the Arduino IDE serial monitor CLOSED (only one program can use the port):

    pip install pyserial
    python tools\\serial_log.py COM5 60 data\\imu_still.csv

Find the port name (COM3, COM5, ...) in the Arduino IDE under Tools > Port.
"""
import sys
import time

import serial


def log(port: str, seconds: float, path: str, baud: int = 921600) -> None:
    """Read lines from the serial port and write them to a file.

    Args:
        port:    serial port, e.g. "COM5".
        seconds: duration of the recording [s].
        path:    output CSV file.
        baud:    baud rate, must match Serial.begin() in the sketch.
    """
    with serial.Serial(port, baud, timeout=1) as ser, open(path, "w") as out:
        ser.reset_input_buffer()
        out.write("t_us,ax,ay,az,gx,gy,gz\n")
        n, t_end = 0, time.time() + seconds
        while time.time() < t_end:
            line = ser.readline().decode(errors="ignore").strip()
            fields = line.split(",")
            if len(fields) != 7 or not fields[0].isdigit():   # header, messages, partial lines
                continue
            out.write(line + "\n")
            n += 1
        print(f"saved {n} samples to {path}")


if __name__ == "__main__":
    log(sys.argv[1], float(sys.argv[2]), sys.argv[3])
