import serial
import struct
import argparse

# ANSI escape codes for coloring
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RESET = "\033[0m"

def update_crc8(data, crc):
    crc ^= data
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0x07
        else:
            crc <<= 1
        crc &= 0xFF  # Ensure crc is 8-bit
    return crc

def get_crc8(buf):
    crc = 0
    for byte in buf:
        crc = update_crc8(byte, crc)
    return crc

def main(port, baud):
    esc_telem = serial.Serial(port, baud)
    esc_telem.flushInput()
    
    while True:
        buf = esc_telem.read(10)
        data = struct.unpack('10B', buf)
        
        # Calculate CRC for the received data (excluding the last byte which is the received CRC)
        calculated_crc = get_crc8(data[:-1])
        received_crc = data[-1]
        
        if calculated_crc == received_crc:
            temp = data[0]
            voltage = (data[1] << 8) | data[2]
            current = (data[3] << 8) | data[4]
            consumption = (data[5] << 8) | data[6]
            e_rpm = (data[7] << 8) | data[8]
            
            print(f"\rTemperature: {GREEN}{temp:3}{RESET} °C  Voltage: {GREEN}{voltage:5}{RESET}mV  Current: {GREEN}{current:5}{RESET}mA  Consumption: {GREEN}{consumption:5}{RESET}mAh  eRPM: {GREEN}{e_rpm:5}{RESET}", end='')
        else:
            print("\rCRC mismatch! Data may be corrupted.", end='')
            esc_telem.flushInput()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Telemetry Data Monitor")
    parser.add_argument("-p", "--port", required=True, help="Serial port to read telemetry data from")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate for the serial connection (default: 115200)")
    args = parser.parse_args()
    main(args.port, args.baud)