# check_ports.py
import serial.tools.list_ports

def list_ports():
    ports = serial.tools.list_ports.comports()
    print(f"{'Device':<15} {'Description':<30} {'HWID':<40}")
    print("-" * 40)
    
    for port, desc, hwid in sorted(ports):
        print(f"{port:<15} {desc:<30} {hwid:<40}")

if __name__ == "__main__":
    list_ports()