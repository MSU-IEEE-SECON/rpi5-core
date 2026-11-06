#!/usr/bin/env python3
"""
lidar_identify.py - figure out which Slamtec RPLIDAR you have (no sticker needed).

How it works:
  Every RPLIDAR model talks at a fixed baud rate. This script tries each one,
  sends GET_INFO (0xA5 0x50) and GET_HEALTH (0xA5 0x52), and reports which baud
  answered plus the model byte, firmware, hardware rev and serial number.

  Baud rate that answers  ->  likely model
    115200               ->  A1 (A1M8) or A2M8
    256000               ->  A2M12 / A3 / S1
    460800               ->  C1
    1000000              ->  S2 / S3

Usage:
  pip install pyserial
  Windows:  python lidar_identify.py COM5        (check Device Manager > Ports)
  Pi/Linux: python3 lidar_identify.py /dev/ttyUSB0
  No port given -> it lists ports and tries each USB-serial one.

Note: this only reads info, it does not start the motor or a scan.
"""
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial not installed. Run:  pip install pyserial")

BAUDS = [115200, 256000, 460800, 1000000]
BAUD_HINT = {
    115200: "A1 (A1M8) or A2M8",
    256000: "A2M12, A3, or S1",
    460800: "C1",
    1000000: "S2 or S3",
}

CMD_STOP = b"\xA5\x25"
CMD_RESET = b"\xA5\x40"
CMD_GET_INFO = b"\xA5\x50"
CMD_GET_HEALTH = b"\xA5\x52"
HEALTH_STATUS = {0: "Good", 1: "Warning", 2: "Error"}


def read_response(ser, expected_len, timeout=1.0):
    """Read a 7-byte descriptor (A5 5A ...) and then expected_len payload bytes."""
    deadline = time.time() + timeout
    buf = b""
    # sync to A5 5A
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if len(buf) >= 2 and buf[-2:] == b"\xA5\x5A":
            break
    else:
        return None
    rest = ser.read(5)
    if len(rest) < 5:
        return None
    payload = ser.read(expected_len)
    if len(payload) < expected_len:
        return None
    return payload


def probe(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.3)
    except serial.SerialException as e:
        print(f"  can't open {port}: {e}")
        return None
    try:
        # Many USB adapters wire DTR to the motor enable; keep motor off.
        ser.dtr = True
        ser.write(CMD_STOP)
        time.sleep(0.05)
        ser.reset_input_buffer()

        ser.write(CMD_GET_INFO)
        info = read_response(ser, 20)
        if info is None:
            return None

        model = info[0]
        fw_minor, fw_major, hw = info[1], info[2], info[3]
        serial_no = info[4:20].hex().upper()

        ser.reset_input_buffer()
        ser.write(CMD_GET_HEALTH)
        health = read_response(ser, 3)
        health_str = "unknown"
        if health:
            status = health[0]
            err = health[1] | (health[2] << 8)
            health_str = HEALTH_STATUS.get(status, f"?({status})")
            if err:
                health_str += f" (error code 0x{err:04X})"

        return {
            "baud": baud,
            "model_byte": model,
            "major_id": model >> 4,
            "sub_id": model & 0x0F,
            "firmware": f"{fw_major}.{fw_minor:02d}",
            "hardware": hw,
            "serial": serial_no,
            "health": health_str,
        }
    finally:
        ser.close()


def candidate_ports():
    ports = list(list_ports.comports())
    print("Serial ports found:")
    for p in ports:
        print(f"  {p.device:15s} {p.description}  [VID:PID {p.vid and hex(p.vid)}:{p.pid and hex(p.pid)}]")
    # RPLIDAR adapters are almost always CP210x (10C4:EA60) or CH340 (1A86:7523)
    likely = [p.device for p in ports if (p.vid, p.pid) in {(0x10C4, 0xEA60), (0x1A86, 0x7523)}]
    return likely or [p.device for p in ports]


def main():
    ports = [sys.argv[1]] if len(sys.argv) > 1 else candidate_ports()
    if not ports:
        sys.exit("No serial ports found. Is the LiDAR plugged in / driver installed?")

    for port in ports:
        print(f"\nProbing {port} ...")
        for baud in BAUDS:
            print(f"  trying {baud} baud ... ", end="", flush=True)
            r = probe(port, baud)
            if r is None:
                print("no answer")
                continue
            print("ANSWERED")
            print("\n=== RPLIDAR found ===")
            print(f"  Port          : {port}")
            print(f"  Baud          : {r['baud']}  ->  likely {BAUD_HINT[r['baud']]}")
            print(f"  Model byte    : 0x{r['model_byte']:02X} (major {r['major_id']}, sub {r['sub_id']})")
            print(f"  Firmware      : {r['firmware']}")
            print(f"  Hardware rev  : {r['hardware']}")
            print(f"  Serial number : {r['serial']}")
            print(f"  Health        : {r['health']}")
            return
    print("\nNothing answered. Check: USB cable is data-capable, driver installed "
          "(CP210x on Windows), and nothing else (e.g. RoboStudio) has the port open.")


if __name__ == "__main__":
    main()
