#!/usr/bin/env python3
import sys
import serial
import serial.tools.list_ports
import struct
import time
import curses
import argparse
import threading
import socket
import os

# Betaflight Mode IDs
BOX_NAMES = {
    0: "ARM",
    1: "ANGLE",
    2: "HORIZON",
    3: "MAG",
    4: "HEADFREE",
    5: "BARO",
    8: "FAILSAFE",
    9: "GPS HOME",
    10: "GPS HOLD",
    12: "BEEPER",
    24: "BLACKBOX",
    26: "AIRMODE",
    39: "TURTLE",
    42: "GPS RESCUE",
    50: "PREARM",
}

def is_jetson():
    hostname = socket.gethostname()
    return hostname in ['zephyrus', 'boreas', 'jetson', 'orin']

# Default RC state: Roll (0), Pitch (1), Throttle (2), Yaw (3), AUX1-4 (4-7)
rc_channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
aux_labels = {0: "ARM", 1: "AUX 2", 2: "AUX 3", 3: "AUX 4"}
is_running = True
serial_port = None

def build_msp_set_raw_rc(channels):
    # MSP_SET_RAW_RC (200)
    # Order: ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4
    payload = struct.pack('<8H', *channels)
    size = len(payload)
    cmd = 200
    cs = size ^ cmd
    for b in payload:
        cs ^= b
    return b'$M<' + struct.pack('<BB', size, cmd) + payload + struct.pack('<B', cs)

def msp_sender_thread():
    global is_running, serial_port, rc_channels
    while is_running:
        if serial_port and serial_port.is_open:
            try:
                msg = build_msp_set_raw_rc(rc_channels)
                serial_port.write(msg)
            except Exception:
                pass
        time.sleep(0.02)  # 50Hz

def parse_config(file_path):
    mapping = {}
    if not file_path or not os.path.exists(file_path):
        return mapping
    try:
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('aux'):
                    parts = line.split()
                    if len(parts) >= 3:
                        aux_idx = int(parts[1])
                        mode_id = int(parts[2])
                        name = BOX_NAMES.get(mode_id, f"MODE_{mode_id}")
                        mapping[aux_idx] = name
    except Exception:
        pass
    return mapping

def select_port(stdscr):
    ports = serial.tools.list_ports.comports()
    default_dev = '/dev/ttyTCU0' if is_jetson() else '/dev/ttyACM0'
    devices = [p.device for p in ports]
    if default_dev not in devices:
        devices.insert(0, default_dev)

    current_selection = 0
    while True:
        stdscr.clear()
        stdscr.addstr(0, 0, "Select Serial Port (Use Up/Down arrows, Enter to select):", curses.A_BOLD)
        for idx, dev in enumerate(devices):
            desc = ""
            for p in ports:
                if p.device == dev:
                    desc = f" - {p.description}"
                    break
            label = f"  {dev}{desc}"
            if dev == default_dev: label += " (Default)"
            if idx == current_selection:
                stdscr.addstr(idx + 2, 2, f"> {label.strip()}", curses.A_REVERSE)
            else:
                stdscr.addstr(idx + 2, 2, f"  {label.strip()}")
        stdscr.refresh()
        key = stdscr.getch()
        if key == curses.KEY_UP and current_selection > 0: current_selection -= 1
        elif key == curses.KEY_DOWN and current_selection < len(devices) - 1: current_selection += 1
        elif key == ord('\n'): return devices[current_selection]

def draw_stick(stdscr, y, x, val_h, val_v, label):
    # val_h and val_v are 1000-2000
    size = 7
    stdscr.addstr(y-1, x + (size//2) - (len(label)//2), label, curses.A_BOLD)
    for i in range(size):
        stdscr.addstr(y + i, x, "|" + " " * (size*2-1) + "|")
    stdscr.addstr(y, x, "+" + "-" * (size*2-1) + "+")
    stdscr.addstr(y + size - 1, x, "+" + "-" * (size*2-1) + "+")
    
    # Calculate position
    pos_x = int((val_h - 1000) / 1000 * (size*2-2)) + 1
    pos_y = int((2000 - val_v) / 1000 * (size-2)) + 1
    stdscr.addstr(y + pos_y, x + pos_x, "X", curses.A_BOLD | curses.A_REVERSE)

def draw_ui(stdscr):
    global is_running, serial_port, rc_channels, aux_labels
    
    stdscr.clear()
    curses.curs_set(0)
    stdscr.nodelay(1)
    
    while is_running:
        stdscr.erase()
        stdscr.addstr(0, 0, "=== MSP TX Emulator Premium ===", curses.A_BOLD | curses.A_UNDERLINE)
        stdscr.addstr(1, 0, f"Port: {serial_port.port if serial_port else 'None'} | 50Hz", curses.A_DIM)
        
        # Joysticks
        draw_stick(stdscr, 4, 2, rc_channels[3], rc_channels[2], "THR / YAW")
        draw_stick(stdscr, 4, 25, rc_channels[0], rc_channels[1], "PIT / ROL")
        
        # Channel values
        stdscr.addstr(12, 2, f"T: {rc_channels[2]:4} Y: {rc_channels[3]:4}")
        stdscr.addstr(12, 25, f"R: {rc_channels[0]:4} P: {rc_channels[1]:4}")

        # AUX Channels
        stdscr.addstr(4, 50, "AUX Controls:", curses.A_BOLD)
        for i in range(4):
            val = rc_channels[4+i]
            label = aux_labels.get(i, f"AUX {i+1}")
            status = "[ACTIVE]" if val > 1500 else "[      ]"
            attr = curses.A_REVERSE if val > 1500 else curses.A_NORMAL
            stdscr.addstr(6+i, 50, f"({i+1}) {label:10} : {val:4} {status}", attr)

        # Instructions
        stdscr.addstr(14, 0, "-" * 80)
        stdscr.addstr(15, 0, "Sticks: W/S (Pit), A/D (Rol), Up/Dn (Thr), L/R (Yaw) | C: Center")
        stdscr.addstr(16, 0, "Modes: Space (AUX 1), 2 (AUX 2), 3 (AUX 3), 4 (AUX 4) | Q: Quit")
        
        stdscr.refresh()
        key = stdscr.getch()
        if key != -1:
            if key in [ord('q'), ord('Q')]: is_running = False
            elif key == ord(' '): rc_channels[4] = 2000 if rc_channels[4] < 1500 else 1000
            elif key == ord('2'): rc_channels[5] = 2000 if rc_channels[5] < 1500 else 1000
            elif key == ord('3'): rc_channels[6] = 2000 if rc_channels[6] < 1500 else 1000
            elif key == ord('4'): rc_channels[7] = 2000 if rc_channels[7] < 1500 else 1000
            elif key in [ord('w'), ord('W')]: rc_channels[1] = min(2000, rc_channels[1] + 50)
            elif key in [ord('s'), ord('S')]: rc_channels[1] = max(1000, rc_channels[1] - 50)
            elif key in [ord('a'), ord('A')]: rc_channels[0] = max(1000, rc_channels[0] - 50)
            elif key in [ord('d'), ord('D')]: rc_channels[0] = min(2000, rc_channels[0] + 50)
            elif key == curses.KEY_UP: rc_channels[2] = min(2000, rc_channels[2] + 50)
            elif key == curses.KEY_DOWN: rc_channels[2] = max(1000, rc_channels[2] - 50)
            elif key == curses.KEY_LEFT: rc_channels[3] = max(1000, rc_channels[3] - 50)
            elif key == curses.KEY_RIGHT: rc_channels[3] = min(2000, rc_channels[3] + 50)
            elif key in [ord('c'), ord('C')]:
                rc_channels[0], rc_channels[1], rc_channels[3] = 1500, 1500, 1500
                
        time.sleep(0.02)

def main_curses(stdscr, port, baud, config):
    global serial_port, aux_labels
    if config:
        mapping = parse_config(config)
        for idx, name in mapping.items():
            if idx < 4: aux_labels[idx] = name
            
    if not port:
        port = select_port(stdscr)
        if not port: return

    try:
        serial_port = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        stdscr.clear()
        stdscr.addstr(0, 0, f"Failed: {e}")
        stdscr.refresh()
        stdscr.getch()
        return

    sender_thread = threading.Thread(target=msp_sender_thread)
    sender_thread.daemon = True
    sender_thread.start()
    draw_ui(stdscr)

def main():
    parser = argparse.ArgumentParser(description="MSP TX Emulator Premium")
    parser.add_argument('--port', default='', help="Serial port")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate")
    parser.add_argument('--config', default='', help="Path to Betaflight .txt config to parse AUX labels")
    
    args, _ = parser.parse_known_args()
    curses.wrapper(main_curses, args.port, args.baud, args.config)
    
    global is_running, serial_port
    is_running = False
    if serial_port and serial_port.is_open: serial_port.close()

if __name__ == '__main__':
    main()
