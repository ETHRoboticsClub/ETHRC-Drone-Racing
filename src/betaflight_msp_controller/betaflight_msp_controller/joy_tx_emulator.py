#!/usr/bin/env python3
import sys
import time
import curses
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

# Axes mapping:
# 0: Roll (A/D, Left/Right)
# 1: Pitch (W/S, Up/Down)
# 2: Yaw (Left/Right arrow keys)
# 3: Throttle (Up/Down arrow keys)
# Default values: Roll=0.0, Pitch=0.0, Yaw=0.0, Throttle=-1.0 (lowest)
axes = [0.0, 0.0, 0.0, -1.0]

# Goal mapping: x, y, z (Default hovering at 1.5m height)
goal_pos = [0.0, 0.0, 1.5]

# Buttons mapping:
# 4: AUX1 (Arming) - Toggled by Space
# 5: AUX2 (Override) - Toggled by '2'
# 6: AUX3 - Toggled by '3'
# 7: AUX4 - Toggled by '4'
buttons = [0] * 8
button_labels = {
    4: "ARM (AUX1)",
    5: "OVERRIDE (AUX2)",
    6: "AUX 3",
    7: "AUX 4"
}

is_running = True

class JoyTxEmulator(Node):
    def __init__(self):
        super().__init__('joy_tx_emulator')
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.goal_pub = self.create_publisher(Point, '/drone_goal', 10)
        self.get_logger().info('Joy TX Emulator ROS Node Initialized')

    def publish_joy(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joy_emulator'
        msg.axes = list(axes)
        msg.buttons = list(buttons)
        self.pub.publish(msg)

    def publish_goal(self):
        msg = Point()
        msg.x = goal_pos[0]
        msg.y = goal_pos[1]
        msg.z = goal_pos[2]
        self.goal_pub.publish(msg)

def safe_addstr(stdscr, y, x, s, attr=curses.A_NORMAL):
    try:
        stdscr.addstr(y, x, s, attr)
    except curses.error:
        pass

def draw_stick(stdscr, y, x, val_h, val_v, label):
    # val_h and val_v are in [-1.0, 1.0]
    size = 7
    safe_addstr(stdscr, y-1, x + (size//2) - (len(label)//2), label, curses.A_BOLD)
    for i in range(size):
        safe_addstr(stdscr, y + i, x, "|" + " " * (size*2-1) + "|")
    safe_addstr(stdscr, y, x, "+" + "-" * (size*2-1) + "+")
    safe_addstr(stdscr, y + size - 1, x, "+" + "-" * (size*2-1) + "+")
    
    # Calculate position
    pos_x = int((val_h + 1.0) / 2.0 * (size*2-2)) + 1
    pos_y = int((1.0 - val_v) / 2.0 * (size-2)) + 1
    safe_addstr(stdscr, y + pos_y, x + pos_x, "X", curses.A_BOLD | curses.A_REVERSE)

def draw_ui(stdscr, node):
    global is_running, axes, buttons, goal_pos
    
    stdscr.clear()
    curses.curs_set(0)
    stdscr.nodelay(1)
    
    # Publish initial goal
    node.publish_goal()
    
    while is_running:
        stdscr.erase()
        safe_addstr(stdscr, 0, 0, "=== ROS 2 Joy TUI Emulator ===", curses.A_BOLD | curses.A_UNDERLINE)
        safe_addstr(stdscr, 1, 0, "Publishing to /joy & /drone_goal @ 50Hz", curses.A_DIM)
        
        # Joysticks
        # Left Stick: Yaw (axes[2]) / Throttle (axes[3])
        draw_stick(stdscr, 4, 2, axes[2], axes[3], "THR / YAW")
        # Right Stick: Roll (axes[0]) / Pitch (axes[1])
        draw_stick(stdscr, 4, 25, axes[0], axes[1], "PIT / ROL")
        
        # Values representation
        safe_addstr(stdscr, 12, 2, f"T: {axes[3]:6.2f} Y: {axes[2]:6.2f}")
        safe_addstr(stdscr, 12, 25, f"R: {axes[0]:6.2f} P: {axes[1]:6.2f}")

        # Buttons/AUX status
        safe_addstr(stdscr, 4, 50, "AUX / Buttons:", curses.A_BOLD)
        for i in [4, 5, 6, 7]:
            val = buttons[i]
            label = button_labels.get(i, f"Button {i}")
            status = "[ACTIVE]" if val == 1 else "[      ]"
            attr = curses.A_REVERSE if val == 1 else curses.A_NORMAL
            safe_addstr(stdscr, 6 + (i - 4), 50, f"({i}) {label:15} : {val} {status}", attr)

        # Target Goal status
        safe_addstr(stdscr, 10, 50, "Target Goal (X, Y, Z):", curses.A_BOLD)
        safe_addstr(stdscr, 11, 50, f"X: {goal_pos[0]:5.2f}  Y: {goal_pos[1]:5.2f}  Z: {goal_pos[2]:5.2f}", curses.A_REVERSE)

        # Instructions
        safe_addstr(stdscr, 14, 0, "-" * 80)
        safe_addstr(stdscr, 15, 0, "Sticks: W/S (Pitch), A/D (Roll) | Up/Down (Throttle), Left/Right (Yaw)")
        safe_addstr(stdscr, 16, 0, "Modes: Space (ARM/AUX1), 2 (OVERRIDE/AUX2), 3 (AUX3), 4 (AUX4)")
        safe_addstr(stdscr, 17, 0, "Goals: I/K (Goal X), J/L (Goal Y), [/] (Goal Z) | G: Set Goal manually")
        safe_addstr(stdscr, 18, 0, "Utilities: C (Center Sticks) | R (Reset Throttle) | Q (Quit)")
        
        stdscr.refresh()
        key = stdscr.getch()
        if key != -1:
            if key in [ord('q'), ord('Q')]:
                is_running = False
            elif key == ord(' '):
                buttons[4] = 1 if buttons[4] == 0 else 0
            elif key == ord('2'):
                buttons[5] = 1 if buttons[5] == 0 else 0
            elif key == ord('3'):
                buttons[6] = 1 if buttons[6] == 0 else 0
            elif key == ord('4'):
                buttons[7] = 1 if buttons[7] == 0 else 0
            # Pitch (W/S)
            elif key in [ord('w'), ord('W')]:
                axes[1] = min(1.0, axes[1] + 0.1)
            elif key in [ord('s'), ord('S')]:
                axes[1] = max(-1.0, axes[1] - 0.1)
            # Roll (A/D)
            elif key in [ord('a'), ord('A')]:
                axes[0] = max(-1.0, axes[0] - 0.1)
            elif key in [ord('d'), ord('D')]:
                axes[0] = min(1.0, axes[0] + 0.1)
            # Throttle (Up/Down arrow)
            elif key == curses.KEY_UP:
                axes[3] = min(1.0, axes[3] + 0.1)
            elif key == curses.KEY_DOWN:
                axes[3] = max(-1.0, axes[3] - 0.1)
            # Yaw (Left/Right arrow)
            elif key == curses.KEY_LEFT:
                axes[2] = max(-1.0, axes[2] - 0.1)
            elif key == curses.KEY_RIGHT:
                axes[2] = min(1.0, axes[2] + 0.1)
            # Goals Nudging
            elif key in [ord('i'), ord('I')]:
                goal_pos[0] += 0.1
                node.publish_goal()
            elif key in [ord('k'), ord('K')]:
                goal_pos[0] -= 0.1
                node.publish_goal()
            elif key in [ord('j'), ord('J')]:
                goal_pos[1] += 0.1
                node.publish_goal()
            elif key in [ord('l'), ord('L')]:
                goal_pos[1] -= 0.1
                node.publish_goal()
            elif key == ord('['):
                goal_pos[2] = max(0.0, goal_pos[2] - 0.1)
                node.publish_goal()
            elif key == ord(']'):
                goal_pos[2] += 0.1
                node.publish_goal()
            # Manual Goal setting
            elif key in [ord('g'), ord('G')]:
                curses.curs_set(1)
                curses.echo()
                safe_addstr(stdscr, 19, 0, "Enter Goal coordinates 'X Y Z' (e.g. 0.0 0.0 1.5): ")
                stdscr.refresh()
                try:
                    input_bytes = stdscr.getstr(19, 52, 20)
                    input_str = input_bytes.decode('utf-8').strip()
                    parts = input_str.split()
                    if len(parts) == 3:
                        goal_pos[0] = float(parts[0])
                        goal_pos[1] = float(parts[1])
                        goal_pos[2] = float(parts[2])
                        node.publish_goal()
                except Exception:
                    pass
                curses.curs_set(0)
                curses.noecho()
                stdscr.nodelay(1)
            # Utilities
            elif key in [ord('c'), ord('C')]:
                # Center attitude controls, preserve throttle
                axes[0], axes[1], axes[2] = 0.0, 0.0, 0.0
            elif key in [ord('r'), ord('R')]:
                # Reset throttle to minimum
                axes[3] = -1.0
                
        # Publish at each tick
        node.publish_joy()
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(0.02)
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(0.02)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTxEmulator()
    
    # Run curses wrapper
    try:
        curses.wrapper(draw_ui, node)
    except KeyboardInterrupt:
        pass
    finally:
        global is_running
        is_running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
