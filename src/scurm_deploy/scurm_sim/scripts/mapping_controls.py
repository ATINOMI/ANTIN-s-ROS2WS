#!/usr/bin/env python3
"""Hold-to-drive controls for the isolated FAST-LIO2 simulator."""
import math
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = Node('scurm_mapping_controls')
    publisher = node.create_publisher(TwistStamped, '/scurm/cmd_vel_smoothed', 1)
    root = tk.Tk()
    root.title('SCURM FAST-LIO2 仿真控制')
    root.geometry('440x300+40+80')
    command = [0.0, 0.0]
    clearance = {'front': 0.0, 'back': 0.0, 'received': None}
    status = tk.StringVar(value='按住按钮或 W/A/S/D 移动，松开停止')

    def scan(msg):
        for name, center in [('front', 0.0), ('back', math.pi)]:
            ranges = [value for index, value in enumerate(msg.ranges)
                      if abs(math.remainder(msg.angle_min + index * msg.angle_increment - center,
                                            2.0 * math.pi)) < 0.5
                      and value >= msg.range_min and not math.isnan(value)]
            clearance[name] = min(ranges, default=0.0)
        clearance['received'] = node.get_clock().now()

    subscription = node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)

    def set_command(v, w):
        command[:] = [v, w]

    tk.Label(root, text='FAST-LIO2 三维建图', font=('Sans', 17)).pack(pady=8)
    tk.Label(root, text='W 前进  S 后退  A 左转  D 右转\n空格停止；窗口失去焦点自动停止').pack()
    frame = tk.Frame(root)
    frame.pack(pady=8)
    for label, v, w, row, column in [
        ('前进 W', 0.15, 0.0, 0, 1), ('左转 A', 0.0, 0.4, 1, 0),
        ('停止', 0.0, 0.0, 1, 1), ('右转 D', 0.0, -0.4, 1, 2),
        ('后退 S', -0.12, 0.0, 2, 1)]:
        button = tk.Button(frame, text=label, width=10)
        button.grid(row=row, column=column, padx=3, pady=3)
        button.bind('<ButtonPress-1>', lambda event, v=v, w=w: set_command(v, w))
        button.bind('<ButtonRelease-1>', lambda event: set_command(0.0, 0.0))
    for key, (v, w) in {'w': (0.15, 0.0), 's': (-0.12, 0.0),
                       'a': (0.0, 0.4), 'd': (0.0, -0.4)}.items():
        root.bind(f'<KeyPress-{key}>', lambda event, v=v, w=w: set_command(v, w))
        root.bind(f'<KeyRelease-{key}>', lambda event: set_command(0.0, 0.0))
    root.bind('<space>', lambda event: set_command(0.0, 0.0))
    root.bind('<FocusOut>', lambda event: set_command(0.0, 0.0))
    tk.Label(root, textvariable=status, wraplength=425).pack()

    def update():
        rclpy.spin_once(node, timeout_sec=0.0)
        out = TwistStamped()
        out.header.stamp = node.get_clock().now().to_msg()
        out.header.frame_id = 'base_footprint'
        v, w = command
        received = clearance['received']
        fresh = received is not None and 0.0 <= (node.get_clock().now() - received).nanoseconds * 1e-9 < 0.5
        if v and (not fresh or clearance['front' if v > 0 else 'back'] < 0.45):
            v = 0.0
            status.set('激光过期或前后距离不足 0.45 m：已阻止平移')
        else:
            status.set('按住按钮或 W/A/S/D 移动，松开停止')
        out.twist.linear.x = v
        out.twist.angular.z = w
        publisher.publish(out)
        root.after(50, update)

    def close():
        publisher.publish(TwistStamped())
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', close)
    root.after(50, update)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            publisher.publish(TwistStamped())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
