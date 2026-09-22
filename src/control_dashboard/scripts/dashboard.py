#!/usr/bin/env python3
"""Small Tk dashboard; ROS is polled through the GUI event loop."""

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation

from control_dashboard.msg import MoveProgress
from control_dashboard.srv import MoveToPose

HOME_POSE = ('547.498', '0', '815.002', '-90', '0', '0')


class Dashboard:
    def __init__(self, root):
        self.root = root
        self.node = Node('control_dashboard')
        self.client = self.node.create_client(MoveToPose, '/robot/move_to_pose')
        self.subscription = self.node.create_subscription(
            MoveProgress, '/robot/move_progress', self.on_progress,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pending = None
        self.moving = False
        root.title('Robot pose control')
        frame = ttk.Frame(root, padding=20)
        frame.grid(sticky='nsew')
        ttk.Label(frame, text='Target: world → Link6').grid(row=0, columnspan=2, pady=(0, 12))
        self.fields = []
        labels = (
            'X (mm)', 'Y (mm)', 'Z (mm)',
            'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)',
        )
        for row, (label, default) in enumerate(zip(labels, HOME_POSE), 1):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky='w', pady=3)
            field = ttk.Entry(frame, width=22)
            field.insert(0, default)
            field.grid(row=row, column=1, pady=3)
            self.fields.append(field)
        self.linear = tk.BooleanVar(value=True)
        ttk.Checkbutton(frame, text='Straight Cartesian path (MoveL)', variable=self.linear).grid(
            row=7, columnspan=2, pady=8)
        self.send = ttk.Button(frame, text='Move to target', command=self.request_move)
        self.send.grid(row=8, columnspan=2, sticky='ew')
        self.home = ttk.Button(frame, text='Home', command=self.request_home)
        self.home.grid(row=9, columnspan=2, sticky='ew', pady=(6, 0))
        self.percent = tk.DoubleVar(value=0)
        ttk.Progressbar(frame, maximum=100, variable=self.percent).grid(
            row=10, columnspan=2, sticky='ew', pady=12)
        style = ttk.Style(root)
        style.configure('DashboardError.TLabel', foreground='#c62828')
        self.status = tk.StringVar(value='Waiting for turn_node…')
        self.status_label = ttk.Label(
            frame, textvariable=self.status, wraplength=360)
        self.status_label.grid(row=11, columnspan=2)
        self.connection = tk.StringVar()
        ttk.Label(frame, textvariable=self.connection).grid(
            row=12, columnspan=2, pady=(8, 0))
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(50, self.poll)

    def request_home(self):
        for field, value in zip(self.fields, HOME_POSE):
            field.delete(0, tk.END)
            field.insert(0, value)
        self.request_move(move_linear=False)

    def request_move(self, move_linear=None):
        try:
            values = [float(field.get()) for field in self.fields]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('Enter finite numbers in all six fields')
            request = MoveToPose.Request()
            p = request.target.position
            p.x, p.y, p.z = [value / 1000.0 for value in values[:3]]
            q = request.target.orientation
            q.x, q.y, q.z, q.w = map(float, Rotation.from_euler(
                'xyz', values[3:], degrees=True).as_quat())
            request.move_linear = (
                self.linear.get() if move_linear is None else move_linear)
            self.pending = self.client.call_async(request)
            self.moving = True
            self.percent.set(0)
            self.set_status('Planning target…')
            self.send.state(['disabled'])
            self.home.state(['disabled'])
        except (ValueError, RuntimeError) as error:
            self.set_status(str(error), error=True)

    def set_status(self, message, error=False):
        self.status.set(message)
        self.status_label.configure(
            style='DashboardError.TLabel' if error else 'TLabel')

    def on_progress(self, msg):
        self.moving = msg.state in ('planning', 'moving', 'settling')
        self.percent.set(msg.percent)
        self.set_status(
            f'{msg.state}: {msg.percent:.0f}% — {msg.message}',
            error=msg.state == 'failed',
        )

    def poll(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.pending is not None and self.pending.done():
            try:
                response = self.pending.result()
                self.set_status(response.message, error=not response.accepted)
                if not response.accepted:
                    self.moving = False
            except Exception as error:
                self.set_status(f'Service call failed: {error}', error=True)
                self.moving = False
            self.pending = None
        ready = self.client.service_is_ready()
        self.connection.set('turn_node connected' if ready else 'Waiting for /robot/move_to_pose')
        button_state = (
            ['!disabled']
            if ready and not self.moving and self.pending is None
            else ['disabled']
        )
        self.send.state(button_state)
        self.home.state(button_state)
        self.root.after(50, self.poll)

    def close(self):
        # Closing the dashboard does not cancel an already accepted robot move.
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def main():
    rclpy.init()
    root = tk.Tk()
    Dashboard(root)
    root.mainloop()


if __name__ == '__main__':
    main()
