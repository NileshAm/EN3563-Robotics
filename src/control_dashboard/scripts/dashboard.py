#!/usr/bin/env python3
"""Editable waypoints; ROS is polled through the Tk event loop."""

from dataclasses import asdict, dataclass
import json
import math
import tkinter as tk
from tkinter import filedialog, ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool

from control_dashboard.msg import MoveProgress
from control_dashboard.srv import MoveToPose

HOME_POSE = (547.498, 0, 815.002, -90, 0, 0)
ACTIONS = {'Keep': 'keep', 'Grab': 'grab', 'Release': 'release'}
BUSY_STATES = ('planning', 'moving', 'settling', 'grabbing', 'releasing')


@dataclass(frozen=True)
class Waypoint:
    name: str
    pose: tuple
    linear: bool = True
    gripper: str = 'keep'

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('Enter a waypoint name')
        if (not isinstance(self.pose, (tuple, list)) or len(self.pose) != 6 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in self.pose)):
            raise ValueError('Enter six finite pose values')
        if type(self.linear) is not bool or self.gripper not in ACTIONS.values():
            raise ValueError('Choose a valid motion type and gripper action')

    def request(self):
        request = MoveToPose.Request()
        p, q = request.target.position, request.target.orientation
        p.x, p.y, p.z = [float(v) / 1000 for v in self.pose[:3]]
        q.x, q.y, q.z, q.w = map(float, Rotation.from_euler(
            'xyz', self.pose[3:], degrees=True).as_quat())
        request.move_linear = self.linear
        request.gripper_action = self.gripper
        return request


def default_waypoints():
    """Original flange poses, with grasp/release performed after arrival."""
    return [Waypoint(name, (x, y, z, 0, 90, 0), linear, action)
            for name, x, y, z, linear, action in [
                ('Approach', 450, -200, 470, False, 'keep'),
                ('Descend and grab', 450, -200, 170, True, 'grab'),
                ('Lift', 450, -200, 470, True, 'keep'),
                ('Transfer', 500, 500, 470, True, 'keep'),
                ('Place and release', 500, 500, 372, True, 'release'),
                ('Retreat', 500, 500, 470, True, 'keep'),
            ]]


def read_waypoints(path):
    with open(path, encoding='utf-8') as source:
        data = json.load(source)
    if not isinstance(data, list) or not data:
        raise ValueError('Waypoint file must contain a nonempty list')
    try:
        return [Waypoint(**item) for item in data]
    except (TypeError, KeyError) as error:
        raise ValueError('Invalid waypoint file') from error


class Dashboard:
    def __init__(self, root):
        self.root = root
        self.node = Node('control_dashboard')
        self.client = self.node.create_client(MoveToPose, '/robot/move_to_pose')
        self.subscription = self.node.create_subscription(
            MoveProgress, '/robot/move_progress', self.on_progress,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.gripper_subscription = self.node.create_subscription(
            Bool, '/gripper/attached', self.on_gripper, 10)
        self.pending = None
        self.moving = False
        self.plan = None
        self.plan_index = 0
        self.plan_result = None
        self.progress_started = False
        self.waypoints = default_waypoints()
        self.edit_controls = []
        root.title('Robot waypoint dashboard')
        root.minsize(980, 520)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        frame = ttk.Frame(root, padding=18)
        frame.grid(sticky='nsew')
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)
        ttk.Label(frame, text='Waypoints', font=('', 16, 'bold')).grid(
            row=0, column=0, sticky='w', pady=(0, 10))
        ttk.Label(frame, text='World → Link6 · position in mm · angles in degrees').grid(
            row=0, column=1, sticky='w', padx=(20, 0))
        columns = ('name', 'x', 'y', 'z', 'motion', 'gripper')
        self.table = ttk.Treeview(frame, columns=columns, show='headings', selectmode='browse')
        for key, title, width in zip(columns, ('Waypoint', 'X', 'Y', 'Z', 'Motion', 'Gripper'),
                                     (190, 65, 65, 65, 80, 80)):
            self.table.heading(key, text=title)
            self.table.column(key, width=width, stretch=key == 'name')
        self.table.grid(row=1, column=0, sticky='nsew')
        self.table.bind('<<TreeviewSelect>>', self.select_waypoint)
        editor = ttk.Frame(frame, padding=(20, 0, 0, 0))
        editor.grid(row=1, column=1, sticky='n')
        self.name = tk.StringVar()
        ttk.Label(editor, text='Name').grid(row=0, column=0, sticky='w')
        entry = ttk.Entry(editor, textvariable=self.name, width=24)
        entry.grid(row=0, column=1, pady=3)
        self.edit_controls.append(entry)
        self.fields = []
        for row, label in enumerate(('X (mm)', 'Y (mm)', 'Z (mm)', 'Roll (deg)',
                                     'Pitch (deg)', 'Yaw (deg)'), 1):
            ttk.Label(editor, text=label).grid(row=row, column=0, sticky='w')
            field = ttk.Entry(editor, width=24)
            field.grid(row=row, column=1, pady=3)
            self.fields.append(field)
            self.edit_controls.append(field)
        self.linear = tk.BooleanVar(value=True)
        linear = ttk.Checkbutton(editor, text='Straight Cartesian path (MoveL)', variable=self.linear)
        linear.grid(row=7, columnspan=2, pady=8)
        self.edit_controls.append(linear)
        self.action = tk.StringVar(value='Keep')
        ttk.Label(editor, text='On arrival').grid(row=8, column=0, sticky='w')
        action = ttk.Combobox(editor, textvariable=self.action, values=list(ACTIONS),
                              state='readonly', width=21)
        action.grid(row=8, column=1, pady=3)
        self.edit_controls.append(action)
        ttk.Label(editor, text='Keep holds the current gripper state.').grid(
            row=9, columnspan=2, pady=5)
        self.send = ttk.Button(editor, text='Move to waypoint', command=self.request_move)
        self.send.grid(row=10, columnspan=2, sticky='ew', pady=(8, 0))
        self.home = ttk.Button(editor, text='Move home', command=self.request_home)
        self.home.grid(row=11, columnspan=2, sticky='ew', pady=(6, 0))
        self.execute = ttk.Button(editor, text='Execute motion plan', command=self.request_plan)
        self.execute.grid(row=12, columnspan=2, sticky='ew', pady=(12, 0))
        toolbar = ttk.Frame(frame)
        toolbar.grid(row=2, column=0, sticky='w', pady=10)
        for label, command in [('Add waypoint', self.add_waypoint), ('Update', self.update_waypoint),
                               ('Remove', self.remove_waypoint), ('Defaults', self.restore_defaults),
                               ('Save…', self.save), ('Load…', self.load)]:
            button = ttk.Button(toolbar, text=label, command=command)
            button.pack(side='left', padx=(0, 5))
            self.edit_controls.append(button)
        self.percent = tk.DoubleVar(value=0)
        ttk.Progressbar(frame, maximum=100, variable=self.percent).grid(
            row=3, columnspan=2, sticky='ew', pady=(8, 12))
        ttk.Style(root).configure('DashboardError.TLabel', foreground='#c62828')
        self.status = tk.StringVar(value='Choose a waypoint and click Move. No motion starts automatically.')
        self.status_label = ttk.Label(frame, textvariable=self.status, wraplength=900)
        self.status_label.grid(row=4, columnspan=2, sticky='w')
        self.connection = tk.StringVar()
        ttk.Label(frame, textvariable=self.connection).grid(row=5, column=0, sticky='w', pady=(8, 0))
        self.gripper_status = tk.StringVar(value='Gripper: waiting for feedback')
        ttk.Label(frame, textvariable=self.gripper_status).grid(row=5, column=1, sticky='w', pady=(8, 0))
        self.refresh(0)
        self.send.state(['disabled'])
        self.home.state(['disabled'])
        self.execute.state(['disabled'])
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.poll_id = root.after(50, self.poll)

    def selected(self):
        selection = self.table.selection()
        return int(selection[0]) if selection else None

    def refresh(self, selected=0):
        self.table.delete(*self.table.get_children())
        for index, waypoint in enumerate(self.waypoints):
            self.table.insert('', 'end', iid=str(index), values=(
                waypoint.name, *[f'{v:g}' for v in waypoint.pose[:3]],
                'MoveL' if waypoint.linear else 'MoveJ', waypoint.gripper.capitalize()))
        if self.waypoints:
            selected = min(selected, len(self.waypoints) - 1)
            self.table.selection_set(str(selected))
            self.table.see(str(selected))
            self.select_waypoint()

    def select_waypoint(self, _event=None):
        index = self.selected()
        if index is None:
            return
        waypoint = self.waypoints[index]
        self.name.set(waypoint.name)
        # Temporarily enable fields so browsing the list also works during motion.
        for field, value in zip(self.fields, waypoint.pose):
            disabled = field.instate(['disabled'])
            field.state(['!disabled'])
            field.delete(0, tk.END)
            field.insert(0, f'{value:g}')
            if disabled:
                field.state(['disabled'])
        self.linear.set(waypoint.linear)
        self.action.set(waypoint.gripper.capitalize())

    def edited_waypoint(self):
        try:
            pose = tuple(float(field.get()) for field in self.fields)
        except ValueError as error:
            raise ValueError('Enter numbers in all six pose fields') from error
        return Waypoint(self.name.get().strip(), pose, self.linear.get(), ACTIONS[self.action.get()])

    def add_waypoint(self):
        try:
            self.waypoints.append(self.edited_waypoint())
            self.refresh(len(self.waypoints) - 1)
        except ValueError as error:
            self.set_status(str(error), error=True)

    def update_waypoint(self):
        try:
            index = self.selected()
            if index is None:
                raise ValueError('Select a waypoint first, or click Add waypoint')
            self.waypoints[index] = self.edited_waypoint()
            self.refresh(index)
            return True
        except ValueError as error:
            self.set_status(str(error), error=True)
            return False

    def remove_waypoint(self):
        index = self.selected()
        if index is not None:
            del self.waypoints[index]
            self.refresh(max(0, index - 1))

    def restore_defaults(self):
        self.waypoints = default_waypoints()
        self.refresh(0)
        self.set_status('Original pick-and-place waypoints restored. Click Move for each step.')

    def save(self):
        if not self.waypoints:
            self.set_status('Add a waypoint before saving', error=True)
            return
        if self.selected() is not None and not self.update_waypoint():
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension='.json',
                                            filetypes=[('Waypoints', '*.json')])
        if path:
            try:
                with open(path, 'w', encoding='utf-8') as output:
                    json.dump([asdict(w) for w in self.waypoints], output, indent=2)
                self.set_status('Waypoints saved')
            except OSError as error:
                self.set_status(str(error), error=True)

    def load(self):
        path = filedialog.askopenfilename(parent=self.root, filetypes=[('Waypoints', '*.json')])
        if path:
            try:
                waypoints = read_waypoints(path)
                self.waypoints = waypoints
                self.refresh(0)
                self.set_status('Waypoints loaded. Select one and click Move.')
            except (OSError, ValueError) as error:
                self.set_status(str(error), error=True)

    def request_home(self):
        if self.plan is None:
            self.start_move(Waypoint('Home', HOME_POSE, False, 'keep'))

    def request_move(self):
        if self.pending is not None or self.moving or self.plan is not None:
            return
        if self.update_waypoint():
            self.start_move(self.waypoints[self.selected()])

    def request_plan(self):
        if self.pending is not None or self.moving or self.plan is not None:
            return
        if not self.waypoints:
            self.set_status('Add a waypoint before executing a motion plan', error=True)
            return
        if self.selected() is not None and not self.update_waypoint():
            return
        # Freeze the list for this run, including the selected row's current edits.
        self.plan = tuple(self.waypoints)
        self.plan_index = 0
        self.start_plan_waypoint()

    def start_plan_waypoint(self):
        self.table.selection_set(str(self.plan_index))
        self.table.see(str(self.plan_index))
        self.select_waypoint()
        if not self.start_move(self.plan[self.plan_index]):
            self.stop_plan(self.status.get())

    def stop_plan(self, message):
        step = self.plan_index + 1
        self.plan = None
        self.plan_result = None
        self.set_status(f'Motion plan stopped at waypoint {step}: {message}', error=True)

    def start_move(self, waypoint):
        if self.pending is not None or self.moving:
            return False
        if not self.client.service_is_ready():
            self.set_status('Waiting for /robot/move_to_pose', error=True)
            return False
        try:
            self.progress_started = False
            self.plan_result = None
            self.pending = self.client.call_async(waypoint.request())
            self.moving = True
            self.percent.set(100 * self.plan_index / len(self.plan) if self.plan else 0)
            prefix = f'Plan {self.plan_index + 1}/{len(self.plan)} — ' if self.plan else ''
            self.set_status(f'{prefix}Planning {waypoint.name}…')
            self.set_enabled(False)
            return True
        except (ValueError, RuntimeError) as error:
            self.set_status(str(error), error=True)
            return False

    def set_status(self, message, error=False):
        self.status.set(message)
        self.status_label.configure(style='DashboardError.TLabel' if error else 'TLabel')

    def on_progress(self, msg):
        if msg.state in BUSY_STATES:
            self.progress_started = True
        if self.plan is not None and not self.progress_started:
            # A latched result from before this request cannot advance the plan.
            return
        self.moving = msg.state in BUSY_STATES
        percent = ((100 * self.plan_index + msg.percent) / len(self.plan)
                   if self.plan else msg.percent)
        self.percent.set(percent)
        prefix = (f'Plan {self.plan_index + 1}/{len(self.plan)} '
                  f'({self.plan[self.plan_index].name}) — ' if self.plan else '')
        self.set_status(f'{prefix}{msg.state}: {msg.percent:.0f}% — {msg.message}',
                        error=msg.state == 'failed')
        if self.plan is not None and msg.state in ('succeeded', 'failed'):
            self.plan_result = (msg.state, msg.message)

    def on_gripper(self, msg):
        self.gripper_status.set('Gripper: holding object' if msg.data else 'Gripper: no object attached')

    def set_enabled(self, enabled):
        state = ['!disabled'] if enabled and self.plan is None else ['disabled']
        self.send.state(state)
        self.home.state(state)
        self.execute.state(state)
        # Editing is available while disconnected, but locked during an active request.
        edit_state = (['disabled'] if self.moving or self.pending is not None or self.plan is not None
                      else ['!disabled'])
        for widget in self.edit_controls:
            widget.state(edit_state)

    def poll(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.pending is not None and self.pending.done():
            try:
                response = self.pending.result()
                if not response.accepted:
                    self.moving = False
                    if self.plan is not None:
                        self.stop_plan(response.message)
                    else:
                        self.set_status(response.message, error=True)
            except Exception as error:
                message = f'Service call failed: {error}'
                if self.plan is not None:
                    self.stop_plan(message)
                else:
                    self.set_status(message, error=True)
                self.moving = False
            self.pending = None
        ready = self.client.service_is_ready()
        if self.plan is not None:
            if not ready:
                self.stop_plan('Motion controller disconnected')
            elif self.pending is None and self.plan_result is not None:
                state, message = self.plan_result
                self.plan_result = None
                if state == 'failed':
                    self.stop_plan(message)
                elif self.plan_index + 1 == len(self.plan):
                    count = len(self.plan)
                    self.plan = None
                    self.percent.set(100)
                    self.set_status(f'Motion plan completed: {count} waypoints')
                else:
                    self.plan_index += 1
                    self.start_plan_waypoint()
        self.connection.set('Motion controller connected' if ready else 'Waiting for /robot/move_to_pose')
        self.set_enabled(ready and not self.moving and self.pending is None)
        self.poll_id = self.root.after(50, self.poll)

    def close(self):
        # The controller completes an accepted waypoint even if this window closes.
        self.plan = None  # Do not schedule any remaining waypoints after closing.
        self.root.after_cancel(self.poll_id)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def main():
    rclpy.init()
    root = tk.Tk()
    try:
        Dashboard(root)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
