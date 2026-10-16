"""Keyboard teach pendant with dead man's switch.

    ros2 run scara_control teach_pendant

The window must have keyboard focus.

    HOLD  SPACE        dead man's switch (enable). Release = motors stop.
    Q / A              joint 1  + / -        (only while SPACE is held)
    W / S              joint 2  + / -
    E / D              P up / P down (P down -> valve ON automatically)
    R / F              valve ON / OFF
    + / -              jog speed (10 % ... 100 %)
    H                  HOME routine
    X  or  Esc         STOP (motors off, aborts any motion)
    C                  reset fault
    T                  teach: store current pose in the list
    Enter              go to the x, y, z typed in the entry boxes

Safety chain for the dead man's switch:
  1. SPACE released -> deadman=False is published at once.
  2. Window loses focus -> every key is considered released.
  3. Pendant closed / crashed / cable unplugged -> the heartbeat stops and the
     firmware stops the motors after DEADMAN_TIMEOUT_MS (250 ms).
"""
import os
import time
import tkinter as tk
from tkinter import filedialog, ttk

import rclpy
import yaml
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32MultiArray, Int32, String

from scara_control import protocol as P

PERIOD_MS = 50                 # 20 Hz heartbeat (firmware timeout is 250 ms)
RELEASE_DEBOUNCE_MS = 60       # X11 key auto-repeat sends release+press pairs

JOG_KEYS = {'q': (0, +1), 'a': (0, -1), 'w': (1, +1), 's': (1, -1)}


class PendantNode(Node):
    def __init__(self):
        super().__init__('teach_pendant')
        self.pub_deadman = self.create_publisher(Bool, P.TOPIC_DEADMAN, qos_profile_sensor_data)
        self.pub_jog = self.create_publisher(Vector3, P.TOPIC_JOG, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Int32, P.TOPIC_CMD, 10)
        self.pub_pose = self.create_publisher(Point, P.TOPIC_CMD_POSE, 10)
        self.state = None
        self.state_time = 0.0
        self.events = []
        self.create_subscription(Float32MultiArray, P.TOPIC_STATE, self.on_state, qos_profile_sensor_data)
        self.create_subscription(String, P.TOPIC_EVENT, lambda m: self.events.append(m.data), 50)

    def on_state(self, msg):
        if len(msg.data) >= P.ST_LEN:
            self.state = list(msg.data)
            self.state_time = time.monotonic()


class Pendant:
    def __init__(self, node):
        self.node = node
        self.pressed = set()
        self.release_jobs = {}
        self.speed = 0.5
        self.taught = []

        self.root = tk.Tk()
        self.root.title('SCARA teach pendant')
        self.root.geometry('760x620')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind('<KeyPress>', self.on_press)
        self.root.bind('<KeyRelease>', self.on_release)
        self.root.bind('<FocusOut>', lambda e: self.release_all())
        self._build_ui()
        self.root.after(PERIOD_MS, self.tick)

    # ------------------------------------------------------------------- UI
    def _build_ui(self):
        pad = {'padx': 6, 'pady': 3}
        top = ttk.Frame(self.root)
        top.pack(fill='x', **pad)
        self.deadman_lbl = tk.Label(top, text='DEADMAN: RELEASED', font=('Sans', 16, 'bold'),
                                    bg='#b22', fg='white', width=24)
        self.deadman_lbl.pack(side='left', **pad)
        self.conn_lbl = tk.Label(top, text='ESP32: --', font=('Sans', 12), width=18)
        self.conn_lbl.pack(side='left', **pad)
        self.speed_lbl = ttk.Label(top, text='', font=('Sans', 12))
        self.speed_lbl.pack(side='left', **pad)

        grid = ttk.Frame(self.root)
        grid.pack(fill='x', **pad)
        self.vals = {}
        rows = [('Mode', 'mode'), ('Homed', 'homed'), ('Error', 'error'),
                ('q1 / q2 [deg]', 'q'), ('goal q1 / q2 [deg]', 'qg'),
                ('x / y / z [mm]', 'xyz'), ('PWM u1 / u2', 'u'),
                ('P / valve', 'io'), ('Limits J1 R/L  J2 R/L', 'lim')]
        for r, (txt, key) in enumerate(rows):
            ttk.Label(grid, text=txt + ':', font=('Sans', 11)).grid(row=r, column=0, sticky='e', padx=4)
            v = ttk.Label(grid, text='--', font=('Monospace', 11, 'bold'))
            v.grid(row=r, column=1, sticky='w', padx=4)
            self.vals[key] = v

        goto = ttk.LabelFrame(self.root, text='Point to point (mm)  - Enter = GO')
        goto.pack(fill='x', **pad)
        self.entries = []
        for i, axis in enumerate('xyz'):
            ttk.Label(goto, text=axis).pack(side='left')
            e = ttk.Entry(goto, width=8)
            e.insert(0, ['200', '100', '65'][i])
            e.pack(side='left', padx=3)
            e.bind('<Return>', lambda ev: self.send_goto())
            self.entries.append(e)
        ttk.Button(goto, text='GO', command=self.send_goto).pack(side='left', padx=6)

        teach = ttk.LabelFrame(self.root, text='Taught points (T = teach)')
        teach.pack(fill='x', **pad)
        self.teach_list = tk.Listbox(teach, height=4, font=('Monospace', 10))
        self.teach_list.pack(side='left', fill='x', expand=True)
        btns = ttk.Frame(teach)
        btns.pack(side='left')
        ttk.Button(btns, text='Go to selected', command=self.goto_selected).pack(fill='x')
        ttk.Button(btns, text='Save YAML', command=self.save_taught).pack(fill='x')
        ttk.Button(btns, text='Clear', command=self.clear_taught).pack(fill='x')

        help_txt = ('HOLD SPACE = deadman | Q/A: J1 +/-  W/S: J2 +/- | E/D: P up/down | '
                    'R/F: valve on/off | +/-: speed | H: home | X/Esc: STOP | C: reset | T: teach')
        ttk.Label(self.root, text=help_txt, wraplength=740, foreground='#555').pack(**pad)

        self.log = tk.Listbox(self.root, height=9, font=('Monospace', 9))
        self.log.pack(fill='both', expand=True, **pad)

    # ------------------------------------------------------------- keyboard
    @staticmethod
    def _key(event):
        k = event.keysym.lower()
        return {'plus': '+', 'kp_add': '+', 'minus': '-', 'kp_subtract': '-'}.get(k, k)

    def on_press(self, event):
        if isinstance(event.widget, tk.Entry) or isinstance(event.widget, ttk.Entry):
            return  # typing coordinates
        k = self._key(event)
        job = self.release_jobs.pop(k, None)
        if job is not None:            # auto-repeat: cancel the fake release
            self.root.after_cancel(job)
        first = k not in self.pressed
        self.pressed.add(k)
        if first:
            self.on_key_down(k)

    def on_release(self, event):
        k = self._key(event)
        if k in self.release_jobs:
            return
        # Wait a few ms: with X11 auto-repeat a held key produces release+press
        # pairs. Only a release NOT followed by a press is a real release.
        self.release_jobs[k] = self.root.after(RELEASE_DEBOUNCE_MS, self._really_released, k)

    def _really_released(self, k):
        self.release_jobs.pop(k, None)
        self.pressed.discard(k)
        if k == 'space':               # deadman released: tell the robot right now
            self.publish_heartbeat()

    def release_all(self):
        self.pressed.clear()
        self.publish_heartbeat()

    def deadman(self):
        return 'space' in self.pressed

    def on_key_down(self, k):
        cmd = {'h': P.CMD_HOME, 'x': P.CMD_STOP, 'escape': P.CMD_STOP, 'c': P.CMD_RESET_FAULT,
               'r': P.CMD_VALVE_ON, 'f': P.CMD_VALVE_OFF}.get(k)
        if k in ('e', 'd'):            # P motion only with the deadman held
            if self.deadman():
                cmd = P.CMD_P_UP if k == 'e' else P.CMD_P_DOWN
            else:
                self.add_log('hold SPACE (deadman) to move the P axis')
        if cmd is not None:
            self.node.pub_cmd.publish(Int32(data=cmd))
        elif k == '+':
            self.speed = min(1.0, self.speed + 0.1)
        elif k == '-':
            self.speed = max(0.1, self.speed - 0.1)
        elif k == 't':
            self.teach_current()

    # ------------------------------------------------------------- commands
    def send_goto(self):
        try:
            x, y, z = (float(e.get()) / 1000.0 for e in self.entries)
        except ValueError:
            self.add_log('invalid x, y, z')
            return
        self.node.pub_pose.publish(Point(x=x, y=y, z=z))
        self.root.focus_set()

    def teach_current(self):
        s = self.node.state
        if s is None:
            return
        pt = [round(s[P.ST_X] * 1e3, 2), round(s[P.ST_Y] * 1e3, 2), round(s[P.ST_Z] * 1e3, 2)]
        self.taught.append(pt)
        self.teach_list.insert('end', f'P{len(self.taught)}: x={pt[0]:.1f} y={pt[1]:.1f} z={pt[2]:.1f} mm')

    def goto_selected(self):
        sel = self.teach_list.curselection()
        if sel:
            x, y, z = self.taught[sel[0]]
            self.node.pub_pose.publish(Point(x=x / 1e3, y=y / 1e3, z=z / 1e3))
        self.root.focus_set()

    def save_taught(self):
        path = filedialog.asksaveasfilename(defaultextension='.yaml', initialfile='taught_points.yaml',
                                            initialdir=os.getcwd())
        if path:
            with open(path, 'w') as f:
                yaml.safe_dump({'units': 'mm', 'points': self.taught}, f)
            self.add_log(f'saved {len(self.taught)} points to {path}')
        self.root.focus_set()

    def clear_taught(self):
        self.taught.clear()
        self.teach_list.delete(0, 'end')

    # ---------------------------------------------------------------- loop
    def publish_heartbeat(self):
        dm = self.deadman()
        jog = [0.0, 0.0]
        if dm:
            for k, (j, sign) in JOG_KEYS.items():
                if k in self.pressed:
                    jog[j] += sign * self.speed
        self.node.pub_deadman.publish(Bool(data=dm))
        self.node.pub_jog.publish(Vector3(x=jog[0], y=jog[1], z=0.0))

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.publish_heartbeat()
        self.refresh()
        self.root.after(PERIOD_MS, self.tick)

    def add_log(self, text):
        self.log.insert('end', time.strftime('%H:%M:%S ') + text)
        self.log.see('end')

    def refresh(self):
        while self.node.events:
            self.add_log(self.node.events.pop(0))

        dm = self.deadman()
        self.deadman_lbl.config(text='DEADMAN: ENABLED' if dm else 'DEADMAN: RELEASED',
                                bg='#2a2' if dm else '#b22')
        self.speed_lbl.config(text=f'jog speed {self.speed * 100:.0f} %')

        s = self.node.state
        online = s is not None and time.monotonic() - self.node.state_time < 1.0
        self.conn_lbl.config(text='ESP32: online' if online else 'ESP32: OFFLINE',
                             fg='#080' if online else '#b00')
        if s is None:
            return
        f = P.flags_of(s)
        deg = 57.29578
        mode = int(s[P.ST_MODE])
        self.vals['mode'].config(text=P.MODE_NAMES[mode] if mode < len(P.MODE_NAMES) else str(mode))
        self.vals['homed'].config(text='YES' if f & P.FLAG_HOMED else 'NO (press H)')
        self.vals['error'].config(text=P.ERR_NAMES[int(s[P.ST_ERROR])])
        self.vals['q'].config(text=f'{s[P.ST_Q1] * deg:8.2f} {s[P.ST_Q2] * deg:8.2f}')
        self.vals['qg'].config(text=f'{s[P.ST_Q1_GOAL] * deg:8.2f} {s[P.ST_Q2_GOAL] * deg:8.2f}'
                                    f'   elbow {"+" if s[P.ST_ELBOW] > 0 else "-"}')
        self.vals['xyz'].config(text=f'{s[P.ST_X] * 1e3:8.1f} {s[P.ST_Y] * 1e3:8.1f} {s[P.ST_Z] * 1e3:8.1f}')
        self.vals['u'].config(text=f'{s[P.ST_U1]:+.2f} {s[P.ST_U2]:+.2f}')
        self.vals['io'].config(text=f'{"DOWN" if f & P.FLAG_P_DOWN else "UP"} / '
                                    f'{"ON" if f & P.FLAG_VALVE else "OFF"}')
        lim = ''.join('#' if f & b else '.' for b in (P.FLAG_LIM1_R, P.FLAG_LIM1_L, P.FLAG_LIM2_R, P.FLAG_LIM2_L))
        self.vals['lim'].config(text=f'{lim[0]} {lim[1]}    {lim[2]} {lim[3]}')

    def close(self):
        self.pressed.clear()
        for _ in range(3):
            self.publish_heartbeat()
        self.root.destroy()

    def run(self):
        self.root.focus_force()
        self.root.mainloop()


def main():
    rclpy.init()
    node = PendantNode()
    try:
        Pendant(node).run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
