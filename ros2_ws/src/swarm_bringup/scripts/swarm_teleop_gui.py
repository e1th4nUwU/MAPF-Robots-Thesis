#!/usr/bin/env python3
"""Swarm Remote Control + Monitor GUI.

Run:
  ros2 run swarm_bringup swarm_teleop_gui.py

Each robot panel has:
  • D-pad   — hold to move, release to stop
  • Live telemetry — position (x, y, θ) and velocity (vx, ω) from /odom
  • Heading arrow  — rotates in real time

Global controls:
  • Speed slider   — affects all robots
  • STOP ALL       — immediate halt
"""

import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from navig_msgs.msg import RobotHealth

# ── Robot definitions ──────────────────────────────────────────────────────────

ROBOTS = [
    {'name': 'alvin',   'bg': '#c0392b', 'fg': 'white', 'label': 'ALVIN'},
    {'name': 'teodoro', 'bg': '#27ae60', 'fg': 'white', 'label': 'TEODORO'},
    {'name': 'simon',   'bg': '#2980b9', 'fg': 'white', 'label': 'SIMON'},
]

REPEAT_MS  = 80    # publish interval while button held → ~12 Hz
DISPLAY_MS = 100   # GUI telemetry refresh rate → 10 Hz

# ── Helpers ────────────────────────────────────────────────────────────────────

def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def _lighten(hex_color: str, amount: float = 0.28) -> str:
    r, g, b = (int(hex_color[i:i+2], 16) for i in (1, 3, 5))
    r = min(255, int(r + (255 - r) * amount))
    g = min(255, int(g + (255 - g) * amount))
    b = min(255, int(b + (255 - b) * amount))
    return f'#{r:02x}{g:02x}{b:02x}'

# ── ROS2 node ──────────────────────────────────────────────────────────────────

class SwarmNode(Node):
    """Publishes cmd_vel + subscribes to odom/health for all robots."""

    def __init__(self):
        super().__init__('swarm_teleop_gui')
        self._pubs = {}
        self._kill_pubs = {}
        # Telemetry: (x, y, yaw_deg, vx, omega)  — None until first message
        self.telem = {r['name']: None for r in ROBOTS}
        # Health: True = alive, None = unknown yet
        self.health = {r['name']: None for r in ROBOTS}

        for r in ROBOTS:
            name = r['name']
            self._pubs[name] = self.create_publisher(
                Twist, f'/{name}/cmd_vel', 10)
            self._kill_pubs[name] = self.create_publisher(
                Bool, f'/{name}/kill', 10)
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(msg, n), 10)
            self.create_subscription(
                RobotHealth, f'/{name}/health',
                lambda msg, n=name: self._health_cb(msg, n), 10)

    def send(self, name: str, lx: float, az: float) -> None:
        msg = Twist()
        msg.linear.x = float(lx)
        msg.angular.z = float(az)
        self._pubs[name].publish(msg)

    def stop(self, name: str) -> None:
        self.send(name, 0.0, 0.0)

    def stop_all(self) -> None:
        for r in ROBOTS:
            self.stop(r['name'])

    def kill(self, name: str) -> None:
        """Stop the robot immediately and signal the health monitor to mark it dead."""
        self.stop(name)
        self._kill_pubs[name].publish(Bool(data=True))

    def _odom_cb(self, msg: Odometry, name: str) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist
        self.telem[name] = (
            p.x,
            p.y,
            math.degrees(_quat_to_yaw(msg.pose.pose.orientation)),
            v.linear.x,
            v.angular.z,
        )

    def _health_cb(self, msg: RobotHealth, name: str) -> None:
        self.health[name] = msg.is_alive

# ── Heading canvas ─────────────────────────────────────────────────────────────

class HeadingCanvas(tk.Canvas):
    """Small arrow that rotates to show the robot's current heading."""

    R   = 28   # circle radius
    SZ  = 64

    def __init__(self, parent, color: str):
        super().__init__(parent, width=self.SZ, height=self.SZ,
                         bg='#16213e', highlightthickness=0)
        cx = cy = self.SZ // 2
        self.create_oval(cx - self.R, cy - self.R,
                         cx + self.R, cy + self.R,
                         outline=color, width=2)
        self._arrow = self.create_line(
            cx, cy, cx + self.R - 4, cy,
            arrow='last', fill=color, width=3)
        self._cx = cx
        self._cy = cy
        self._color = color

    def set_yaw(self, deg: float) -> None:
        rad = math.radians(deg)
        ex = self._cx + (self.R - 4) * math.cos(rad)
        ey = self._cy - (self.R - 4) * math.sin(rad)
        self.coords(self._arrow, self._cx, self._cy, ex, ey)

# ── Per-robot panel ────────────────────────────────────────────────────────────

class RobotPanel(tk.LabelFrame):

    def __init__(self, parent, robot: dict, node: SwarmNode,
                 speed_var: tk.DoubleVar):
        super().__init__(
            parent,
            text=robot['label'],
            fg=robot['bg'],
            font=('Helvetica', 13, 'bold'),
            bg='#16213e', padx=8, pady=6,
            relief='ridge', bd=3,
        )
        self._name = robot['name']
        self._node = node
        self._speed = speed_var
        self._after_id = None
        self._cmd = (0.0, 0.0)

        self._build(robot['bg'], robot['fg'])

    # ── UI construction ────────────────────────────────────────────────────────

    def _build(self, bg: str, fg: str) -> None:
        # ── D-pad ──────────────────────────────────────────────────────────────
        dpad = tk.Frame(self, bg='#16213e')
        dpad.pack(pady=(0, 6))

        btn_kw = dict(width=3, height=2, font=('Helvetica', 20),
                      relief='raised', bd=3, cursor='hand2',
                      bg=bg, fg=fg,
                      activebackground=_lighten(bg), activeforeground=fg)
        stop_kw = dict(btn_kw, bg='#2c3e50', fg='white',
                       activebackground='#4a6278')

        fwd  = tk.Button(dpad, text='↑', **btn_kw)
        back = tk.Button(dpad, text='↓', **btn_kw)
        left = tk.Button(dpad, text='←', **btn_kw)
        rght = tk.Button(dpad, text='→', **btn_kw)
        stop = tk.Button(dpad, text='■', **stop_kw)

        fwd .grid(row=0, column=1, padx=3, pady=3)
        left.grid(row=1, column=0, padx=3, pady=3)
        stop.grid(row=1, column=1, padx=3, pady=3)
        rght.grid(row=1, column=2, padx=3, pady=3)
        back.grid(row=2, column=1, padx=3, pady=3)

        def bind(btn, lx, az):
            btn.bind('<ButtonPress-1>',   lambda _: self._press(lx, az))
            btn.bind('<ButtonRelease-1>', lambda _: self._release())

        bind(fwd,   1.0,  0.0)
        bind(back, -1.0,  0.0)
        bind(left,  0.0,  1.0)
        bind(rght,  0.0, -1.0)
        stop.bind('<ButtonPress-1>', lambda _: self._release())

        # ── Telemetry display ──────────────────────────────────────────────────
        sep = tk.Frame(self, height=1, bg=bg)
        sep.pack(fill='x', pady=4)

        telem = tk.Frame(self, bg='#16213e')
        telem.pack(fill='x')

        # Heading arrow (left) + numbers (right)
        self._heading = HeadingCanvas(telem, bg)
        self._heading.pack(side='left', padx=(0, 8))

        nums = tk.Frame(telem, bg='#16213e')
        nums.pack(side='left', fill='x', expand=True)

        lbl_kw  = dict(bg='#16213e', fg='#7f8c8d',
                       font=('Courier', 10), anchor='w')
        val_kw  = dict(bg='#16213e', fg='#ecf0f1',
                       font=('Courier', 10, 'bold'), anchor='w')

        fields = [
            ('x',  'm  '), ('y', 'm  '), ('θ', '°  '),
            ('vx', 'm/s'), ('ω', 'r/s'),
        ]
        self._vars = {}
        for row, (key, unit) in enumerate(fields):
            tk.Label(nums, text=f'{key:>2}:', **lbl_kw).grid(
                row=row, column=0, sticky='w')
            var = tk.StringVar(value='---')
            self._vars[key] = var
            tk.Label(nums, textvariable=var, width=8, **val_kw).grid(
                row=row, column=1, sticky='w')
            tk.Label(nums, text=unit, **lbl_kw).grid(
                row=row, column=2, sticky='w')

        # Status dot (green = receiving, grey = waiting)
        dot_frame = tk.Frame(self, bg='#16213e')
        dot_frame.pack(fill='x', pady=(4, 0))
        self._status_dot = tk.Label(
            dot_frame, text='● waiting for odom…',
            bg='#16213e', fg='#7f8c8d', font=('Helvetica', 9))
        self._status_dot.pack(side='left')

        # Health indicator + KILL button
        health_frame = tk.Frame(self, bg='#16213e')
        health_frame.pack(fill='x', pady=(6, 0))

        self._health_label = tk.Label(
            health_frame, text='◉ HEALTH: unknown',
            bg='#16213e', fg='#7f8c8d', font=('Helvetica', 9, 'bold'))
        self._health_label.pack(side='left')

        tk.Button(
            health_frame, text='☠ KILL',
            bg='#1a1a2e', fg='#e74c3c',
            font=('Helvetica', 10, 'bold'),
            relief='raised', bd=2, cursor='hand2', padx=6,
            activebackground='#e74c3c', activeforeground='white',
            command=lambda: self._node.kill(self._name),
        ).pack(side='right')

    # ── Continuous publish ─────────────────────────────────────────────────────

    def _press(self, lx: float, az: float) -> None:
        self._cmd = (lx, az)
        self._tick()

    def _release(self) -> None:
        if self._after_id is not None:
            self.after_cancel(self._after_id)
            self._after_id = None
        self._node.stop(self._name)

    def _tick(self) -> None:
        spd = self._speed.get()
        lx, az = self._cmd
        self._node.send(self._name, lx * spd, az * spd * 1.5)
        self._after_id = self.after(REPEAT_MS, self._tick)

    # ── Telemetry refresh (called by main loop) ────────────────────────────────

    def refresh(self) -> None:
        t = self._node.telem.get(self._name)
        if t is not None:
            x, y, yaw_deg, vx, omega = t

            self._vars['x'] .set(f'{x:+8.3f}')
            self._vars['y'] .set(f'{y:+8.3f}')
            self._vars['θ'] .set(f'{yaw_deg:+8.1f}')
            self._vars['vx'].set(f'{vx:+8.3f}')
            self._vars['ω'] .set(f'{omega:+8.3f}')

            self._heading.set_yaw(yaw_deg)

            moving = abs(vx) > 0.01 or abs(omega) > 0.01
            self._status_dot.configure(
                text='● moving' if moving else '● stopped',
                fg='#2ecc71' if moving else '#95a5a6',
            )

        # Health indicator
        alive = self._node.health.get(self._name)
        if alive is None:
            self._health_label.configure(text='◉ HEALTH: unknown', fg='#7f8c8d')
        elif alive:
            self._health_label.configure(text='◉ HEALTH: alive', fg='#2ecc71')
        else:
            self._health_label.configure(text='◉ HEALTH: DEAD', fg='#e74c3c')

# ── Main window ────────────────────────────────────────────────────────────────

class SwarmGUI:

    def __init__(self, node: SwarmNode):
        self._node = node

        root = tk.Tk()
        root.title('Swarm Remote Control + Monitor')
        root.configure(bg='#0f3460')
        root.resizable(False, False)
        self.root = root

        # Title
        tk.Label(root, text='🤖  Swarm Control & Monitor  🤖',
                 bg='#0f3460', fg='white',
                 font=('Helvetica', 16, 'bold')).pack(pady=(14, 8))

        # Robot panels
        speed_var = tk.DoubleVar(value=0.4)
        panels_frame = tk.Frame(root, bg='#0f3460')
        panels_frame.pack(padx=12, pady=4)

        self._panels = []
        for i, robot in enumerate(ROBOTS):
            p = RobotPanel(panels_frame, robot, node, speed_var)
            p.grid(row=0, column=i, padx=7, sticky='n')
            self._panels.append(p)

        # Bottom bar
        bar = tk.Frame(root, bg='#0f3460')
        bar.pack(fill='x', padx=12, pady=(8, 14))

        tk.Label(bar, text='Speed', bg='#0f3460', fg='#bdc3c7',
                 font=('Helvetica', 11)).pack(side='left')

        ttk.Scale(bar, from_=0.05, to=1.0, variable=speed_var,
                  orient='horizontal', length=220).pack(side='left', padx=8)

        speed_lbl = tk.Label(bar, text='0.40', bg='#0f3460', fg='white',
                             font=('Courier', 11, 'bold'), width=4)
        speed_lbl.pack(side='left')
        speed_var.trace_add('write',
            lambda *_: speed_lbl.configure(text=f'{speed_var.get():.2f}'))

        tk.Button(bar, text='⛔  STOP ALL',
                  bg='#c0392b', fg='white',
                  font=('Helvetica', 12, 'bold'),
                  relief='flat', padx=10, pady=4, cursor='hand2',
                  command=node.stop_all).pack(side='right')

        root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._schedule_refresh()

    def _schedule_refresh(self) -> None:
        for p in self._panels:
            p.refresh()
        self.root.after(DISPLAY_MS, self._schedule_refresh)

    def _on_close(self) -> None:
        self._node.stop_all()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()

# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = SwarmNode()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    SwarmGUI(node).run()

    node.stop_all()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
