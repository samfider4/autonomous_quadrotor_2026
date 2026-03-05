import numpy as np
import math
import time
import logging
from threading import Thread

import motioncapture
import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

import tkinter as tk
from tkinter import ttk

# --- CONFIG ---
URI                     = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E7E9')
HOST_NAME               = '128.101.167.111'
MOCAP_SYSTEM_TYPE       = 'vicon'
DRONE_NAME              = '2026_Drone1'
BOX_NAME                = 'box'       # Vicon rigid-body name
BASESTATION_NAME_NAME   = 'basestation'       # Vicon rigid-body name
DODGE_DIST              = 0.2         # dodge distance in X/Y
DODGE_THRESH            = 0.2         # threshold to trigger dodge
CONTROL_RATE            = 0.05        # 20 Hz

logging.basicConfig(level=logging.ERROR)

class DroneController:
    def __init__(self, link_uri):
        self.control = True
        self.init_pos = None
        self.target = None
        self.target_yaw = None
        self.hover_height = 1.0
        self.dodge_enabled = True

        # connect mocap & Crazyflie
        self._mc = motioncapture.connect(MOCAP_SYSTEM_TYPE, {'hostname': HOST_NAME})
        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.open_link(link_uri)

        # PID state
        self.prev_time      = None
        self.prev_error     = np.zeros(3)
        self.prev_error_yaw  = 0
        self.integral_error = np.zeros(3)
        # PID gains
        self.Kp_xy, self.Kd_xy, self.Ki_xy = 1.2, 1.2, 0.0
        self.Kp_z,  self.Kd_z,  self.Ki_z  = 2.4, 0.8, 0.0
        self.Kp_yaw, self.Kd_yaw = 1.8, 1.8

    def _connected(self, link_uri):
        print(f"[{DRONE_NAME}] Connected")
        Thread(target=self._control_loop, daemon=True).start()

    def _connection_failed(self, link_uri, msg): print(f"Conn failed: {msg}")
    def _connection_lost(self, link_uri, msg):   print(f"Conn lost: {msg}")
    def _disconnected(self, link_uri):          print(f"[{DRONE_NAME}] Disconnected")

    def _get_position_data(self, name):
        while True:
            self._mc.waitForNextFrame()
            for nm, obj in self._mc.rigidBodies.items():
                if nm == name:
                    pos = np.array(obj.position)
                    if all(1e-9 < abs(p) < 1e4 for p in pos):
                        q = obj.rotation
                        yaw = math.atan2(
                            2*(q.w*q.z+q.x*q.y),
                            1-2*(q.y*q.y+q.z*q.z)
                        )
                        return pos, yaw, time.time()
            time.sleep(0.001)

    def pid_control(self, cur, tgt, yaw, tgt_yaw, dt):
        err = tgt - cur
        err_yaw = tgt_yaw - yaw
        for i, d in enumerate((0.05,0.05,0.05)):
            if abs(err[i]) < d: err[i] = 0.0
            if abs(err_yaw) < d: err_yaw = 0.0
        self.integral_error += err*dt

        derr_yaw = (err_yaw - self.prev_error_yaw)/dt if dt>0 else 0
        derr = (err - self.prev_error)/dt if dt>0 else np.zeros(3)
        self.prev_error = err.copy()
        self.prev_error_yaw = err_yaw

        cy, sy = math.cos(yaw), math.sin(yaw)
        lx =  cy*err[0] + sy*err[1]
        ly = -sy*err[0] + cy*err[1]

        pitch  = self.Kp_xy*lx   + self.Kd_xy*derr[0]
        roll   = -(self.Kp_xy*ly + self.Kd_xy*derr[1])
        thrust =  self.Kp_z*err[2] + self.Kd_z*derr[2]
        new_yaw = self.Kp_yaw*err_yaw + self.Kd_yaw*derr_yaw

        pd, rd, yd = np.rad2deg(pitch), np.rad2deg(roll), np.rad2deg(yaw)
        pd, rd = np.clip([pd, rd], -15, 15)
        cmd = int(np.clip(35000 + thrust*(50000/2.3346), 30000, 60000))
        return pd, rd, yd, cmd

    def _dodge_box(self, target):
        if not self.dodge_enabled:
            return target, None
        box_pos, yaw_tgt, _ = self._get_position_data(BOX_NAME)
        # target = box_pos.copy()
        # target[2] += 1.0
        diff = target[:2] - box_pos[:2]
        # if abs(diff[0])<DODGE_THRESH and abs(diff[1])<DODGE_THRESH:
        #     dodge = np.sign(diff)*DODGE_DIST
        #     return target + np.array([dodge[0], dodge[1], 0.0]), yaw_tgt
        return target, yaw_tgt

    # def do_rotation(self, target):
    #     #TODO

    # def _basestation_nav(self):
    #     station_pos, yaw, _ = self._get_position_data(BASESTATION_NAME)

    # GUI commands
    def takeoff_and_hover(self):
        if self.init_pos is None:
            print("Init not ready")
            return
        x0,y0,z0 = self.init_pos
        self.target = np.array([x0, y0, z0 + self.hover_height])
        print(f"Taking off to {self.target}")

    def move_relative(self, dx, dy):
        if self.target is None:
            print("Set target first")
            return
        self.target[0] += dx
        self.target[1] += dy
        print(f"Moved target to {self.target}")

    def go_home(self):
        if self.init_pos is None or self.target is None:
            print("Cannot go home")
            return
        self.target[0], self.target[1] = self.init_pos[0], self.init_pos[1]
        print(f"Home target {self.target}")

    def land(self):
        if self.init_pos is None:
            print("Init not ready")
            return
        self.target[2] = self.init_pos[2]
        print(f"Landing target {self.target}")

    def kill_motors(self):
        self.control = False
        self._cf.commander.send_setpoint(0,0,0,0)
        print("Motors killed")

    # Control loop
    def _control_loop(self):
        self._cf.commander.send_setpoint(0,0,0,0)
        pos, yaw, _ = self._get_position_data(DRONE_NAME)
        self.init_pos = pos.copy()
        print(f"Initial pos {self.init_pos}")

        while self.control and self.target is None:
            time.sleep(0.1)

        self.prev_time = time.time()
        while self.control:
            now = time.time(); dt = now - self.prev_time
            self.prev_time = now
            cur, yaw, _ = self._get_position_data(DRONE_NAME)
            safe_tgt, tgt_yaw = self._dodge_box(self.target.copy())
            if tgt_yaw == None: tgt_yaw = yaw
            pd, rd, yd, th = self.pid_control(cur, safe_tgt, yaw, tgt_yaw, dt)
            print(safe_tgt, tgt_yaw)
            print("pitch: ", pd, " roll: ", rd, " yaw: ", yd, " thrust: ", th)
            self._cf.commander.send_setpoint(rd, pd, 30, th)
            time.sleep(CONTROL_RATE)

        self._cf.commander.send_setpoint(0,0,0,0)
        self._cf.close_link()
        print("Control loop ended")

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    controller = DroneController(URI)

    # build GUI
    root = tk.Tk()
    root.title("Drone2 Control Panel")
    root.configure(bg='#E6F7FF')
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('Blue.TButton',
        background='#007ACC', foreground='white',
        font=('Segoe UI',12), padding=6)
    style.map('Blue.TButton',
        background=[('active','#005A9E')])
    style.configure('TLabel',
        background='#E6F7FF', foreground='#003366',
        font=('Segoe UI',12))
    style.configure('TEntry', font=('Segoe UI',12), width=6)
    style.configure('TCheckbutton',
        background='#E6F7FF', font=('Segoe UI',12),
        foreground='#003366')

    # hover height
    ttk.Label(root, text="Hover height (m):").grid(row=0,column=0,padx=5,pady=5,sticky='e')
    hover_var = tk.StringVar(value=str(controller.hover_height))
    hover_entry = ttk.Entry(root, textvariable=hover_var, justify='center')
    hover_entry.grid(row=0,column=1,padx=5,pady=5,sticky='w')
    def do_takeoff():
        try:
            controller.hover_height = float(hover_var.get())
        except ValueError:
            print("Invalid height")
            return
        controller.takeoff_and_hover()
    ttk.Button(root, text="Takeoff & Hover", style='Blue.TButton',
               command=do_takeoff).grid(row=1,column=0,columnspan=2, pady=5)

    # dodge toggle
    dodge_var = tk.BooleanVar(value=controller.dodge_enabled)
    ttk.Checkbutton(root, text="Enable dodge", style='TCheckbutton',
                    variable=dodge_var,
                    command=lambda: setattr(controller,'dodge_enabled',dodge_var.get())
                   ).grid(row=2,column=0,columnspan=2,pady=5)

    # step size slider
    ttk.Label(root, text="Step (m):").grid(row=3,column=0,pady=5,sticky='e')
    step_var = tk.DoubleVar(value=0.2)
    def update_step(v): pass  # we'll read step_var in move button binds
    step_slider = ttk.Scale(root, from_=0.05, to=0.5, orient='horizontal',
                            variable=step_var, command=update_step)
    step_slider.grid(row=3,column=1,pady=5,sticky='w')

    # movement buttons with press-and-hold auto-repeat
    move_job = None
    def start_move(dx,dy):
        controller.move_relative(dx,dy)
        global move_job
        move_job = root.after(200, lambda: start_move(dx,dy))
    def stop_move(evt=None):
        global move_job
        if move_job:
            root.after_cancel(move_job)
            move_job = None

    btn_up = ttk.Button(root, text="↑ Forward", style='Blue.TButton')
    btn_up.grid(row=4,column=1,pady=5)
    btn_up.bind('<ButtonPress-1>', lambda e: start_move(0, step_var.get()))
    btn_up.bind('<ButtonRelease-1>', stop_move)

    btn_left = ttk.Button(root, text="← Left", style='Blue.TButton')
    btn_left.grid(row=5,column=0,padx=5)
    btn_left.bind('<ButtonPress-1>', lambda e: start_move(-step_var.get(), 0))
    btn_left.bind('<ButtonRelease-1>', stop_move)

    btn_right = ttk.Button(root, text="→ Right", style='Blue.TButton')
    btn_right.grid(row=5,column=2,padx=5)
    btn_right.bind('<ButtonPress-1>', lambda e: start_move(step_var.get(), 0))
    btn_right.bind('<ButtonRelease-1>', stop_move)

    btn_down = ttk.Button(root, text="↓ Back", style='Blue.TButton')
    btn_down.grid(row=6,column=1,pady=5)
    btn_down.bind('<ButtonPress-1>', lambda e: start_move(0, -step_var.get()))
    btn_down.bind('<ButtonRelease-1>', stop_move)

    # home, land, kill
    ttk.Button(root, text="Go Home", style='Blue.TButton',
               command=controller.go_home).grid(row=7,column=0,pady=10)
    ttk.Button(root, text="Land (Return)", style='Blue.TButton',
               command=controller.land).grid(row=7,column=1,pady=10)
    ttk.Button(root, text="Kill Motors", style='Blue.TButton',
               command=controller.kill_motors).grid(row=7,column=2,pady=10)

    root.mainloop()