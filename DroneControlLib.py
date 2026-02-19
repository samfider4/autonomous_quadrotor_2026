from typing import Callable
import numpy as np
import numpy.typing as npt
import math
import time
import signal
from pynput import keyboard

import logging
import time
from threading import Thread
import motioncapture

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper


TimeSize = 5

# The host name or ip address of the mocap system
host_name = '128.101.167.111' # PLEASE UPDATE THIS IF NECESSARY

# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'vicon'

# The name of the rigid body that represents the Crazyflie (VICON object name)
drone_object_name = 'Drone1'
ground_object_name = 'Drone2'

logging.basicConfig(level=logging.ERROR)

class DroneCommanderStub:
    """A stub for a CrazyFlie commander."""
    def __init__(self, name):
        self.name = name

    def send_setpoint(self, roll, pitch, yawrate, thrust):
        pass

class DroneCallbackManagerStub:
    """A stub for callbacks in a Drone."""
    def __init__(self, name):
        self.name = name

    def add_callback(self, _):
        print(f"Adding callback to {self.name}")

class DroneStub:
    """A stub a Drone."""
    def __init__(self, name):
        self.name = name
        self.commander = DroneCommanderStub(name)
        self.connected = DroneCallbackManagerStub("connected")
        self.disconnected = DroneCallbackManagerStub("disconnected")
        self.connection_failed = DroneCallbackManagerStub("connection_failed")
        self.connection_lost = DroneCallbackManagerStub("connection_lost")
    
    def open_link(self, uri):
        print(f"Opening link to {uri}")

    def close_link(self):
        print(f"Closing link")

class ControlLoop:
    """A superclass with a template function for control loops."""
    def get(self, current_pos: npt.NDArray, target_pos: npt.NDArray, yaw: float, dt: float) -> tuple[float, float, int]:
        """Returns a roll angle, pitch angle, and thrust based off current position and target position.
        
        current_pos: A 3 element numpy array with x, y, and z components, representing the current position in cartesian form.
        target_pos: A 3 element numpy array with x, y, and z components, representing the target position in cartesian form.
        yaw: The current yaw angle in radians.
        dt: The amount of time in seconds since the last time this was called.

        Return value: A tuple of the roll angle in degrees, pitch angle in degrees, and thrust value, which is sent to the drone. 

        """
        pass

class PIDControlLoop(ControlLoop):
    """A PID control loop"""
    def __init__(self, name: str, Kp_xy: float, Kp_z: float, Ki_xy: float, Ki_z: float, Kd_xy: float, Kd_z: float):
        """Creates a PID control loop.
        FIXME: Integral term is ignored and may cause errors if implemented
        
        name: Controller name
        K_pxy: Lateral kP (proportional) constant
        K_pz: Vertical kP (proportional) constant
        K_ixy: Lateral kI (integral) constant
        K_iz: Vertical kI (integral) constant
        K_dxy: Lateral kD (derivative) constant
        K_dz: Vertical kD (derivative) constant
        """
        self.name = name
        self.Kp_xy = Kp_xy
        self.Kp_z = Kp_z
        self.Ki_xy = Ki_xy
        self.Ki_z = Ki_z
        self.Kd_xy = Kd_xy
        self.Kd_z = Kd_z
        self.integral_error = 0
        self.prev_error = 0

    def get(self, current_pos, target_pos, yaw, dt):
        error = target_pos - current_pos
        error[np.abs(error) < 0.05] = 0.0
        self.integral_error += error * dt
        d_error = (error - self.prev_error) / dt if dt > 0 else np.array([0.0, 0.0, 0.0])
        self.prev_error = error.copy()

        cy, sy = math.cos(yaw), math.sin(yaw)
        local_ctrl_x = cy * error[0] + sy * error[1]
        local_ctrl_y = -sy * error[0] + cy * error[1]

        pitch = self.Kp_xy * local_ctrl_x + self.Kd_xy * d_error[0]
        roll = - (self.Kp_xy * local_ctrl_y + self.Kd_xy * d_error[1])
        thrust = self.Kp_z * error[2] + self.Kd_z * d_error[2]

        pitch_deg = float(np.clip(np.rad2deg(pitch), -15, 15))
        roll_deg = float(np.clip(np.rad2deg(roll), -15, 15))
        thrust_cmd = 35000 + thrust * (50000 / 2.3346)
        thrust_cmd = int(np.clip(thrust_cmd, 30000, 60000))

        print(f"[{self.name}] Error: {error}, Pitch: {pitch_deg:.2f}, Roll: {roll_deg:.2f}, Thrust: {thrust_cmd}")
        return pitch_deg, roll_deg, thrust_cmd

class Drone:
    """An instance of a drone."""
    def __init__(self, link_uri: str, name: str, control_loop: ControlLoop, goal_func: Callable[[float, dict[str, tuple[float, float, float, float, float]]], None]):
        """Creates a new Drone.
        
        link_uri: The URI of the drone.
        Typically formatted as radio://<USB_INTERFACE>/<CHANNEL>/<BANDWIDTH>/<ADDRESS>.
        name: The name of the drone. This MUST match the drone's object name in the VICON motion capture system.
        control_loop: A ControlLoop which will be used to get roll, pitch, and thrust setpoints.
        goal_func: A function which takes float with the current run time, and a dictionary of drone positions.
        The dictionary is indexed by drone name and holds tuples (x, y, z, yaw, timestamp),
        where x, y, z are the position in cartesian form, yaw is the yaw angle in radians,
        and timestamp is timestamp when the latest position was recorded.
        """
        self.link_uri = link_uri
        self.name = name
        self.control_loop = control_loop
        self.goal_func = goal_func
        # goal_func(current_time, drone_positions): -> x_goal, y_goal, z_goal

class DroneController:
    """A DroneController which manages a set of drones and their motion capture objects.
    Example:
    
    # Create URIs
    drone1_uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    drone2_uri = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E7E8')

    # Create PID controllers
    drone1_pid = PIDControlLoop('Drone1', 1.5, 2.0, 0.0, 0.0, 1.0, 0.8)
    drone2_pid = PIDControlLoop('Drone1', 1.0, 2.4, 0.0, 0.0, 1.0, 0.8)

    # Create drone objects
    drone1 = Drone(drone1_uri, 'Drone1', drone1_pid, drone1_goal)
    drone2 = Drone(drone2_uri, 'Drone2', drone2_pid, drone2_goal)

    # Create controller and add drones
    controller = DroneController()
    controller.add_drone(drone1)
    controller.add_drone(drone2)

    # Connect drones and begin control.
    controller.connect_drones()
    """

    def __init__(self):
        self._mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        self.drones = {}
        self.latest_positions = {}
        self.control = False
        self._pull_position_data()
        self.position_thread = Thread(target=self._read_position_data, daemon=True)

    def add_drone(self, drone: Drone):
        """Adds a Drone to the controller"""
        cache_dir = "./%scache" % drone.name
        # drone._cf = DroneStub(drone.name)
        drone._cf = Crazyflie(rw_cache=cache_dir)

        drone._cf.connected.add_callback(self._connected)
        drone._cf.disconnected.add_callback(self._disconnected)
        drone._cf.connection_failed.add_callback(self._connection_failed)
        drone._cf.connection_lost.add_callback(self._connection_lost)

        self.drones[drone.link_uri] = drone

    def cleanup(self):
        self.control = False
        self.key_listener.stop()

    def sig_handler(self, sig, frame):
        self.cleanup()

    def on_press(self, key):
        if key == keyboard.Key.esc:
            self.cleanup()

    def connect_drones(self):
        """Connects all to added drones and begins flight. Control can be cancelled by pressing ESC or Ctrl-C."""
        self.control = True
        self.position_thread.start()
        self.key_listener = keyboard.Listener(on_press=self.on_press)
        self.key_listener.start()
        signal.signal(signal.SIGINT, self.sig_handler)
        signal.signal(signal.SIGTERM, self.sig_handler)

        for uri, drone in self.drones.items():
            drone._cf.open_link(uri)
            print('Connecting to %s' % uri)

        self._control_loop()

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        print('Connected to %s successfully' % (link_uri))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _pull_position_data(self):
        self._mc.waitForNextFrame()
        for name, obj in self._mc.rigidBodies.items():
            # Position data is sent as [x, y, z]
            pos = obj.position
            # Rotation data is sent as a quaternion object with w,x,y,z values
            q_w = obj.rotation.w
            q_x = obj.rotation.x
            q_y = obj.rotation.y
            q_z = obj.rotation.z

            # Sometimes Vicon sends data packets that have x, y, and z as either extremely large values or all zero (they're floats so something like 1e-30)
            # Discard that packet and wait for the next one
            # This is a temporary solution until a more reliable way to detect those bad packets is found
            # while ((np.abs(pos[0]) < 1.0e-9 or np.abs(pos[0]) > 1.0e4) or (np.abs(pos[1]) < 1.0e-9 or np.abs(pos[1]) > 1.0e4) or (np.abs(pos[2]) < 1.0e-9 or np.abs(pos[2]) > 1.0e4)):
            if np.abs(pos[0]) < 0.000000001 or np.abs(pos[0]) > 10000 or np.abs(
                    pos[1]) < 0.000000001 or np.abs(pos[1]) > 10000 or np.abs(
                    pos[2]) < 0.000000001 or np.abs(pos[2]) > 10000:
                continue

            yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y * q_y + q_z * q_z))
            timestamp = time.time()
            self.latest_positions[name] = (pos[0], pos[1], pos[2], yaw, timestamp)

    def _read_position_data(self):
        """Run in a separate thread, and pulls motion capture data from the VICON server"""
        while self.control:
            self._pull_position_data()

    def get_position_data(self, name):
        """Gets known position data for a given drone. Returns all zeroes if unknown."""
        if name in self.latest_positions:
            return self.latest_positions[name]
        else:
            return (0, 0, 0, 0, 0)

    def _control_loop(self):
        """ The main loop of the controller code """
        # These goals are in standard unts for mocap (should be in meters)
        zero_time = time.time()
        prev_time = 0

        # Initial goal states
        x_goal = 0
        y_goal = 0
        z_goal = 0

        for drone in self.drones.values():
            # Unlock startup thrust protection
            drone._cf.commander.send_setpoint(0, 0, 0, 0)

        while self.control:
            run_time = time.time() - zero_time
            dt = run_time - prev_time
            prev_time = run_time
            for drone in self.drones.values():
                x_goal, y_goal, z_goal = drone.goal_func(run_time, self)

                X_new, Y_new, Z_new, Yaw, new_time = self.get_position_data(drone.name)
                # print(self.get_position_data(drone.name))

                pitch, roll, thrust = drone.control_loop.get(np.array((X_new, Y_new, Z_new)), np.array((x_goal, y_goal, z_goal)), Yaw, dt)

                # print(f"Setting (name: {drone.name}) roll {roll}, pitch {pitch}, 0, thrust {thrust}")
                drone._cf.commander.send_setpoint(roll, pitch, 0, thrust)

        # This is where we would put any code to be run before the program ends
        print("\nKill button pressed, shutting down")
        for drone in self.drones.values():
            drone._cf.commander.send_setpoint(0, 0, 0, 0)
            drone._cf.close_link()


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    drone1_uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7') # radio://USB_ID/CHANNEL/DATA_RATE/ADDRESS
    drone2_uri = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E7E8')
    drone1_pid = PIDControlLoop('Drone1', 0.5, 2.4, 0.0, 0.0, 0.5, 0.8)
    drone2_pid = PIDControlLoop('Drone2', 0.5, 2.4, 0.0, 0.0, 0.5, 0.8)
    drone1 = Drone(drone1_uri, 'Drone1', drone1_pid, drone2_goal)
    # drone2 = Drone(drone2_uri, 'Drone2', drone2_pid, drone2_goal)
    controller = DroneController()
    controller.add_drone(drone1)
    # controller.add_drone(drone2)
    controller.connect_drones()
