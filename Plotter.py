'''
IMPLEMENTATION INSTRUCTIONS:

Add the following code to the exisiting controller code:

in DroneController.__init__ add:

    #create plotter
    self.latest_plot_data = None


in DroneController._control_loop, while self.control
    after this line:
    pitch, roll, thrust = self.PID_control(x_pos, y_pos, z_pos, x_goal, y_goal, z_goal, Yaw, times)
    add:

    #send new data to plotter
    yaw_deg = np.rad2deg(Yaw)
    self.latest_plot_data = (pitch, roll, yaw_deg, thrust)


in main, after creating the controller, add:

    # Initialize plotter 
    from Plotter import SimplePlotter
    plotter = SimplePlotter(window_seconds=5)

    while True:
        #exit program if the window is closed and the drone is not active
        if plotter.closed and not controller.control:
            break

        # Only update while drone is active
        if controller.control and controller.latest_plot_data is not None:
            plotter.update(*controller.latest_plot_data)

        #update window title to indicate when the drone is not in self control
        if not controller.control:
            plotter.win.setWindowTitle("Drone Variables (STOPPED)")

        # only keep GUI if the window is still open 
        if not plotter.closed:
            plotter.app.processEvents()
    
        time.sleep(0.02)

'''

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import sys
import time


class SimplePlotter:
    def __init__(self, window_seconds=10):
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)

        self.win = pg.GraphicsLayoutWidget(title="Drone Variables")
        self.win.show()

        self.closed = False
        self.win.closeEvent = self._on_close

        self.window_seconds = window_seconds

        # Time + data buffers
        self.t = []
        self.pitch = []
        self.roll = []
        self.yaw = []
        self.thrust = []

        #  Pitch 
        self.pitch_plot = self.win.addPlot(title="Pitch")
        self.pitch_plot.showGrid(x=True, y=True)
        self.pitch_plot.setYRange(-15, 15)
        self.pitch_curve = self.pitch_plot.plot()
        self.win.nextRow()

        #  Roll 
        self.roll_plot = self.win.addPlot(title="Roll")
        self.roll_plot.showGrid(x=True, y=True)
        self.roll_plot.setYRange(-15, 15)
        self.roll_curve = self.roll_plot.plot()
        self.win.nextRow()

        #  Yaw 
        self.yaw_plot = self.win.addPlot(title="Yaw")
        self.yaw_plot.showGrid(x=True, y=True)
        self.yaw_plot.setYRange(-180, 180)
        self.yaw_curve = self.yaw_plot.plot()
        self.win.nextRow()

        #  Thrust 
        self.thrust_plot = self.win.addPlot(title="Thrust")
        self.thrust_plot.showGrid(x=True, y=True)
        self.thrust_plot.setYRange(0, 65000) 
        self.thrust_curve = self.thrust_plot.plot()

        self.start_time = time.time()

    def update(self, pitch, roll, yaw, thrust):
        current_time = time.time() - self.start_time

        # Store data
        self.t.append(current_time)
        self.pitch.append(pitch)
        self.roll.append(roll)
        self.yaw.append(yaw)
        self.thrust.append(thrust)

        # Keep only last N seconds
        while self.t and (current_time - self.t[0] > self.window_seconds):
            self.t.pop(0)
            self.pitch.pop(0)
            self.roll.pop(0)
            self.yaw.pop(0)
            self.thrust.pop(0)

        # Update plots
        self.pitch_curve.setData(self.t, self.pitch)
        self.roll_curve.setData(self.t, self.roll)
        self.yaw_curve.setData(self.t, self.yaw)
        self.thrust_curve.setData(self.t, self.thrust)

        self.app.processEvents()

    def _on_close(self, event):
        self.closed = True
        event.accept()