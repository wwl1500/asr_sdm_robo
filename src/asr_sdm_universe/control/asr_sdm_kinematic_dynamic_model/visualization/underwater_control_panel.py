#!/usr/bin/env python3
"""Qt controls for the live underwater simulator parameters.

Sections:
  Hydrodynamics  - fluid density, damping, displaced volume per link
  Fluid flow     - current + current acceleration (3 axes)
  Thrust inputs  - per-segment thrust OR per-rotor rate (4 vs 8 controls)
  Joint torques  - 6-DOF joint torque vector
  Presets        - one-click nominal / no-fluid / zero-input / step-on-joint-1

The panel talks to ``/underwater_simulator_node`` via ``AsyncParameterClient``,
so every change is pushed to the running C++ node at ~20 Hz.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys

from python_qt_binding.QtCore import QSocketNotifier, Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


# Defaults / ranges for the joint torque row (Nm).
# Match URDF SERVO_EFFORT = 4.0 Nm, and keep range small to avoid hitting joint limits.
JOINT_TORQUE_RANGE = 1.0
# Defaults / ranges for rotor rate (rad/s); kNumRotors = 8 in the C++ node.
ROTOR_RATE_RANGE = 100.0
# Defaults / ranges for fluid current acceleration (m/s^2).
CURRENT_ACCEL_RANGE = 2.0


class ControlPanel(QMainWindow):
    def __init__(self, node_name: str):
        super().__init__()
        self.setWindowTitle('ASR-SDM underwater simulator control panel')
        self.resize(960, 760)
        self.node = rclpy.create_node('underwater_fluid_control_panel')
        self.client = AsyncParameterClient(self.node, node_name)
        self.controls = {}
        self.pending = {}
        self.status = QLabel('Waiting for simulator parameter service...')

        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)

        # --- Hydrodynamics -------------------------------------------------
        hydro_box = QGroupBox('Hydrodynamics')
        hydro_grid = QGridLayout(hydro_box)
        row = 0
        row = self.add_control(hydro_grid, row, 'fluid_density', 'Density (kg/m³)', 0.0, 1500.0, 1000.0)
        row = self.add_control(hydro_grid, row, 'linear_damping', 'Linear damping', 0.0, 5.0, 0.1)
        row = self.add_control(hydro_grid, row, 'quadratic_damping', 'Quadratic damping', 0.0, 2.0, 0.01)
        for link in range(4):
            row = self.add_control(
                hydro_grid, row, f'displaced_volume_{link}',
                f'Volume {link} (m³)', 0.0, 0.001, 5.81e-4,
            )
        outer.addWidget(hydro_box)

        # --- Fluid flow ---------------------------------------------------
        flow_box = QGroupBox('Fluid flow (world frame)')
        flow_grid = QGridLayout(flow_box)
        row = 0
        for axis in ('x', 'y', 'z'):
            row = self.add_control(
                flow_grid, row, f'fluid_current_{axis}', f'Current {axis} (m/s)', -2.0, 2.0, 0.0)
        for axis in ('x', 'y', 'z'):
            row = self.add_control(
                flow_grid, row, f'fluid_current_acceleration_{axis}',
                f'Accel {axis} (m/s²)', -CURRENT_ACCEL_RANGE, CURRENT_ACCEL_RANGE, 0.0)
        outer.addWidget(flow_box)

        # --- Thrust inputs ------------------------------------------------
        thrust_box = QGroupBox('Thrust inputs (segment thrust + rotor rate)')
        thrust_grid = QGridLayout(thrust_box)
        row = 0
        for link in range(4):
            row = self.add_control(
                thrust_grid, row, f'segment_thrust_{link}',
                f'Thrust {link} (N)', 0.0, 1.0, 0.0,
            )
        for rotor in range(8):
            row = self.add_control(
                thrust_grid, row, f'rotor_rate_{rotor}',
                f'Rotor {rotor} (rad/s)', -ROTOR_RATE_RANGE, ROTOR_RATE_RANGE, 0.0,
            )
        outer.addWidget(thrust_box)

        # --- Joint torques ------------------------------------------------
        torque_box = QGroupBox('Joint torques (6-DOF)')
        torque_grid = QGridLayout(torque_box)
        for joint in range(6):
            self.add_control(
                torque_grid, joint, f'joint_torque_{joint}',
                f'Joint {joint} (N·m)', -JOINT_TORQUE_RANGE, JOINT_TORQUE_RANGE, 0.0,
            )
        outer.addWidget(torque_box)

        # --- Bottom action bar: presets + buttons -------------------------
        actions = QHBoxLayout()
        self.preset_combo = QComboBox()
        self.preset_combo.addItems([
            '— select preset —',
            'Nominal (water, no thrust)',
            'No fluid (air-like)',
            'Zero all inputs',
            'Joint 1 step (+5 N·m)',
            'All rotors slow forward (5 rad/s)',
            'All rotors full forward',
            'Apply current surge (+X 1 m/s)',
        ])
        self.preset_combo.activated.connect(self.apply_preset)
        actions.addWidget(QLabel('Preset:'))
        actions.addWidget(self.preset_combo, 1)

        for text, callback in (
            ('Apply now', self.apply_pending),
            ('Nominal', self.nominal),
            ('No fluid', self.no_fluid),
            ('Zero inputs', self.zero_inputs),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            actions.addWidget(button)
        outer.addLayout(actions)

        outer.addWidget(self.status)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50)
        # Set by the SIGTERM/SIGINT signal handler in main(). spin_ros() polls
        # this on every tick and performs the clean shutdown sequence from
        # inside the Qt event loop (no blocking work in the handler itself).
        self._shutdown_requested = False

    def add_control(self, grid, row, name, label, minimum, maximum, initial):
        grid.addWidget(QLabel(label), row, 0)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 1000)
        slider.setValue(self.to_slider(minimum, maximum, initial))
        value_label = QLabel()
        value_label.setMinimumWidth(80)
        self.controls[name] = (slider, value_label, minimum, maximum)
        slider.valueChanged.connect(lambda value, key=name: self.slider_changed(key, value))
        grid.addWidget(slider, row, 1)
        grid.addWidget(value_label, row, 2)
        self.slider_changed(name, slider.value())
        return row + 1

    @staticmethod
    def to_slider(minimum, maximum, value):
        return round(1000.0 * (value - minimum) / (maximum - minimum))

    @staticmethod
    def from_slider(minimum, maximum, value):
        return minimum + (maximum - minimum) * value / 1000.0

    def slider_changed(self, name, slider_value):
        _, value_label, minimum, maximum = self.controls[name]
        value = self.from_slider(minimum, maximum, slider_value)
        value_label.setText(f'{value:.6g}')
        self.pending[name] = value

    def set_control(self, name, value):
        slider, _, minimum, maximum = self.controls[name]
        target = self.to_slider(minimum, maximum, value)
        # setValue() only emits valueChanged when the value actually changes,
        # so calls from presets (Nominal, No fluid, ...) that re-set a slider
        # to its existing position never reach slider_changed. Block signals,
        # then explicitly push the value so pending gets populated.
        slider.blockSignals(True)
        slider.setValue(target)
        slider.blockSignals(False)
        self.pending[name] = value

    def nominal(self):
        self.set_control('fluid_density', 1000.0)
        self.set_control('linear_damping', 0.1)
        self.set_control('quadratic_damping', 0.01)
        for axis in ('x', 'y', 'z'):
            self.set_control(f'fluid_current_{axis}', 0.0)
            self.set_control(f'fluid_current_acceleration_{axis}', 0.0)
        for link in range(4):
            self.set_control(f'displaced_volume_{link}', 5.81e-4)
            self.set_control(f'segment_thrust_{link}', 0.0)
        for rotor in range(8):
            self.set_control(f'rotor_rate_{rotor}', 0.0)
        for joint in range(6):
            self.set_control(f'joint_torque_{joint}', 0.0)
        self.apply_pending()

    def no_fluid(self):
        self.set_control('fluid_density', 0.0)
        self.set_control('linear_damping', 0.0)
        self.set_control('quadratic_damping', 0.0)
        for axis in ('x', 'y', 'z'):
            self.set_control(f'fluid_current_{axis}', 0.0)
            self.set_control(f'fluid_current_acceleration_{axis}', 0.0)
        for link in range(4):
            self.set_control(f'displaced_volume_{link}', 0.0)
            self.set_control(f'segment_thrust_{link}', 0.0)
        for rotor in range(8):
            self.set_control(f'rotor_rate_{rotor}', 0.0)
        for joint in range(6):
            self.set_control(f'joint_torque_{joint}', 0.0)
        self.apply_pending()

    def zero_inputs(self):
        """Reset every control input (current, thrust, rotor rate, joint torque)
        but leave hydrodynamics (density, damping, volume) alone."""
        for axis in ('x', 'y', 'z'):
            self.set_control(f'fluid_current_{axis}', 0.0)
            self.set_control(f'fluid_current_acceleration_{axis}', 0.0)
        for link in range(4):
            self.set_control(f'segment_thrust_{link}', 0.0)
        for rotor in range(8):
            self.set_control(f'rotor_rate_{rotor}', 0.0)
        for joint in range(6):
            self.set_control(f'joint_torque_{joint}', 0.0)
        self.apply_pending()

    def apply_preset(self, index):
        """Combo-box handler. Index 0 is the placeholder."""
        if index == 0:
            return
        presets = {
            1: self.nominal,
            2: self.no_fluid,
            3: self.zero_inputs,
            4: lambda: self._step_joint(0, 5.0),
            5: lambda: self._all_rotors(5.0),
            6: lambda: self._all_rotors(ROTOR_RATE_RANGE),
            7: lambda: (self.set_control('fluid_current_x', 1.0), self.apply_pending()),
        }
        action = presets.get(index)
        if action is not None:
            action()
        # Reset combo to placeholder so the same preset can be re-applied.
        self.preset_combo.setCurrentIndex(0)

    def _step_joint(self, joint_index, value):
        for joint in range(6):
            self.set_control(
                f'joint_torque_{joint}',
                value if joint == joint_index else 0.0,
            )
        self.apply_pending()

    def _all_rotors(self, value):
        for rotor in range(8):
            self.set_control(f'rotor_rate_{rotor}', value)
        self.apply_pending()

    def make_parameter(self, name, value):
        return Parameter(name, Parameter.Type.DOUBLE, float(value))

    def _read_vector(self, prefix, count, slider_min, slider_max):
        """Read ``count`` sliders named ``prefix_0`` .. ``prefix_{count-1}``
        and return their decoded numeric values as a list."""
        return [
            self.from_slider(
                slider_min, slider_max,
                self.controls[f'{prefix}_{i}'][0].value(),
            )
            for i in range(count)
        ]

    def apply_pending(self):
        if not self.pending:
            return

        # Check services BEFORE clearing pending so retries on the next spin work.
        if not self.client.services_are_ready():
            self.status.setText('Waiting for simulator parameter service...')
            return

        # Snapshot and clear atomically so new slider moves go into a fresh batch.
        pending = dict(self.pending)
        self.pending.clear()
        parameters = []

        # Scalars - pushed individually.
        vector_prefixes = (
            'displaced_volume', 'segment_thrust', 'rotor_rate', 'joint_torque',
        )
        for name, value in pending.items():
            if any(name.startswith(prefix + '_') for prefix in vector_prefixes):
                continue
            parameters.append(self.make_parameter(name, value))

        # Vectors - gathered and sent as DOUBLE_ARRAY. Only push if at least
        # one component changed in this batch.
        def push_array(prefix, count, slider_min, slider_max):
            for i in range(count):
                if f'{prefix}_{i}' in pending:
                    return True
            return False

        if push_array('displaced_volume', 4, 0.0, 0.001):
            parameters.append(Parameter(
                'displaced_volume', Parameter.Type.DOUBLE_ARRAY,
                self._read_vector('displaced_volume', 4, 0.0, 0.001)))
        if push_array('segment_thrust', 4, 0.0, 1.0):
            parameters.append(Parameter(
                'segment_thrust', Parameter.Type.DOUBLE_ARRAY,
                self._read_vector('segment_thrust', 4, 0.0, 1.0)))
        if push_array('rotor_rate', 8, -ROTOR_RATE_RANGE, ROTOR_RATE_RANGE):
            parameters.append(Parameter(
                'rotor_rate', Parameter.Type.DOUBLE_ARRAY,
                self._read_vector('rotor_rate', 8, -ROTOR_RATE_RANGE, ROTOR_RATE_RANGE)))
        if push_array('joint_torque', 6, -JOINT_TORQUE_RANGE, JOINT_TORQUE_RANGE):
            parameters.append(Parameter(
                'joint_torque', Parameter.Type.DOUBLE_ARRAY,
                self._read_vector('joint_torque', 6, -JOINT_TORQUE_RANGE, JOINT_TORQUE_RANGE)))

        future = self.client.set_parameters(parameters)
        future.add_done_callback(self.parameter_result)

    def parameter_result(self, future):
        try:
            response = future.result()
            results = response.results
            if all(result.successful for result in results):
                self.status.setText('Parameters applied')
            else:
                reasons = '; '.join(result.reason for result in results if not result.successful)
                self.status.setText(f'Rejected: {reasons}')
        except Exception as error:  # pragma: no cover - depends on ROS service state
            self.status.setText(f'Parameter update failed: {error}')

    def spin_ros(self):
        if not rclpy.ok():
            # ROS context was already torn down (e.g. launch sent SIGTERM and
            # rclpy.shutdown() ran). Don't touch the wait set, just bail out.
            if self.timer.isActive():
                self.timer.stop()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except rclpy._rclpy_pybind11.RCLError:
            # Race: shutdown() raced with this timer tick. Stop the timer and
            # let the Qt loop finish closing the window cleanly.
            if self.timer.isActive():
                self.timer.stop()
            return
        if self.client.services_are_ready():
            self.status.setText('Connected to simulator')
            # Flush any params that were queued before the service came up.
            self.apply_pending()

        # Signal-driven shutdown path. We only flip flags here - no blocking
        # calls, no direct rclpy.shutdown(). closeEvent() does the actual
        # teardown when Qt processes the close request below.
        if self._shutdown_requested:
            self._shutdown_requested = False
            if self.timer.isActive():
                self.timer.stop()
            # Closing the window triggers closeEvent, which destroys the node
            # and shuts ROS down. We also ask the Qt event loop to exit so the
            # process actually terminates; otherwise exec_() would just keep
            # spinning on a hidden window. aboutToQuit will invoke close() a
            # second time, which is idempotent because of the timer check.
            try:
                self.close()
            except Exception:
                pass
            qt_app = QApplication.instance()
            if qt_app is not None:
                qt_app.quit()

    def closeEvent(self, event):
        if self.timer.isActive():
            self.timer.stop()
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    # Strip ROS 2 arguments that launch injects automatically
    # (--ros-args ... --, --node, -r __ns:=, etc.)
    stripped = []
    skip = False
    for arg in sys.argv:
        if arg in ('--ros-args', '-r', '--'):
            skip = True
            continue
        if skip:
            # -r takes a remapping arg; skip that too
            if arg.startswith('__'):
                continue
            skip = False
        if arg.startswith('--node') or arg.startswith('-r __'):
            continue
        stripped.append(arg)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', default='/underwater_simulator_node')
    parsed, _ = parser.parse_known_args(stripped)
    rclpy.init(args=None)
    app = QApplication(['underwater_control_panel'])

    # Build the panel BEFORE wiring up shutdown plumbing so the notifier
    # handler can safely call panel.close().
    panel = ControlPanel(parsed.node)
    panel.show()

    # Python signal handlers installed via signal.signal() are only invoked
    # when the interpreter is between two bytecode instructions. When the
    # main thread is parked inside Qt's C-level app.exec_(), the handler
    # never runs and SIGTERM falls back to the default action (terminate
    # with exit code 143 - which is exactly what we observed). The fix is
    # to route the signal through a self-pipe via signal.set_wakeup_fd()
    # and have Qt observe the pipe's read end with QSocketNotifier. That
    # way the "wake up" callback runs inside the Qt event loop on every
    # signal arrival, regardless of what C code the interpreter is in.
    wake_r, wake_w = os.pipe()
    os.set_blocking(wake_r, False)
    # signal.set_wakeup_fd requires both ends to be non-blocking. Without
    # this, Python raises ValueError at the set_wakeup_fd() call below.
    os.set_blocking(wake_w, False)
    signal.set_wakeup_fd(wake_w)

    def _on_signal(_notifier=None):
        # Drain the self-pipe so the next signal can fire the notifier again.
        try:
            os.read(wake_r, 1024)
        except BlockingIOError:
            pass
        except Exception:
            pass
        # Delegate teardown to spin_ros via the flag path - that way the
        # ROS context is shut down inside the Qt event loop, not here.
        panel._shutdown_requested = True
        # Disable the notifier so we don't re-fire while the window closes.
        notifier.setEnabled(False)

    notifier = QSocketNotifier(wake_r, QSocketNotifier.Read, panel)
    notifier.activated.connect(_on_signal)

    # Restore the default wakeup behaviour on shutdown so a follow-up
    # signal during interpreter teardown doesn't try to write to a
    # closed pipe and raise an OSError.
    def _release_wakeup_fd():
        try:
            signal.set_wakeup_fd(-1)
        except Exception:
            pass
        try:
            os.close(wake_r)
        except Exception:
            pass
        try:
            os.close(wake_w)
        except Exception:
            pass

    signal.signal(signal.SIGINT, lambda signum, _frame: None)
    signal.signal(signal.SIGTERM, lambda signum, _frame: None)
    app.aboutToQuit.connect(_release_wakeup_fd)
    app.aboutToQuit.connect(lambda: (panel.close(), None))

    app.exec_()


if __name__ == '__main__':
    main()
