#!/usr/bin/env python3
"""Qt controls for the live underwater simulator parameters."""

from __future__ import annotations

import argparse
import signal

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QGridLayout,
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


class ControlPanel(QMainWindow):
    def __init__(self, node_name: str):
        super().__init__()
        self.setWindowTitle('ASR-SDM underwater fluid controls')
        self.node = rclpy.create_node('underwater_fluid_control_panel')
        self.client = AsyncParameterClient(self.node, node_name)
        self.controls = {}
        self.pending = {}
        self.status = QLabel('Waiting for simulator parameter service...')

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        grid = QGridLayout()
        layout.addLayout(grid)

        row = 0
        row = self.add_control(grid, row, 'fluid_density', 'Density (kg/m³)', 0.0, 1500.0, 1000.0)
        row = self.add_control(grid, row, 'linear_damping', 'Linear damping', 0.0, 5.0, 0.0)
        row = self.add_control(grid, row, 'quadratic_damping', 'Quadratic damping', 0.0, 2.0, 0.0)
        for axis in ('x', 'y', 'z'):
            row = self.add_control(
                grid, row, f'fluid_current_{axis}', f'Current {axis} (m/s)', -2.0, 2.0, 0.0
            )
        for link in range(4):
            row = self.add_control(
                grid, row, f'displaced_volume_{link}',
                f'Volume {link} (m³)', 0.0, 0.001, 0.0,
            )
        for link in range(4):
            row = self.add_control(
                grid, row, f'segment_thrust_{link}',
                f'Thrust {link} (N)', 0.0, 1.0, 0.0,
            )

        buttons = QHBoxLayout()
        for text, callback in (
            ('Nominal', self.nominal),
            ('No fluid', self.no_fluid),
            ('Apply now', self.apply_pending),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        layout.addLayout(buttons)
        layout.addWidget(self.status)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50)

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
        slider.setValue(self.to_slider(minimum, maximum, value))

    def nominal(self):
        self.set_control('fluid_density', 1000.0)
        self.set_control('linear_damping', 0.1)
        self.set_control('quadratic_damping', 0.01)
        for axis in ('x', 'y', 'z'):
            self.set_control(f'fluid_current_{axis}', 0.0)
        for link in range(4):
            self.set_control(f'displaced_volume_{link}', 5.81e-4)
            self.set_control(f'segment_thrust_{link}', 0.0)
        self.apply_pending()

    def no_fluid(self):
        self.set_control('fluid_density', 0.0)
        self.set_control('linear_damping', 0.0)
        self.set_control('quadratic_damping', 0.0)
        for axis in ('x', 'y', 'z'):
            self.set_control(f'fluid_current_{axis}', 0.0)
        for link in range(4):
            self.set_control(f'displaced_volume_{link}', 0.0)
            self.set_control(f'segment_thrust_{link}', 0.0)
        self.apply_pending()

    def make_parameter(self, name, value):
        return Parameter(name, Parameter.Type.DOUBLE, float(value))

    def apply_pending(self):
        if not self.pending:
            return
        pending = dict(self.pending)
        self.pending.clear()
        parameters = []
        for name, value in pending.items():
            if name.startswith('displaced_volume_') or name.startswith('segment_thrust_'):
                continue
            parameters.append(self.make_parameter(name, value))

        volumes = [self.controls[f'displaced_volume_{i}'][0].value() for i in range(4)]
        thrust = [self.controls[f'segment_thrust_{i}'][0].value() for i in range(4)]
        parameters.append(Parameter('displaced_volume', Parameter.Type.DOUBLE_ARRAY,
                                    [self.from_slider(0.0, 0.001, value) for value in volumes]))
        parameters.append(Parameter('segment_thrust', Parameter.Type.DOUBLE_ARRAY,
                                    [self.from_slider(0.0, 1.0, value) for value in thrust]))
        if not self.client.services_are_ready():
            self.status.setText('Simulator parameter service is not available')
            self.pending.update(pending)
            return
        future = self.client.set_parameters(parameters)
        future.add_done_callback(self.parameter_result)

    def parameter_result(self, future):
        try:
            results = future.result()
            if all(result.successful for result in results):
                self.status.setText('Parameters applied')
            else:
                reasons = '; '.join(result.reason for result in results if not result.successful)
                self.status.setText(f'Rejected: {reasons}')
        except Exception as error:  # pragma: no cover - depends on ROS service state
            self.status.setText(f'Parameter update failed: {error}')

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.client.services_are_ready():
            self.status.setText('Connected to simulator')

    def closeEvent(self, event):
        self.timer.stop()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', default='/underwater_simulator_node')
    args = parser.parse_args()
    rclpy.init(args=None)
    app = QApplication(['underwater_control_panel'])
    panel = ControlPanel(args.node)
    panel.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app.exec_()


if __name__ == '__main__':
    main()
