#!/usr/bin/env python3
"""
UR10e Control Center GUI Node
ROS2 + PyQt5 based control interface with real-time force sensor visualization
"""

import sys
import subprocess
import signal
import os
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QSizePolicy, QMessageBox, QGroupBox,
    QListWidget, QListWidgetItem, QGridLayout, QSpacerItem
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject, QSize
from PyQt5.QtGui import QFont, QPalette, QColor, QIcon

import pyqtgraph as pg
import numpy as np


# Get the directory where this script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ICONS_DIR = os.path.join(SCRIPT_DIR, 'icons')


def get_icon(name):
    """Get an icon from the icons directory"""
    icon_path = os.path.join(ICONS_DIR, f'{name}.svg')
    if os.path.exists(icon_path):
        return QIcon(icon_path)
    return QIcon()




class ProcessManager:
    """Manages subprocess launches without blocking the GUI"""
    
    def __init__(self):
        self.processes = {}
        
    def run_command(self, name, command, shell=True):
        """
        Run a command as a non-blocking subprocess
        
        Args:
            name: Identifier for the process
            command: Command string or list to execute
            shell: Whether to use shell execution (default True)
        
        Returns:
            bool: True if successfully started, False otherwise
        """
        # Kill existing process with same name if running
        if name in self.processes and self.processes[name].poll() is None:
            print(f"⚠ Process '{name}' already running. Terminating existing instance...")
            self.processes[name].terminate()
            try:
                self.processes[name].wait(timeout=3)
                print(f"  ✓ Existing '{name}' terminated")
            except subprocess.TimeoutExpired:
                print(f"  ! Existing '{name}' not responding, force killing...")
                self.processes[name].kill()
                self.processes[name].wait()
                print(f"  ✓ Existing '{name}' force-killed")
        
        try:
            # Start process with separate process group for clean termination
            # Use DEVNULL for stdout/stderr to prevent pipe buffer blocking
            # (ROS2 nodes produce continuous log output that can fill up pipe buffers)
            process = subprocess.Popen(
                command,
                shell=shell,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
            )
            self.processes[name] = process
            print(f"✓ Started '{name}' [PID: {process.pid}]")
            print(f"  Command: {command}")
            return True
        except Exception as e:
            print(f"✗ Failed to start '{name}': {e}")
            return False
    
    def is_running(self, name):
        """Check if a process is currently running"""
        if name in self.processes:
            return self.processes[name].poll() is None
        return False
    
    def get_running_processes(self):
        """Get list of currently running process names"""
        return [name for name, proc in self.processes.items() if proc.poll() is None]
    
    def terminate_all(self):
        """Terminate all managed processes"""
        running_count = 0
        for name, process in self.processes.items():
            if process.poll() is None:
                running_count += 1
                print(f"⏹ Terminating '{name}' [PID: {process.pid}]...")
                process.terminate()
                try:
                    process.wait(timeout=2)
                    print(f"  ✓ '{name}' stopped gracefully")
                except subprocess.TimeoutExpired:
                    print(f"  ! '{name}' not responding, force killing...")
                    process.kill()
                    process.wait()
                    print(f"  ✓ '{name}' force-killed")
        
        if running_count == 0:
            print("ℹ No processes were running")
        else:
            print(f"✓ Terminated {running_count} process(es)")
    
    def terminate_process(self, name):
        """Terminate a specific process by name"""
        if name in self.processes:
            process = self.processes[name]
            if process.poll() is None:
                print(f"⏹ Terminating '{name}' [PID: {process.pid}]...")
                process.terminate()
                try:
                    process.wait(timeout=2)
                    print(f"  ✓ '{name}' stopped gracefully")
                    return True
                except subprocess.TimeoutExpired:
                    print(f"  ! '{name}' not responding, force killing...")
                    process.kill()
                    process.wait()
                    print(f"  ✓ '{name}' force-killed")
                    return True
            else:
                print(f"ℹ '{name}' is not running")
                return False
        else:
            print(f"ℹ '{name}' not found in process list")
            return False


class ROS2WorkerSignals(QObject):
    """Signals for thread-safe communication between ROS2 and Qt"""
    force_data = pyqtSignal(float, float, float)  # X, Y, Z force values


class UR10eControlGUI(QMainWindow):
    """Main GUI Window for UR10e Robot Control"""
    
    def __init__(self, ros_node):
        super().__init__()
        
        self.ros_node = ros_node
        self.process_manager = ProcessManager()
        
        # Data storage for plotting (keep last 500 points)
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.force_x_data = deque(maxlen=self.max_points)
        self.force_y_data = deque(maxlen=self.max_points)
        self.force_z_data = deque(maxlen=self.max_points)
        self.time_counter = 0
        
        self.init_ui()
        self.setup_plot()
        
        # Connect ROS2 signals to Qt slots
        self.ros_node.signals.force_data.connect(self.update_force_data)
        
        # Timer for updating the plot
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(50)  # Update every 50ms (20 Hz)
        
        # Timer for updating process status
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status_display)
        self.status_timer.start(1000)  # Update every 1 second

        
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("UR10e Control Center GUI")
        self.setGeometry(100, 100, 1200, 700)
        
        # Set modern color scheme
        self.set_style()
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(15, 15, 15, 15)
        
        # ========== LEFT PANEL: Control Buttons ==========
        left_panel = self.create_control_panel()
        left_panel.setFixedWidth(350)
        main_layout.addWidget(left_panel)
        
        # ========== RIGHT PANEL: Visualization ==========
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(15)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        # Top: Force Sensor Plot
        plot_group = self.create_plot_panel()
        right_layout.addWidget(plot_group, stretch=1)
        
        main_layout.addWidget(right_panel, stretch=1)
        
    def set_style(self):
        """Set modern GUI styling"""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(40, 40, 42))
        palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
        palette.setColor(QPalette.Base, QColor(50, 50, 52))
        palette.setColor(QPalette.AlternateBase, QColor(45, 45, 48))
        palette.setColor(QPalette.Text, QColor(220, 220, 220))
        palette.setColor(QPalette.Button, QColor(55, 55, 58))
        palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
        self.setPalette(palette)
        
        # Global stylesheet
        self.setStyleSheet("""
            QMainWindow {
                background-color: #282830;
            }
            QPushButton {
                background-color: #3a3a3f;
                border: 1px solid #4a4a4f;
                border-radius: 6px;
                padding: 10px 15px;
                font-size: 15px;
                color: #dcdcdc;
            }
            QPushButton:hover {
                background-color: #454550;
                border: 1px solid #5a5a6f;
            }
            QPushButton:pressed {
                background-color: #2a2a2f;
            }
            QLabel {
                color: #dcdcdc;
            }
            QListWidget {
                background-color: #32323a;
                border: 1px solid #4a4a4f;
                border-radius: 6px;
                padding: 5px;
            }
            QListWidget::item {
                padding: 5px;
                border-radius: 3px;
            }
            QListWidget::item:hover {
                background-color: #3a3a42;
            }
        """)
    
    def create_control_panel(self):
        """Create the left control panel with action buttons"""
        panel = QWidget()
        panel.setStyleSheet("""
            QWidget {
                background-color: #32323a;
                border-radius: 10px;
            }
        """)
        layout = QVBoxLayout(panel)
        layout.setSpacing(15)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # ========== Controller Section ==========
        controller_label = QLabel("Controller")
        controller_label.setFont(QFont("Arial", 15, QFont.Bold))
        controller_label.setStyleSheet("background-color: transparent;")
        layout.addWidget(controller_label)
        
        # Cartesian Compliance Controller button
        self.cartesian_btn = QPushButton("Activate Cartesian Compliance")
        self.cartesian_btn.clicked.connect(self.switch_to_cartesian)
        self.cartesian_btn.setStyleSheet("""
            QPushButton {
                background-color: #3a3a3f;
                border: 1px solid #4a4a4f;
                border-radius: 6px;
                padding: 12px;
                font-size: 15px;
            }
            QPushButton:hover {
                background-color: #454550;
            }
        """)
        layout.addWidget(self.cartesian_btn)
        
        # Joint Trajectory Controller button
        self.joint_btn = QPushButton("Activate Joint Trajectory")
        self.joint_btn.clicked.connect(self.switch_to_joint)
        self.joint_btn.setStyleSheet("""
            QPushButton {
                background-color: #2563eb;
                border: 1px solid #3b82f6;
                border-radius: 6px;
                padding: 12px;
                font-size: 15px;
                color: white;
            }
            QPushButton:hover {
                background-color: #1d4ed8;
            }
        """)
        layout.addWidget(self.joint_btn)
        
        layout.addSpacing(10)
        
        actions_label = QLabel("Actions")
        actions_label.setFont(QFont("Arial", 12, QFont.Bold))
        actions_label.setStyleSheet("background-color: transparent;")
        layout.addWidget(actions_label)
        
        # Button configurations: (icon_name, label, command, name)
        buttons_config = [
            ("rocket", "Launch Simulation", "ros2 launch ur10e_robot_driver robot_driver.launch.py", "launch_sim"),
            ("home", "Home Robot", "ros2 run ur10e_testing_pkg ur10e_home", "home_robot"),
            ("circle", "Circle Follow", "ros2 run ur10e_simulation_pkg action_circle_follow", "circle_follow"),
            ("retract", "Retract", "ros2 run ur10e_simulation_pkg action_retract", "retract"),
            ("clipboard", "Task Manager", "ros2 run ur10e_simulation_pkg action_task_manager", "task_manager"),
        ]
        
        # Create action buttons
        for icon_name, label, command, name in buttons_config:
            row_widget = QWidget()
            row_widget.setStyleSheet("""
                QWidget {
                    background-color: #3a3a3f;
                    border-radius: 6px;
                }
            """)
            row_layout = QHBoxLayout(row_widget)
            row_layout.setContentsMargins(8, 6, 8, 6)
            row_layout.setSpacing(10)
            
            # Icon button (clicking launches the command)
            icon_btn = QPushButton()
            icon_btn.setIcon(get_icon(icon_name))
            icon_btn.setIconSize(QSize(20, 20))
            icon_btn.setFixedSize(36, 36)
            icon_btn.setCursor(Qt.PointingHandCursor)
            icon_btn.clicked.connect(lambda checked, cmd=command, n=name: self.execute_command(cmd, n))
            icon_btn.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    border: none;
                }
                QPushButton:hover {
                    background-color: #454550;
                    border-radius: 4px;
                }
            """)
            row_layout.addWidget(icon_btn)
            
            # Label (also clickable)
            label_btn = QPushButton(label)
            label_btn.setCursor(Qt.PointingHandCursor)
            label_btn.clicked.connect(lambda checked, cmd=command, n=name: self.execute_command(cmd, n))
            label_btn.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    border: none;
                    text-align: left;
                    font-size: 15px;
                    padding: 5px;
                }
                QPushButton:hover {
                    color: #ffffff;
                }
            """)
            row_layout.addWidget(label_btn, stretch=1)
            
            # Close/kill button
            close_btn = QPushButton()
            close_btn.setIcon(get_icon("close"))
            close_btn.setIconSize(QSize(14, 14))
            close_btn.setFixedSize(28, 28)
            close_btn.setCursor(Qt.PointingHandCursor)
            close_btn.setToolTip(f"Stop {label}")
            close_btn.clicked.connect(lambda checked, n=name, lbl=label: self.kill_process(n, lbl))
            close_btn.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    border: none;
                }
                QPushButton:hover {
                    background-color: #5a3a3a;
                    border-radius: 4px;
                }
            """)
            row_layout.addWidget(close_btn)
            
            layout.addWidget(row_widget)
        
        layout.addSpacing(10)
        
        # ========== Status Section ==========
        status_label = QLabel("Status")
        status_label.setFont(QFont("Arial", 12, QFont.Bold))
        status_label.setStyleSheet("background-color: transparent;")
        layout.addWidget(status_label)
        
        # Status header with count
        self.status_header = QLabel("Running (0)")
        self.status_header.setStyleSheet("background-color: transparent; color: #888888;")
        layout.addWidget(self.status_header)
        
        # Process list
        self.process_list = QListWidget()
        self.process_list.setMaximumHeight(100)
        self.process_list.setStyleSheet("""
            QListWidget {
                background-color: #32323a;
                border: 1px solid #4a4a4f;
                border-radius: 6px;
            }
            QListWidget::item {
                color: #7aff7a;
                padding: 3px;
            }
        """)
        layout.addWidget(self.process_list)
        
        layout.addStretch()
        
        # Emergency stop button
        stop_btn = QPushButton("STOP ALL PROCESSES")
        stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #b91c1c;
                border: 1px solid #dc2626;
                border-radius: 6px;
                font-size: 15px;
                font-weight: bold;
                padding: 12px;
                color: white;
            }
            QPushButton:hover {
                background-color: #991b1b;
            }
        """)
        stop_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(stop_btn)
        
        return panel
    
    def create_plot_panel(self):
        """Create the force sensor plotting panel"""
        group = QWidget()
        group.setStyleSheet("""
            QWidget {
                background-color: #32323a;
                border-radius: 10px;
            }
        """)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Title
        title = QLabel("Force Sensor (N)")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setStyleSheet("background-color: transparent;")
        layout.addWidget(title)
        
        # Create pyqtgraph plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#2a2a32')
        self.plot_widget.setLabel('left', 'Force', units='N')
        self.plot_widget.setLabel('bottom', 'Time', units='samples')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.setYRange(-30, 30)
        
        # Create three plot curves with distinct colors
        self.curve_x = self.plot_widget.plot(pen=pg.mkPen(color='#ef4444', width=2), name='X')
        self.curve_y = self.plot_widget.plot(pen=pg.mkPen(color='#22c55e', width=2), name='Y')
        self.curve_z = self.plot_widget.plot(pen=pg.mkPen(color='#3b82f6', width=2), name='Z')
        
        # Add legend
        self.plot_widget.addLegend()
        layout.addWidget(self.plot_widget)
        return group
    
    def setup_plot(self):
        """Initialize plot data"""
        pass
    
    def execute_command(self, command, name):
        """Execute a ROS2 command via subprocess"""
        print(f"Executing: {command}")
        success = self.process_manager.run_command(name, command)
        
        if success:
            self.update_status_display()
        else:
            QMessageBox.warning(self, "Command Failed",
                              f"Failed to start: {name}\n\nCommand: {command}")
    
    def switch_to_cartesian(self):
        """Switch to Cartesian Compliance Controller"""
        command = "ros2 control switch_controllers --deactivate joint_trajectory_controller --activate cartesian_compliance_controller"
        
        print(f"\n{'='*60}")
        print("SWITCHING TO CARTESIAN COMPLIANCE CONTROLLER")
        print(f"{'='*60}")
        
        try:
            result = subprocess.run(
                command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                print("✓ Successfully switched to Cartesian Compliance Controller")
                # Update button styles
                self.cartesian_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #2563eb;
                        border: 1px solid #3b82f6;
                        border-radius: 6px;
                        padding: 12px;
                        font-size: 13px;
                        color: white;
                    }
                    QPushButton:hover {
                        background-color: #1d4ed8;
                    }
                """)
                self.joint_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3a3a3f;
                        border: 1px solid #4a4a4f;
                        border-radius: 6px;
                        padding: 12px;
                        font-size: 13px;
                    }
                    QPushButton:hover {
                        background-color: #454550;
                    }
                """)
            else:
                print(f"✗ Failed to switch controller: {result.stderr}")
                QMessageBox.warning(self, "Switch Failed", 
                    f"Failed to activate Cartesian Compliance Controller.\n\nError: {result.stderr}")
        except subprocess.TimeoutExpired:
            QMessageBox.warning(self, "Timeout", 
                "Controller switch command timed out.\n\nMake sure the robot simulation is running.")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"An error occurred:\n{e}")
        
        print(f"{'='*60}\n")
    
    def switch_to_joint(self):
        """Switch to Joint Trajectory Controller"""
        command = "ros2 control switch_controllers --deactivate cartesian_compliance_controller --activate joint_trajectory_controller"
        
        print(f"\n{'='*60}")
        print("SWITCHING TO JOINT TRAJECTORY CONTROLLER")
        print(f"{'='*60}")
        
        try:
            result = subprocess.run(
                command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                print("✓ Successfully switched to Joint Trajectory Controller")
                # Update button styles
                self.joint_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #2563eb;
                        border: 1px solid #3b82f6;
                        border-radius: 6px;
                        padding: 12px;
                        font-size: 13px;
                        color: white;
                    }
                    QPushButton:hover {
                        background-color: #1d4ed8;
                    }
                """)
                self.cartesian_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #3a3a3f;
                        border: 1px solid #4a4a4f;
                        border-radius: 6px;
                        padding: 12px;
                        font-size: 13px;
                    }
                    QPushButton:hover {
                        background-color: #454550;
                    }
                """)
            else:
                print(f"✗ Failed to switch controller: {result.stderr}")
                QMessageBox.warning(self, "Switch Failed", 
                    f"Failed to activate Joint Trajectory Controller.\n\nError: {result.stderr}")
        except subprocess.TimeoutExpired:
            QMessageBox.warning(self, "Timeout", 
                "Controller switch command timed out.\n\nMake sure the robot simulation is running.")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"An error occurred:\n{e}")
        
        print(f"{'='*60}\n")
    
    def kill_process(self, name, label):
        """Kill a specific process by name"""
        if self.process_manager.terminate_process(name):
            self.update_status_display()
        else:
            QMessageBox.information(self, "Not Running", 
                                  f'"{label}" is not currently running.')
    
    def emergency_stop(self):
        """Stop all running processes"""
        running_processes = self.process_manager.get_running_processes()
        
        if not running_processes:
            QMessageBox.information(self, "No Processes Running", 
                                  "No processes are currently running.")
            return
        
        reply = QMessageBox.question(
            self, 'Emergency Stop',
            f'Stop all {len(running_processes)} running processes?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            print("\n" + "="*50)
            print("EMERGENCY STOP - Terminating all processes...")
            print("="*50)
            
            self.process_manager.terminate_all()
            self.update_status_display()
            
            print("="*50)
            print("All processes terminated.")
            print("="*50 + "\n")
    
    def update_force_data(self, fx, fy, fz):
        """Slot to receive force data from ROS2 (thread-safe)"""
        self.time_counter += 1
        self.time_data.append(self.time_counter)
        self.force_x_data.append(fx)
        self.force_y_data.append(fy)
        self.force_z_data.append(fz)
    
    def update_plot(self):
        """Update the plot with latest data"""
        if len(self.time_data) > 0:
            time_array = np.array(self.time_data)
            fx_array = np.array(self.force_x_data)
            fy_array = np.array(self.force_y_data)
            fz_array = np.array(self.force_z_data)
            
            self.curve_x.setData(time_array, fx_array)
            self.curve_y.setData(time_array, fy_array)
            self.curve_z.setData(time_array, fz_array)
    
    def update_status_display(self):
        """Update the process status display"""
        running_processes = self.process_manager.get_running_processes()
        
        # Update header
        self.status_header.setText(f"Running ({len(running_processes)})")
        
        # Update list
        self.process_list.clear()
        for name in running_processes:
            item = QListWidgetItem(f"● {name.replace('_', ' ').title()}")
            self.process_list.addItem(item)
        
        if running_processes:
            self.status_header.setStyleSheet("background-color: transparent; color: #7aff7a;")
        else:
            self.status_header.setStyleSheet("background-color: transparent; color: #888888;")
    
    def closeEvent(self, event):
        """Handle window close event - automatically stop all processes"""
        running_processes = self.process_manager.get_running_processes()
        
        if running_processes:
            reply = QMessageBox.question(
                self, 'Exit Application',
                f'{len(running_processes)} processes are still running.\n\nTerminate all and exit?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )
            
            if reply == QMessageBox.Yes:
                print("\n" + "="*50)
                print("Shutting down GUI - Terminating all processes...")
                print("="*50)
                
                self.process_manager.terminate_all()
                
                print("="*50)
                print("All processes terminated. Exiting.")
                print("="*50 + "\n")
                
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()


class UR10eGUINode(Node):
    """ROS2 Node for the GUI"""
    
    def __init__(self):
        super().__init__('ur10e_gui_node')
        
        # Create signals for thread-safe Qt updates
        self.signals = ROS2WorkerSignals()
        
        # QoS profile for real-time sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to force/torque sensor
        self.ft_subscription = self.create_subscription(
            WrenchStamped,
            '/cartesian_compliance_controller/ft_sensor_wrench',
            self.ft_callback,
            qos_profile
        )
        
        self.get_logger().info('UR10e GUI Node initialized')
    
    def ft_callback(self, msg):
        """Callback for force/torque sensor data"""
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        
        # Emit signal for Qt GUI (thread-safe)
        self.signals.force_data.emit(fx, fy, fz)


def main(args=None):
    """Main entry point"""
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create ROS2 node
    ros_node = UR10eGUINode()
    
    # Create Qt Application
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # Create and show GUI
    gui = UR10eControlGUI(ros_node)
    gui.show()
    
    # Create timer for spinning ROS2 node
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)  # Spin every 10ms
    
    # Execute Qt application
    exit_code = app.exec_()
    
    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()