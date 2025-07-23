#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import (UInt32MultiArray, UInt8MultiArray, Int32MultiArray,
                         Int8MultiArray, Float32MultiArray, Float64MultiArray,
                         Int16MultiArray, UInt16MultiArray, Int64MultiArray,
                         UInt64MultiArray)
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                           QHBoxLayout, QComboBox, QPushButton, QTableWidget,
                           QTableWidgetItem, QLabel, QSpinBox, QLineEdit,
                           QGroupBox, QScrollArea, QMessageBox, QCheckBox,
                           QHeaderView, QTabWidget, QFileDialog)
from PyQt6.QtCore import QTimer, Qt
import threading
import queue
import subprocess
import signal
import json
import yaml
import re

def parse_json5(content):
    """Parse JSON5 format (simplified parser for Zenoh config)"""
    # Remove comments
    content = re.sub(r'//.*?\n|/\*.*?\*/', '', content, flags=re.DOTALL)
    
    # Allow trailing commas
    content = re.sub(r',(\s*[}\]])', r'\1', content)
    
    # Allow unquoted keys
    content = re.sub(r'([\{\,]\s*)([a-zA-Z_][a-zA-Z0-9_]*)\s*:', r'\1"\2":', content)
    
    # Parse as regular JSON
    return json.loads(content)

class TopicDataWidget(QWidget):
    """Widget for displaying a single topic's data"""
    def __init__(self, topic_name, parent=None):
        super().__init__(parent)
        self.topic_name = topic_name
        self.current_data = []
        self.tracked_indices = {}
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Topic header with close button and tracking controls
        header_layout = QHBoxLayout()
        
        # Left side - topic name and close button
        topic_header = QHBoxLayout()
        topic_header.addWidget(QLabel(f"Topic: {self.topic_name}"))
        close_btn = QPushButton("×")
        close_btn.setMaximumWidth(30)
        close_btn.clicked.connect(lambda: self.parent().remove_topic_tab(self.topic_name))
        topic_header.addWidget(close_btn)
        header_layout.addLayout(topic_header)
        
        # Right side - index tracking controls
        track_layout = QHBoxLayout()
        self.index_spin = QSpinBox()
        self.index_spin.setRange(0, 999)
        self.var_name_edit = QLineEdit()
        self.var_name_edit.setPlaceholderText("Variable Name")
        self.track_btn = QPushButton("Track Index")
        self.track_btn.clicked.connect(self.add_tracked_index)
        self.clear_tracked_btn = QPushButton("Clear Tracked")
        self.clear_tracked_btn.clicked.connect(self.clear_tracked_indices)
        
        track_layout.addWidget(QLabel("Index:"))
        track_layout.addWidget(self.index_spin)
        track_layout.addWidget(QLabel("Name:"))
        track_layout.addWidget(self.var_name_edit)
        track_layout.addWidget(self.track_btn)
        track_layout.addWidget(self.clear_tracked_btn)
        header_layout.addLayout(track_layout)
        
        layout.addLayout(header_layout)

        # Create splitter for tracked and main tables
        tables_layout = QHBoxLayout()
        
        # Tracked indices table (left side)
        tracked_container = QWidget()
        tracked_layout = QVBoxLayout(tracked_container)
        tracked_layout.addWidget(QLabel("Tracked Values:"))
        self.tracked_table = QTableWidget(0, 3)
        self.tracked_table.setHorizontalHeaderLabels(["Index", "Name", "Value"])
        self.tracked_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.tracked_table.setMinimumWidth(300)
        tracked_layout.addWidget(self.tracked_table)
        tables_layout.addWidget(tracked_container, stretch=1)
        
        # Main data table (right side)
        data_container = QWidget()
        data_layout = QVBoxLayout(data_container)
        data_layout.addWidget(QLabel("All Array Values (100 per column):"))
        self.table = QTableWidget(0, 0)
        self.table.setHorizontalScrollMode(QTableWidget.ScrollMode.ScrollPerPixel)
        self.table.setVerticalScrollMode(QTableWidget.ScrollMode.ScrollPerPixel)
        self.table.setAlternatingRowColors(True)
        data_layout.addWidget(self.table)
        tables_layout.addWidget(data_container, stretch=2)
        
        layout.addLayout(tables_layout)

    def update_data(self, data):
        """Update widget with new data"""
        self.current_data = data
        self.update_tables()

    def update_tables(self):
        self.update_tracked_table()
        self.update_main_table()

    def update_tracked_table(self):
        self.tracked_table.setRowCount(len(self.tracked_indices))
        for row, (index, name) in enumerate(self.tracked_indices.items()):
            self.tracked_table.setItem(row, 0, QTableWidgetItem(str(index)))
            self.tracked_table.setItem(row, 1, QTableWidgetItem(name))
            if index < len(self.current_data):
                value = self.current_data[index]
                if isinstance(value, float):
                    value_str = f"{value:.3f}"
                else:
                    value_str = str(value)
                self.tracked_table.setItem(row, 2, QTableWidgetItem(value_str))
            else:
                self.tracked_table.setItem(row, 2, QTableWidgetItem("N/A"))
        self.tracked_table.resizeColumnsToContents()

    def update_main_table(self):
        """Update the main data table with explicit indices"""
        if not self.current_data:
            self.table.setRowCount(0)
            self.table.setColumnCount(0)
            return

        values_per_column = 100
        num_values = len(self.current_data)
        num_columns = (num_values + values_per_column - 1) // values_per_column * 2
        max_rows = min(values_per_column, num_values)

        # Setup table
        self.table.setRowCount(max_rows)
        self.table.setColumnCount(num_columns)

        # Setup headers
        headers = []
        for col in range(0, num_columns, 2):
            headers.extend(["Index", "Value"])
        self.table.setHorizontalHeaderLabels(headers)

        # Fill data
        for value_idx, value in enumerate(self.current_data):
            column_pair = (value_idx // values_per_column) * 2
            row = value_idx % values_per_column

            # Set index
            index_item = QTableWidgetItem(str(value_idx))
            index_item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.table.setItem(row, column_pair, index_item)

            # Set value
            if isinstance(value, float):
                value_str = f"{value:.3f}"
            else:
                value_str = str(value)
            value_item = QTableWidgetItem(value_str)
            value_item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.table.setItem(row, column_pair + 1, value_item)

        # Adjust column widths
        for col in range(num_columns):
            self.table.resizeColumnToContents(col)
            width = self.table.columnWidth(col)
            self.table.setColumnWidth(col, width + 10)

    def add_tracked_index(self):
        """Add a new index to track"""
        index = self.index_spin.value()
        name = self.var_name_edit.text()
        
        if not name:
            QMessageBox.warning(self, "Warning", "Please enter a variable name")
            return
            
        if not self.current_data:
            QMessageBox.warning(self, "Warning", "No data available")
            return
            
        if index >= len(self.current_data):
            QMessageBox.warning(self, "Warning", f"Index {index} is out of range")
            return
            
        self.tracked_indices[index] = name
        self.update_tracked_table()
        self.var_name_edit.clear()

    def clear_tracked_indices(self):
        """Clear all tracked indices"""
        self.tracked_indices.clear()
        self.update_tracked_table()

class ROSArrayAnalyzer(QMainWindow):
    SUPPORTED_TYPES = {
        'UInt8MultiArray': UInt8MultiArray,
        'UInt16MultiArray': UInt16MultiArray,
        'UInt32MultiArray': UInt32MultiArray,
        'UInt64MultiArray': UInt64MultiArray,
        'Int8MultiArray': Int8MultiArray,
        'Int16MultiArray': Int16MultiArray,
        'Int32MultiArray': Int32MultiArray,
        'Int64MultiArray': Int64MultiArray,
        'Float32MultiArray': Float32MultiArray,
        'Float64MultiArray': Float64MultiArray,
    }

    RMW_IMPLEMENTATIONS = {
        'Zenoh': 'rmw_zenoh_cpp',
        'FastRTPS': 'rmw_fastrtps_cpp'
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Array Analyzer")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize variables
        self.current_domain_id = int(os.getenv('ROS_DOMAIN_ID', '0'))
        self.data_queue = queue.Queue()
        self.node = None
        self.executor = None
        self.ros_thread = None
        self.subscriptions = {}  # {topic_name: subscription}
        self.topic_widgets = {}  # {topic_name: TopicDataWidget}
        self.zenoh_router_process = None  # For Zenoh router management
        self.config_file_path = None
        
        # Setup UI
        self.setup_ui()
        
        # Disable most of the UI until domain is selected
        self.set_ui_enabled(False)
        
        # Timer for updating UI
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.process_queue)
        self.update_timer.start(50)

    def set_ui_enabled(self, enabled):
        """Enable or disable UI elements"""
        self.topic_combo.setEnabled(enabled)
        self.refresh_btn.setEnabled(enabled)
        self.type_filter.setEnabled(enabled)
        self.add_topic_btn.setEnabled(enabled)
        self.tab_widget.setEnabled(enabled)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Configuration section
        config_group = QGroupBox("ROS2 Configuration")
        config_layout = QVBoxLayout()
        
        # RMW Implementation selector
        rmw_row = QHBoxLayout()
        rmw_label = QLabel("RMW Implementation:")
        self.rmw_combo = QComboBox()
        self.rmw_combo.addItems(self.RMW_IMPLEMENTATIONS.keys())
        self.rmw_combo.setCurrentText("Zenoh")
        self.rmw_combo.currentTextChanged.connect(self.on_rmw_changed)
        
        rmw_row.addWidget(rmw_label)
        rmw_row.addWidget(self.rmw_combo)
        rmw_row.addStretch()
        
        # Network Configuration
        network_row = QHBoxLayout()
        
        # Remote IP
        remote_ip_layout = QHBoxLayout()
        remote_ip_label = QLabel("Remote IP:")
        self.remote_ip_edit = QLineEdit()
        self.remote_ip_edit.setPlaceholderText("e.g., 192.168.1.100")
        remote_ip_layout.addWidget(remote_ip_label)
        remote_ip_layout.addWidget(self.remote_ip_edit)
        
        # Interface
        iface_layout = QHBoxLayout()
        iface_label = QLabel("Interface:")
        self.iface_edit = QLineEdit()
        self.iface_edit.setPlaceholderText("e.g., eth0")
        iface_layout.addWidget(iface_label)
        iface_layout.addWidget(self.iface_edit)
        
        network_row.addLayout(remote_ip_layout)
        network_row.addLayout(iface_layout)
        
        # Domain ID row
        domain_row = QHBoxLayout()
        domain_label = QLabel("Domain ID:")
        self.domain_spin = QSpinBox()
        self.domain_spin.setRange(0, 232)
        self.domain_spin.setValue(self.current_domain_id)
        self.domain_apply_btn = QPushButton("Start ROS2 Connection")
        self.domain_apply_btn.clicked.connect(self.change_domain)
        
        domain_row.addWidget(domain_label)
        domain_row.addWidget(self.domain_spin)
        domain_row.addWidget(self.domain_apply_btn)
        domain_row.addStretch()
        
        # Add rows to config layout
        config_layout.addLayout(rmw_row)
        config_layout.addLayout(network_row)
        config_layout.addLayout(domain_row)
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)
        
        # Zenoh Configuration
        self.zenoh_config_group = QGroupBox("Zenoh Configuration")
        zenoh_layout = QHBoxLayout()
        
        self.start_router_cb = QCheckBox("Start Zenoh Router")
        self.start_router_cb.setChecked(False)
        self.peer_discovery_cb = QCheckBox("Enable Peer Discovery")
        self.peer_discovery_cb.setChecked(False)  # Default to false as per standard config
        
        zenoh_layout.addWidget(self.start_router_cb)
        zenoh_layout.addWidget(self.peer_discovery_cb)
        zenoh_layout.addStretch()
        
        self.zenoh_config_group.setLayout(zenoh_layout)
        layout.addWidget(self.zenoh_config_group)
        
        # Connection status
        self.status_label = QLabel("Status: Not Connected")
        layout.addWidget(self.status_label)
        
        # Topic selection group
        topic_group = QGroupBox("Topic Selection")
        topic_layout = QHBoxLayout()
        topic_label = QLabel("Add Topic:")
        self.topic_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh Topics")
        self.refresh_btn.clicked.connect(self.refresh_topics)
        
        self.type_filter = QComboBox()
        self.type_filter.addItem("All Types")
        self.type_filter.addItems(sorted(self.SUPPORTED_TYPES.keys()))
        self.type_filter.currentTextChanged.connect(self.refresh_topics)
        
        self.add_topic_btn = QPushButton("Add Topic")
        self.add_topic_btn.clicked.connect(self.add_selected_topic)
        
        topic_layout.addWidget(topic_label)
        topic_layout.addWidget(self.topic_combo)
        topic_layout.addWidget(QLabel("Filter Type:"))
        topic_layout.addWidget(self.type_filter)
        topic_layout.addWidget(self.refresh_btn)
        topic_layout.addWidget(self.add_topic_btn)
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        # Tab widget for multiple topics
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabsClosable(True)
        self.tab_widget.tabCloseRequested.connect(self.remove_topic_tab)
        layout.addWidget(self.tab_widget)

    def on_rmw_changed(self, rmw_name):
        """Handle RMW implementation change"""
        # Show/hide Zenoh specific configuration
        self.zenoh_config_group.setVisible(rmw_name == "Zenoh")
        
        if self.node:  # If already connected, warn about reconnection needed
            QMessageBox.information(self, "RMW Changed",
                                  "Please reconnect to apply the new RMW implementation.")

    def generate_zenoh_config(self):
        """Generate standard Zenoh configuration"""
        remote_ip = self.remote_ip_edit.text().strip() or "localhost"
        iface = self.iface_edit.text().strip() or "lo"
        
        config = {
            "mode": "peer",
            "connect": {
                "timeout_ms": -1,
                "endpoints": {
                    "router": [f"tcp/{remote_ip}:55020#iface={iface}"],
                    "peer": ["tcp/localhost:55000"],
                    "client": ["tcp/localhost:55000"],
                },
                "exit_on_failure": {
                    "router": False,
                    "peer": False,
                    "client": True
                },
                "retry": {
                    "period_init_ms": 1000,
                    "period_max_ms": 4000,
                    "period_increase_factor": 2,
                },
            },
            "listen": {
                "timeout_ms": 0,
                "endpoints": {
                    "router": [
                        f"tcp/{remote_ip}:55020#iface={iface}",
                        "tcp/localhost:55000"
                    ],
                    "peer": ["tcp/localhost:0"],
                    "client": ["tcp/localhost:0"]
                },
                "exit_on_failure": {
                    "router": False,
                    "peer": True,
                    "client": True
                },
                "retry": {
                    "period_init_ms": 1000,
                    "period_max_ms": 4000,
                    "period_increase_factor": 2,
                },
            },
            "scouting": {
                "multicast": {
                    "enabled": False,
                },
            },
            "timestamping": {
                "enabled": True,
                "drop_future_timestamp": False,
            },
        }
        
        return config

    def init_ros(self):
        """Initialize or reinitialize ROS2 with current domain ID"""
        try:
            # Clean up existing ROS2 resources
            self.cleanup_ros()
            
            # Set RMW implementation
            rmw_name = self.rmw_combo.currentText()
            os.environ['RMW_IMPLEMENTATION'] = self.RMW_IMPLEMENTATIONS[rmw_name]
            os.environ['ROS_DOMAIN_ID'] = str(self.current_domain_id)
            
            # Set default Zenoh router check attempts
            os.environ['ZENOH_ROUTER_CHECK_ATTEMPTS'] = '5'
            
            if rmw_name == "Zenoh":
                # Generate config based on UI settings
                config = self.generate_zenoh_config()
                
                # Create temporary config file
                config_dir = "/tmp/zenoh"
                os.makedirs(config_dir, exist_ok=True)
                config_path = os.path.join(config_dir, "analyzer_config.json5")
                
                # Add JSON5 comments
                config_content = """/// ROS2 Array Analyzer Zenoh Configuration
/// Generated configuration for RMW Zenoh
/// See https://github.com/ros2/rmw_zenoh/tree/jazzy for more details.
"""
                config_content += json.dumps(config, indent=2)
                
                with open(config_path, 'w') as f:
                    f.write(config_content)
                
                # Set Zenoh config environment variables
                os.environ['ZENOH_SESSION_CONFIG_URI'] = config_path
                
                print("Using Zenoh configuration:")
                print(config_content)
                
                if self.start_router_cb.isChecked():
                    try:
                        if self.zenoh_router_process:
                            self.zenoh_router_process.terminate()
                            self.zenoh_router_process.wait(timeout=2)
                        
                        router_config = dict(config)
                        router_config["mode"] = "router"
                        router_config_path = os.path.join(config_dir, "analyzer_router_config.json5")
                        
                        with open(router_config_path, 'w') as f:
                            f.write("/// Zenoh Router Configuration\n")
                            json.dump(router_config, f, indent=2)
                        
                        router_cmd = ['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd',
                                    '--config', router_config_path]
                        
                        self.zenoh_router_process = subprocess.Popen(
                            router_cmd,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            preexec_fn=os.setsid
                        )
                        QTimer.singleShot(2000, self.continue_ros_init)
                        return True
                    except Exception as e:
                        QMessageBox.warning(self, "Warning", 
                                          f"Failed to start Zenoh router: {str(e)}\n"
                                          "Will attempt to connect to existing router.")
                        return self.continue_ros_init()
            
            return self.continue_ros_init()
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to initialize ROS2: {str(e)}")
            self.status_label.setText("Status: Connection Failed")
            return False

    def continue_ros_init(self):
        """Continue ROS2 initialization after router setup"""
        try:
            # Initialize ROS2
            if not rclpy.ok():
                rclpy.init()
            
            # Create node and executor
            self.node = Node('array_analyzer')
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Start ROS2 spin in separate thread
            self.running = True
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
            # Update UI
            self.status_label.setText(f"Status: Connected (Domain ID: {self.current_domain_id})")
            self.domain_apply_btn.setText("Change Domain ID")
            self.set_ui_enabled(True)
            
            # Initial topic refresh
            self.refresh_topics()
            
            return True
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to initialize ROS2: {str(e)}")
            self.status_label.setText("Status: Connection Failed")
            return False

    def cleanup_ros(self):
        """Clean up ROS2 resources"""
        # Clean up all topic subscriptions
        for topic_name in list(self.subscriptions.keys()):
            if topic_name in self.subscriptions:
                self.node.destroy_subscription(self.subscriptions[topic_name])
                del self.subscriptions[topic_name]
        
        # Clean up all topic widgets
        for topic_name in list(self.topic_widgets.keys()):
            if topic_name in self.topic_widgets:
                del self.topic_widgets[topic_name]
        
        # Clear all tabs
        while self.tab_widget.count() > 0:
            self.tab_widget.removeTab(0)
        
        # Stop Zenoh router if we started it
        if self.zenoh_router_process:
            try:
                os.killpg(os.getpgid(self.zenoh_router_process.pid), signal.SIGTERM)
                self.zenoh_router_process.wait(timeout=2)
            except Exception as e:
                print(f"Router cleanup error (continuing): {str(e)}")
            self.zenoh_router_process = None
        
        # Clean up ROS2 node and executor
        if self.executor:
            try:
                self.executor.shutdown()
            except Exception as e:
                print(f"Executor shutdown error (continuing): {str(e)}")
            self.executor = None
        
        if self.node:
            try:
                self.node.destroy_node()
            except Exception as e:
                print(f"Node cleanup error (continuing): {str(e)}")
            self.node = None
        
        # Shutdown ROS2 if it's running
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"ROS2 shutdown error (continuing): {str(e)}")
        
        # Clear the message queue
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break

    def change_domain(self):
        new_domain = self.domain_spin.value()
        if new_domain != self.current_domain_id or not self.node:
            self.current_domain_id = new_domain
            self.status_label.setText("Status: Connecting...")
            self.set_ui_enabled(False)
            
            # Clear existing data and tabs
            while self.tab_widget.count() > 0:
                self.remove_topic_tab(0)
            
            # Initialize ROS2 with new domain
            if self.init_ros():
                QMessageBox.information(self, "Success", 
                                      f"Connected to ROS2 Domain ID: {new_domain}")
            else:
                self.status_label.setText("Status: Connection Failed")

    def refresh_topics(self):
        """Refresh the list of available topics"""
        if not self.node:
            self.topic_combo.clear()
            return
            
        # Get list of topics
        topic_list = []
        selected_type = self.type_filter.currentText()
        
        for topic_name, topic_type in self.node.get_topic_names_and_types():
            type_name = topic_type[0].split('/')[-1]
            if type_name in self.SUPPORTED_TYPES.keys():
                if selected_type == "All Types" or selected_type == type_name:
                    topic_list.append((topic_name, type_name))
        
        self.topic_combo.clear()
        for topic_name, _ in topic_list:
            self.topic_combo.addItem(topic_name)

    def subscribe_to_topic(self, topic_name):
        """Subscribe to the selected topic"""
        if not topic_name or not self.node:
            return
            
        if topic_name in self.subscriptions:
            self.node.destroy_subscription(self.subscriptions[topic_name])
        
        # Get topic type
        for name, types in self.node.get_topic_names_and_types():
            if name == topic_name:
                type_name = types[0].split('/')[-1]
                if type_name in self.SUPPORTED_TYPES:
                    self.msg_type = self.SUPPORTED_TYPES[type_name]
                    self.type_label.setText(f"Data Type: {type_name}")
                    break
        
        if self.msg_type:
            self.subscriptions[topic_name] = self.node.create_subscription(
                self.msg_type,
                topic_name,
                lambda msg, topic=topic_name: self.topic_callback(msg, topic),
                10)
            
            # Reset statistics
            self.update_statistics([])

    def topic_callback(self, msg, topic_name):
        """Handle message from a specific topic"""
        if topic_name in self.topic_widgets:
            self.data_queue.put((topic_name, msg.data))

    def update_statistics(self, data):
        """Removed statistics update as no longer needed"""
        pass

    def process_queue(self):
        """Process any pending data from the ROS thread"""
        try:
            while True:
                topic_name, data = self.data_queue.get_nowait()
                if topic_name in self.topic_widgets:
                    self.topic_widgets[topic_name].update_data(data)
        except queue.Empty:
            pass

    def update_display(self):
        """Update all UI elements with current data"""
        self.update_tracked_table()
        self.update_main_table()

    def update_main_table(self):
        """Update the main data table with multi-column layout (100 values per column)"""
        if not self.current_data:
            self.table.setRowCount(0)
            self.table.setColumnCount(0)
            return

        # Calculate number of columns needed (each column holds 100 values)
        values_per_column = 100
        num_values = len(self.current_data)
        num_columns = (num_values + values_per_column - 1) // values_per_column * 2  # *2 for index+value columns
        max_rows = min(values_per_column, num_values)

        # Setup table
        self.table.setRowCount(max_rows)
        self.table.setColumnCount(num_columns)

        # Setup headers
        headers = []
        for col in range(0, num_columns, 2):
            start_idx = (col // 2) * values_per_column
            headers.extend([f"Index ({start_idx}-{min(start_idx + values_per_column - 1, num_values - 1)})", "Value"])
        self.table.setHorizontalHeaderLabels(headers)

        # Fill data
        for value_idx, value in enumerate(self.current_data):
            # Calculate which column pair and row this value belongs to
            column_pair = (value_idx // values_per_column) * 2
            row = value_idx % values_per_column

            # Set index
            index_item = QTableWidgetItem(str(value_idx))
            index_item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.table.setItem(row, column_pair, index_item)

            # Set value
            if isinstance(value, float):
                value_str = f"{value:.3f}"
            else:
                value_str = str(value)
            value_item = QTableWidgetItem(value_str)
            value_item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.table.setItem(row, column_pair + 1, value_item)

        # Adjust column widths and make them fixed
        for col in range(num_columns):
            self.table.resizeColumnToContents(col)
            width = self.table.columnWidth(col)
            self.table.setColumnWidth(col, width + 10)  # Add a little padding

        # Enable sorting
        self.table.setSortingEnabled(True)

    def add_selected_topic(self):
        topic_name = self.topic_combo.currentText()
        if not topic_name or topic_name in self.topic_widgets:
            return

        # Create widget and subscription for the topic
        widget = TopicDataWidget(topic_name)
        self.topic_widgets[topic_name] = widget
        self.tab_widget.addTab(widget, topic_name)
        
        # Create subscription
        for name, types in self.node.get_topic_names_and_types():
            if name == topic_name:
                type_name = types[0].split('/')[-1]
                if type_name in self.SUPPORTED_TYPES:
                    msg_type = self.SUPPORTED_TYPES[type_name]
                    self.subscriptions[topic_name] = self.node.create_subscription(
                        msg_type,
                        topic_name,
                        lambda msg, topic=topic_name: self.topic_callback(msg, topic),
                        10)
                    break

    def remove_topic_tab(self, index_or_name):
        if isinstance(index_or_name, str):
            # Find the index for the topic name
            for i in range(self.tab_widget.count()):
                if self.tab_widget.tabText(i) == index_or_name:
                    index = i
                    break
        else:
            index = index_or_name
            
        topic_name = self.tab_widget.tabText(index)
        
        # Remove subscription
        if topic_name in self.subscriptions:
            self.node.destroy_subscription(self.subscriptions[topic_name])
            del self.subscriptions[topic_name]
        
        # Remove widget
        if topic_name in self.topic_widgets:
            del self.topic_widgets[topic_name]
        
        # Remove tab
        self.tab_widget.removeTab(index)

    def ros_spin(self):
        """ROS2 spin loop with Zenoh-specific error handling"""
        while self.running and rclpy.ok() and self.executor:
            try:
                self.executor.spin_once(timeout_sec=0.01)
            except Exception as e:
                if self.running:  # Only print errors if we're still meant to be running
                    print(f"ROS spin error (continuing): {str(e)}")
                continue

    def closeEvent(self, event):
        """Clean up ROS2 resources on exit"""
        self.cleanup_ros()
        super().closeEvent(event)

def main():
    app = QApplication(sys.argv)
    window = ROSArrayAnalyzer()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main() 