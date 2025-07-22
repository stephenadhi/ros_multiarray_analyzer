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
                           QGroupBox, QScrollArea, QMessageBox)
from PyQt6.QtCore import QTimer, Qt
import threading
import queue

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

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Array Analyzer (Zenoh)")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize variables
        self.current_domain_id = int(os.getenv('ROS_DOMAIN_ID', '0'))
        self.data_queue = queue.Queue()
        self.node = None
        self.executor = None
        self.ros_thread = None
        self.subscription = None
        self.msg_type = None
        self.current_data = []
        self.tracked_indices = {}
        self.running = True # Added for thread control
        
        # Setup UI
        self.setup_ui()
        
        # Disable most of the UI until domain is selected
        self.set_ui_enabled(False)
        
        # Timer for updating UI
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.process_queue)
        self.update_timer.start(50)  # Update every 50ms

    def set_ui_enabled(self, enabled):
        """Enable or disable UI elements"""
        self.topic_combo.setEnabled(enabled)
        self.refresh_btn.setEnabled(enabled)
        self.type_filter.setEnabled(enabled)
        self.index_spin.setEnabled(enabled)
        self.var_name_edit.setEnabled(enabled)
        self.track_btn.setEnabled(enabled)
        self.clear_tracked_btn.setEnabled(enabled)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Domain ID selector
        domain_group = QGroupBox("ROS2 Domain Configuration")
        domain_layout = QHBoxLayout()
        domain_label = QLabel("Domain ID:")
        self.domain_spin = QSpinBox()
        self.domain_spin.setRange(0, 232)  # ROS2 domain ID range
        self.domain_spin.setValue(self.current_domain_id)
        self.domain_apply_btn = QPushButton("Start ROS2 Connection")
        self.domain_apply_btn.clicked.connect(self.change_domain)
        
        domain_layout.addWidget(domain_label)
        domain_layout.addWidget(self.domain_spin)
        domain_layout.addWidget(self.domain_apply_btn)
        domain_layout.addStretch()
        domain_group.setLayout(domain_layout)
        layout.addWidget(domain_group)
        
        # Connection status
        self.status_label = QLabel("Status: Not Connected")
        layout.addWidget(self.status_label)
        
        # Topic selection
        topic_group = QGroupBox("Topic Selection")
        topic_layout = QHBoxLayout()
        topic_label = QLabel("Select Topic:")
        self.topic_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh Topics")
        self.refresh_btn.clicked.connect(self.refresh_topics)
        
        # Add type filter
        self.type_filter = QComboBox()
        self.type_filter.addItem("All Types")
        self.type_filter.addItems(sorted(self.SUPPORTED_TYPES.keys()))
        self.type_filter.currentTextChanged.connect(self.refresh_topics)
        
        topic_layout.addWidget(topic_label)
        topic_layout.addWidget(self.topic_combo)
        topic_layout.addWidget(QLabel("Filter Type:"))
        topic_layout.addWidget(self.type_filter)
        topic_layout.addWidget(self.refresh_btn)
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        # Data type display
        self.type_label = QLabel("Data Type: Not Selected")
        layout.addWidget(self.type_label)
        
        # Statistics Group
        stats_group = QGroupBox("Array Statistics")
        stats_layout = QHBoxLayout()
        self.length_label = QLabel("Length: 0")
        self.min_label = QLabel("Min: N/A")
        self.max_label = QLabel("Max: N/A")
        self.avg_label = QLabel("Avg: N/A")
        stats_layout.addWidget(self.length_label)
        stats_layout.addWidget(self.min_label)
        stats_layout.addWidget(self.max_label)
        stats_layout.addWidget(self.avg_label)
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)
        
        # Index tracking controls
        track_group = QGroupBox("Track Specific Indices")
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
        track_group.setLayout(track_layout)
        layout.addWidget(track_group)
        
        # Tracked indices table
        self.tracked_table = QTableWidget(0, 3)
        self.tracked_table.setHorizontalHeaderLabels(["Index", "Name", "Value"])
        self.tracked_table.setMinimumHeight(150)
        layout.addWidget(self.tracked_table)
        
        # All values table
        table_label = QLabel("All Array Values:")
        layout.addWidget(table_label)
        
        # Make the main table scrollable
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["Index", "Value"])
        scroll.setWidget(self.table)
        layout.addWidget(scroll)
        
        # Connect topic selection signal
        self.topic_combo.currentTextChanged.connect(self.subscribe_to_topic)
        
        # Initial topic refresh
        self.refresh_topics()

    def init_ros(self):
        """Initialize or reinitialize ROS2 with current domain ID"""
        try:
            # Clean up existing ROS2 resources
            self.cleanup_ros()
            
            # Set RMW implementation to Zenoh
            os.environ['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'
            os.environ['ROS_DOMAIN_ID'] = str(self.current_domain_id)
            
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
        # Signal thread to stop
        self.running = False
        
        # Clean up executor
        if self.executor:
            try:
                self.executor.shutdown()
                self.executor = None
            except Exception as e:
                print(f"Executor shutdown error (continuing): {str(e)}")

        # Clean up node
        if self.node:
            try:
                self.node.destroy_node()
                self.node = None
            except Exception as e:
                print(f"Node cleanup error (continuing): {str(e)}")

        # Shutdown ROS2 if it's running
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"ROS2 shutdown error (continuing): {str(e)}")

        # Wait for thread to finish
        if self.ros_thread and self.ros_thread.is_alive():
            try:
                self.ros_thread.join(timeout=2.0)
            except Exception as e:
                print(f"Thread join error (continuing): {str(e)}")
            self.ros_thread = None

        # Clear any remaining data
        self.subscription = None
        self.msg_type = None
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
            
            # Clear existing data
            self.current_data = []
            self.tracked_indices.clear()
            self.update_display()
            
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
            
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
        
        # Get topic type
        for name, types in self.node.get_topic_names_and_types():
            if name == topic_name:
                type_name = types[0].split('/')[-1]
                if type_name in self.SUPPORTED_TYPES:
                    self.msg_type = self.SUPPORTED_TYPES[type_name]
                    self.type_label.setText(f"Data Type: {type_name}")
                    break
        
        if self.msg_type:
            self.subscription = self.node.create_subscription(
                self.msg_type,
                topic_name,
                self.topic_callback,
                10)
            
            # Reset statistics
            self.update_statistics([])

    def topic_callback(self, msg):
        """Thread-safe message handling"""
        self.data_queue.put(msg.data)

    def update_statistics(self, data):
        if not data:
            self.length_label.setText("Length: 0")
            self.min_label.setText("Min: N/A")
            self.max_label.setText("Max: N/A")
            self.avg_label.setText("Avg: N/A")
            return
            
        try:
            self.length_label.setText(f"Length: {len(data)}")
            self.min_label.setText(f"Min: {min(data):.3f}")
            self.max_label.setText(f"Max: {max(data):.3f}")
            self.avg_label.setText(f"Avg: {sum(data)/len(data):.3f}")
        except (TypeError, ValueError):
            # Handle case where data cannot be converted to float
            self.min_label.setText(f"Min: {min(data)}")
            self.max_label.setText(f"Max: {max(data)}")
            self.avg_label.setText("Avg: N/A")

    def add_tracked_index(self):
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
        self.tracked_indices.clear()
        self.update_tracked_table()

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

    def process_queue(self):
        """Process any pending data from the ROS thread"""
        try:
            while True:  # Process all available messages
                data = self.data_queue.get_nowait()
                self.current_data = data
                self.update_statistics(data)
                self.update_display()
        except queue.Empty:
            pass

    def update_display(self):
        """Update all UI elements with current data"""
        self.update_tracked_table()
        self.update_main_table()

    def update_main_table(self):
        """Update the main data table"""
        self.table.setRowCount(len(self.current_data))
        for i, value in enumerate(self.current_data):
            if self.table.item(i, 0) is None:
                self.table.setItem(i, 0, QTableWidgetItem(str(i)))
            
            if isinstance(value, float):
                value_str = f"{value:.3f}"
            else:
                value_str = str(value)
            self.table.setItem(i, 1, QTableWidgetItem(value_str))

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