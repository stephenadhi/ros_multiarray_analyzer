# ROS2 Array Analyzer

A tool for analyzing ROS2 topics containing UInt32MultiArray or UInt8MultiArray messages. This tool provides a dynamic table visualization and allows tracking specific indices with custom variable names.

## Requirements
- Ubuntu 24.04
- ROS2
- Python 3.10+

## Installation

1. Create a virtual environment (recommended):
```bash
python3 -m venv venv
source venv/bin/activate
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

1. Run the analyzer:
```bash
python3 ros_array_analyzer.py
```

2. Select a ROS2 topic from the dropdown
3. The data type will be automatically detected
4. View all values in the dynamic table
5. Optionally select specific indices to track and name them

## Features
- Dynamic topic selection
- Automatic data type recognition (UInt32MultiArray/UInt8MultiArray)
- Real-time data visualization
- Custom index tracking with variable naming
- Dynamic table view of all array elements 