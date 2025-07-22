FROM ros:jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pyqt6 \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /ros2_analyzer

# Copy application files
COPY ros_array_analyzer.py .
COPY test_publisher.py .
COPY requirements.txt .

# Install Python dependencies
RUN pip3 install -r requirements.txt

# Make scripts executable
RUN chmod +x ros_array_analyzer.py test_publisher.py

# Create entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"] 