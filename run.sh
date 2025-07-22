#!/bin/bash

# Allow X server connection from docker
xhost +local:docker

# Build and run the containers
docker-compose up --build

# Cleanup X server permissions
xhost -local:docker 