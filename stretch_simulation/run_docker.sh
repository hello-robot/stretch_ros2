#!/bin/bash

# Stretch Simulation Docker Runner Script
# This script simplifies running the Stretch Simulation Docker container with GPU support

set -e

# Configuration
IMAGE_NAME="stretch-simulation:latest"
CONTAINER_NAME="stretch-sim"
DATA_DIR="${HOME}/stretch_data"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Nvidia Container Toolkit is installed
if ! docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi &> /dev/null; then
    print_warning "Nvidia Container Toolkit may not be properly configured."
    print_warning "GPU acceleration may not work. See DOCKER_SETUP.md for installation instructions."
fi

# Check if image exists
if ! docker image inspect ${IMAGE_NAME} &> /dev/null; then
    print_error "Docker image '${IMAGE_NAME}' not found."
    print_info "Please build the image first with: docker build -t ${IMAGE_NAME} ."
    exit 1
fi

# Create data directory if it doesn't exist
if [ ! -d "${DATA_DIR}" ]; then
    print_info "Creating data directory at ${DATA_DIR}"
    mkdir -p "${DATA_DIR}"
fi

# Allow X11 connections from Docker
print_info "Configuring X11 access..."
xhost +local:docker > /dev/null 2>&1 || print_warning "Could not configure xhost. GUI may not work."

# Parse command line arguments
COMMAND=""
DETACHED=false
INTERACTIVE=true

while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--detached)
            DETACHED=true
            INTERACTIVE=false
            shift
            ;;
        -c|--command)
            COMMAND="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -d, --detached    Run container in detached mode"
            echo "  -c, --command     Run specific command in container"
            echo "  -h, --help        Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Run interactive shell"
            echo "  $0 -d                                 # Run in background"
            echo "  $0 -c 'ros2 launch stretch_simulation stretch_mujoco_driver.launch.py'"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Build docker run command
DOCKER_CMD="docker run --rm"

if [ "$INTERACTIVE" = true ]; then
    DOCKER_CMD="$DOCKER_CMD -it"
fi

if [ "$DETACHED" = true ]; then
    DOCKER_CMD="$DOCKER_CMD -d"
fi

DOCKER_CMD="$DOCKER_CMD \
    --name ${CONTAINER_NAME} \
    --gpus all \
    --env DISPLAY=${DISPLAY} \
    --env QT_X11_NO_MITSHM=1 \
    --env MUJOCO_GL=egl \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume ${HOME}/.Xauthority:/root/.Xauthority:rw \
    --volume ${DATA_DIR}:/root/stretch_user:rw \
    --network host \
    --privileged \
    --shm-size=2g \
    ${IMAGE_NAME}"

if [ -n "$COMMAND" ]; then
    DOCKER_CMD="$DOCKER_CMD bash -c 'source /opt/ros/humble/setup.bash && source ~/ament_ws/install/setup.bash && $COMMAND'"
fi

# Run the container
print_info "Starting Stretch Simulation container..."
print_info "Data directory: ${DATA_DIR}"

if [ "$DETACHED" = true ]; then
    print_info "Running in detached mode..."
    eval $DOCKER_CMD
    print_info "Container started. Use 'docker logs -f ${CONTAINER_NAME}' to view logs"
    print_info "Use 'docker exec -it ${CONTAINER_NAME} bash' to attach to the container"
else
    print_info "Running in interactive mode..."
    eval $DOCKER_CMD
fi

# Cleanup X11 access (optional, commented out for security)
# xhost -local:docker > /dev/null 2>&1

print_info "Container stopped."

