# Makefile for Stretch Simulation Docker Environment

.PHONY: help build run run-detached stop clean shell logs install-nvidia-toolkit test-gpu

# Configuration
IMAGE_NAME := stretch-simulation
IMAGE_TAG := latest
CONTAINER_NAME := stretch-sim
DATA_DIR := $(HOME)/stretch_data

help:
	@echo "Stretch Simulation Docker Commands:"
	@echo ""
	@echo "Setup:"
	@echo "  make install-nvidia-toolkit  Install Nvidia Container Toolkit"
	@echo "  make test-gpu                Test GPU access in Docker"
	@echo "  make build                   Build the Docker image"
	@echo ""
	@echo "Running:"
	@echo "  make run                     Run container interactively"
	@echo "  make run-detached            Run container in background"
	@echo "  make shell                   Open shell in running container"
	@echo "  make stop                    Stop the container"
	@echo ""
	@echo "Maintenance:"
	@echo "  make logs                    View container logs"
	@echo "  make clean                   Remove container and image"
	@echo "  make clean-all               Remove everything including data"

install-nvidia-toolkit:
	@echo "Installing Nvidia Container Toolkit..."
	@distribution=$$(. /etc/os-release; echo $$ID$$VERSION_ID); \
	curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg; \
	curl -s -L https://nvidia.github.io/libnvidia-container/$$distribution/libnvidia-container.list | \
		sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
		sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
	sudo apt-get update
	sudo apt-get install -y nvidia-container-toolkit
	sudo nvidia-ctk runtime configure --runtime=docker
	sudo systemctl restart docker
	@echo "Nvidia Container Toolkit installed successfully!"

test-gpu:
	@echo "Testing GPU access in Docker..."
	docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

build:
	@echo "Building Docker image $(IMAGE_NAME):$(IMAGE_TAG)..."
	@echo "This may take 1-2 hours..."
	docker build -t $(IMAGE_NAME):$(IMAGE_TAG) .
	@echo "Build complete!"

run:
	@echo "Starting container in interactive mode..."
	@mkdir -p $(DATA_DIR)
	@xhost +local:docker > /dev/null 2>&1 || true
	docker run -it --rm \
		--name $(CONTAINER_NAME) \
		--gpus all \
		--env DISPLAY=$(DISPLAY) \
		--env QT_X11_NO_MITSHM=1 \
		--env MUJOCO_GL=egl \
		--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
		--volume $(HOME)/.Xauthority:/root/.Xauthority:rw \
		--volume $(DATA_DIR):/root/stretch_user:rw \
		--network host \
		--privileged \
		--shm-size=2g \
		$(IMAGE_NAME):$(IMAGE_TAG)

run-detached:
	@echo "Starting container in detached mode..."
	@mkdir -p $(DATA_DIR)
	@xhost +local:docker > /dev/null 2>&1 || true
	docker run -d \
		--name $(CONTAINER_NAME) \
		--gpus all \
		--env DISPLAY=$(DISPLAY) \
		--env QT_X11_NO_MITSHM=1 \
		--env MUJOCO_GL=egl \
		--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
		--volume $(HOME)/.Xauthority:/root/.Xauthority:rw \
		--volume $(DATA_DIR):/root/stretch_user:rw \
		--network host \
		--privileged \
		--shm-size=2g \
		$(IMAGE_NAME):$(IMAGE_TAG)
	@echo "Container started. Use 'make logs' to view logs or 'make shell' to attach."

shell:
	@echo "Opening shell in container..."
	docker exec -it $(CONTAINER_NAME) bash

stop:
	@echo "Stopping container..."
	docker stop $(CONTAINER_NAME) || true
	docker rm $(CONTAINER_NAME) || true

logs:
	docker logs -f $(CONTAINER_NAME)

clean:
	@echo "Removing container and image..."
	docker stop $(CONTAINER_NAME) || true
	docker rm $(CONTAINER_NAME) || true
	docker rmi $(IMAGE_NAME):$(IMAGE_TAG) || true
	@echo "Cleaned up!"

clean-all: clean
	@echo "Removing data directory..."
	@read -p "This will delete $(DATA_DIR). Are you sure? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		rm -rf $(DATA_DIR); \
		echo "Data directory removed."; \
	else \
		echo "Cancelled."; \
	fi

# Docker Compose commands
compose-up:
	@xhost +local:docker > /dev/null 2>&1 || true
	docker compose up

compose-up-detached:
	@xhost +local:docker > /dev/null 2>&1 || true
	docker compose up -d

compose-down:
	docker compose down

compose-logs:
	docker compose logs -f

