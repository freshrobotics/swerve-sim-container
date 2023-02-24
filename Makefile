# Run 'make help' in the terminal to see a list of script options

SHELL := /bin/bash

PACKAGE=swerve-sim-container
VERSION:=0.1.0
CONTAINER:=ghcr.io/freshrobotics/${PACKAGE}:${VERSION}
TARFILE:=${PACKAGE}-${VERSION}.tar
WORKSPACE=/workspace

XAUTH := /tmp/.docker.xauth
XAUTH_STRIP := xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $(XAUTH) nmerge -
GPU_FLAGS := --gpus=all \
	--env="NVIDIA_DRIVER_CAPABILITIES=all"

DOCKER_RUN = docker run --rm -it \
  --network host \
	--privileged \
	--volume="/dev:/dev:rw" \
	--env="DISPLAY=$(DISPLAY)" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=$(XAUTH)" \
	--mount="type=bind,source="/tmp",target=/tmp:rw" \
	--name $(PACKAGE)

.PHONY: help
help: ## show help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' ${MAKEFILE_LIST} | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.PHONY: version
version: ## print the package version
	@echo $(VERSION)

.PHONY: run
run: ## start container with shell
	@xhost +local:root
	@$(XAUTH_STRIP)
	@$(DOCKER_RUN) \
	$(CONTAINER) \
		/bin/bash
	@xhost -local:root

.PHONY: run-gpu
run-gpu: ## runs sim in container for systems with NVIDIA GPU
	@xhost +local:root
	@$(XAUTH_STRIP)
	@$(DOCKER_RUN) \
	$(GPU_FLAGS) \
	$(CONTAINER) \
		/bin/bash
	@xhost -local:root

.PHONY: launch
launch: ## start container and launch ros bringup
	@xhost +local:root
	@$(XAUTH_STRIP)
	$(DOCKER_RUN) \
	$(CONTAINER) \
		/bin/bash -ic \
			"ros2 launch swerve_robot_gazebo sim.launch.py"
	@xhost -local:root

.PHONY: launch-gpu
launch-gpu: ## start container and launch ros bringup
	@xhost +local:root
	@$(XAUTH_STRIP)
	$(DOCKER_RUN) \
	$(GPU_FLAGS) \
	$(CONTAINER) \
		/bin/bash -ic \
			"ros2 launch swerve_robot_gazebo sim.launch.py"
	@xhost -local:root

.PHONY: teleop-keyboard
teleop-keyboard: ## get (another) shell with teleop twist keyboard
	docker exec -it ${PACKAGE} /bin/bash -ic \
		"ros2 run teleop_twist_keyboard teleop_twist_keyboard"

.PHONY: teleop-joy
teleop-joy:
	docker exec -it ${PACKAGE} /bin/bash -ic \
		"ros2 launch swerve_robot_gazebo teleop_joy.launch.py"

.PHONY: stop
stop: ## stops running container
	docker stop ${PACKAGE}

.PHONY: shell
shell: ## get (another) shell to running container
	docker exec -it ${PACKAGE} /bin/bash

.PHONY: pull
pull: ## pulls the docker image from cloud package repo
	docker pull $(CONTAINER)

.PHONY: build
build: ## builds the docker image
	docker build \
		--tag $(CONTAINER) \
		.
