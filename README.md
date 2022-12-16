# Swerve Sim Container

Dockerized gazebo simulation of a swerve drive robot.
---

## Dependencies

* docker
* make (`$ sudo apt-get build-essential` on ubuntu)
* x11 (to run gazebo application windows)
* (Optional) - [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian)

---

## Instructions to run

1. Pull or Build sim image.
```
# Pull image from github container repository
$ make pull

# Locally build image
$ make build
```

2. Launch sim
```
# Without Nvidia GPU
$ make launch

# With Nvidia GPU
$ make launch-gpu
```

3. (Optional) Launch teleop nodes
```
# Keyboard Teleop
$ make teleop-keyboard

# Joystick Teleop
$ make teleop-joy
```

---

## Additional Utilities
1. `make run` or `make run-gpu` - Run container with shell.
2. `make shell` - Attach shell to the running container.
3. `make stop` - Stop running container.
