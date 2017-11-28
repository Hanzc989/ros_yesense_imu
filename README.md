# Yesense

## Introduction

This is a ros driver for yesense imu sensor.

## Install Dependence

```shell
git clone https://github.com/wjwwood/serial.git
cd serial/
mkdir build
cmake ..
make
sudo make install
```

## Build

```
cd yesense_ws/
catkin_make
```

## Usage

```shell
roslaunch yesense_imu yesense.launch
```

**note: **change the params in launch to your own