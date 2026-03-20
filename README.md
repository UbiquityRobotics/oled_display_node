# OLED Display Output ROS 2 Node

This module implements a ROS 2 node that provides display updates to an OLED display connected via I2C.
The node initializes with system information (Hostname, IP, Battery, Motor status) and subscribes to a ROS topic for custom message updates.
Full line or line substring updates are supported.

## Requirements

- **Hardware**: 
    - Raspberry Pi 4 (or similar ARM-based SBC)
    - 1.3" I2C OLED Display (SH1106 controller preferred, SSD1306 also supported)
- **OS**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2**: Jazzy Jalisco

## Hardware Setup & Dependencies

### 1. Enable I2C on Raspberry Pi
The display communicates over I2C (typically on `/dev/i2c-1`). Ensure I2C is enabled:
```bash
sudo raspi-config
# Navigate to Interface Options -> I2C -> Enable
```
Or add `dtparam=i2c_arm=on` to `/boot/firmware/config.txt` and reboot.

### 2. Install System Dependencies
Install `i2c-tools` to verify the connection:
```bash
sudo apt update
sudo apt install i2c-tools
```
Verify the display is detected (typically at address `0x3c`):
```bash
i2cdetect -y 1
```

### 3. Permissions
Ensure your user is in the `i2c` group to access the device without sudo:
```bash
sudo usermod -aG i2c $USER
# Log out and back in for changes to take effect
```

## Installation

### 1. Create a Workspace
If you don't have one, create a colcon workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone and Install Dependencies
Clone this repository into `src`, then use `rosdep` to install ROS 2 dependencies:
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro jazzy -y
```

### 3. Build
```bash
colcon build --packages-select oled_display_node
source install/setup.bash
```

## Running the Node

### Launching the Node
Use the provided ROS 2 launch file:
```bash
ros2 launch oled_display_node display_launch.py
```
The node will start and periodically display:
- **Line 0**: Hostname
- **Line 1**: WLAN0 IP Address
- **Line 3**: ETH0 IP Address
- **Line 5**: Battery Voltage (if `battery_state` topic is active)
- **Line 7**: Motor Power & Firmware Version

### Parameters
- `update_delay_ms`: (int) Delay between line updates in milliseconds. Default: `250`.

### Subscriptions
- `display_node` (`oled_display_node/msg/DisplayOutput`): Listen for custom display updates.
- `battery_state` (`sensor_msgs/msg/BatteryState`): Updates battery voltage display.
- `motor_power_active` (`std_msgs/msg/Bool`): Updates motor status.
- `firmware_version` (`std_msgs/msg/String`): Updates firmware version display.

## Test or Example Script
A Python script is provided to test sending custom messages to the display:
```bash
ros2 run oled_display_node display_writer.py
```

## Qualified OLED Displays
The tested display is the 1.3" diagonal display using a **SH1106** chip.
- **Pinout**: Ground, VCC (3.3V), SCL, SDA.
- **Recommendation**: HiLetgo 1.3" IIC I2C Serial 128x64 SSH1106 OLED LCD Display.
- **Note**: 0.96" SSD1306 displays are also supported but may require adjustment if the pixel offset differs.


