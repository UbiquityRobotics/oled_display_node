
# OLED Display Output ROS Node


This module implements a ROS node (process) that does Display Updates to an OLED display connected on I2C.
The node does some initial messages and then will subscribe to an inbound ROS topic  and act on messages for update of lines on the display. 
Full line or line substring updates starting at a given X pixel on the given line are supported.

The tested display is the 1.3" diagonal display using a SH1106 chip controlled over the I2C bus.
We prefer the pinout with  Ground, Vcc, SCL then SDA lines for use with Ubiquity Robotics rev 5.x main boards.
These displays are easily found on EBay but be sure the controller chip is the SH1106 and that they will work with a 3.3V supply and 3.3V I2C control.

The 1.3" OLED display using the SH1106 controller chip offers 8 lines of 15 characters 
The SH1106 is closely compatible with SSD1306 display that is typically the 0.96" OLED display.
The difference is mostly at initialization and then changing the pixel X offset for the SH1106 control chip with a RAM space of 132*64, while the SSD1306 pixel space is 128*64.

### ROS Paramaters

* There are currently no ROS parameters available.

### Publications

* The node does not publish messasges to any ROS topic

### Subscriptions

* `oled_display_node` (msgs/DisplayOutput): Listen for display updates

### The Qualified OLED Display we have Tested 

There are several displays that are 1.3" displays using the SH1106 controller that will allow powering from 3.3V and also allow a 3.3V I2C bus.  The pinout MUST be from left to right:   Ground, 3.3V (Vcc), SCL and SDA.
Sometimes this display is incorrectly stating the controller is a SSH1106 display but it is a point of confusion and is the SH1106.    We have identified a specific vendor as well.   You may purchase this display on amazon.com and look for this text in the title 'HiLetgo 1.3" IIC I2C Serial 128x64 SSH1106 OLED LCD Display LCD Module'.  there are cheaper solutions but only by a few dollars so we wish to only stand behind this display officially.  We claim support for this display on the rev 5.1 motor controller board and this display will plug right in to our female connector on the rev 5.1 board.   The Rev 5.0 controller used a male pin jack so you would have to modify your display with a female socket to use on a rev 5.0 board. 

### Manually Launching the Node

* roslaunch oled_display_node display.launch

The node will start and then print the linux network name on top line and then the IP address will be on the 3rd line.

### Launching the Node From a ROS launch file

To launch the node from a ROS launch file add to the launch file the 2 lines shown below.
As an example you can add these two line to an Ubiquity Robotics Magni host processor 
in the file /opt/ros/kinetic/share/magni_bringup/launch/core.launch although we plan on configurable support in early 2020.

    <node pkg="oled_display_node" type="oled_display_node" name="oled_display_node" output="screen">
    </node>

### Test or Example Script
A very simple python script that sends a few messasges to the display node has been supplied.
Use the python script 'display_writer.py` in the scripts folder to test the display.

* python ./scripts/display_writer.py


