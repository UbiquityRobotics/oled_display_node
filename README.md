
# Display Output ROS Node


This module implements a ROS node (process) that does Display Updates to an OLED display
The node does some initial messages and then will subscribe to an inbound ROS topic  and act on messages for update of lines on the display. 
Full line or line substring updates starting at a given X pixel on the given line are supported.

The tested display is the 1.3" diagonal display using a SH1106 chip controlled over the I2C bus.
We prefer the pinout with  Ground, Vcc, SCL then SDA lines for use with Ubiquity Robotics rev 5.1 main board.
These displays are easily found on EBay but be sure the controller chip is the SH1106.

The 1.3" OLED display using the SH1106 controller chip offers 8 lines of 15 characters 
The SH1106 is closely compatible with SSD1306 display that is typically the 0.96" OLED display.
The difference is mostly at initialization and then changing the pixel X offset for the SH1106 control chip with a RAM space of 132*64, while the SSD1306 pixel space is 128*64.

### ROS Paramaters

* There are currently no ROS parameters available.

### Publications

* The node does not publish messasges to any ROS topic

### Subscriptions

* `display_node` (msgs/DisplayOutput): Listen for display updates

### Manually Launching the Node

* roslaunch display_node display.launch

The node will start and then print the linux network name on top line and then the IP address will be on the 3rd line.

### Launching the Node From a ROS launch file

To launch the node from a ROS launch file add to the launch file these lines:

* <node pkg="display_node" type="display_node" name="display_node" output="screen">
* </node>

### Test or Example Script
A very simple python script that sends a few messasges to the display node has been supplied.
Use the python script 'display_writer.py` in the scripts folder to test the display.

* python ./scripts/display_writer.py


