
# Display Output ROS Node


This module implements a ROS node (process) that does Display Updates to an OLED display
The node does some initial messages and then will subscribe to an inbound ROS topic  and act on messages for update of lines on the display. 
Full line or line substring updates starting at a given X pixel on the given line are supported.

The default display is a 1.3" diagonal display using a SH1106 chip and controlled over the I2C bus.
The OLED display uses either the SSG1106 or the SSH1106 controller chip for 8 lines of 15 characters 
The SSG1106 is closely compatible with SSD1306 display, the difference is mostly at initialization and then changing the pixel X offset for the SSH1106 control chip with a RAM space of 132*64, while the SSD1306 pixel space is 128*64.

### ROS Paramaters

* There are currently no ROS parameters available.

### Publications

* The node does not publish messasges to any ROS topic

### Subscriptions

* `display_node` (msgs/DisplayOutput): Listen for display updates

### Launching the Node

* roslaunch display_node display.launch

The node will start and then print the linux network name on top line and then the IP address will be on the 3rd line.

### Test or Example Script
A very simple python script that sends a few messasges to the display node has been supplied.
Use the python script 'display_writer.py` in the scripts folder to test the display.

* python ./scripts/display_writer.py


