#!/usr/bin/env python3

"""
Copyright (c) 2019, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of display_node nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
Example client program for writing to display using ROS 2 topic
"""

import rclpy
from rclpy.node import Node
import time

# our custom messages for the commands we will follow
from oled_display_node.msg import DisplayOutput


class Controller(Node):
    """
    Constructor for our class
    """
    def __init__(self):
       super().__init__('controller')

       # A publisher for sending commands to the follower node
       self.displayCmdPub = self.create_publisher(DisplayOutput, "/display_node", 10)

    def publishDisplayCommand(self, row, column, text, comment):
       numChars = len(text)
       if numChars > 15:
         numChars = 15

       dispMsg = DisplayOutput()
       dispMsg.action_type    = 2               # Just use MSG_DISPLAY_SUBSTRING
       dispMsg.row           = row             # Display row with 0 as top row
       dispMsg.column        = column          # Display column in pixels
       dispMsg.num_chars      = numChars        # Number of chars to be written
       dispMsg.attributes    = 0               # Just write with no attributes
       dispMsg.text          = text            # The text to be written
       dispMsg.comment       = comment         # Comment in the message, no functional use
       self.displayCmdPub.publish(dispMsg)

    def run(self):
       self.get_logger().info("ROS 2 publisher publishing commands to display topic")

       time.sleep(1.0)
       self.publishDisplayCommand(3, 2, "Data for the", "Write to a line")
       time.sleep(1.0)
       self.publishDisplayCommand(4, 2, "Application ", "Write to a line")
       time.sleep(1.0)
       self.publishDisplayCommand(5, 2, "will now be ", "Write to a line")
       time.sleep(1.0)
       self.publishDisplayCommand(6, 2, "written to  ", "Write to a line")
       time.sleep(1.0)
       self.publishDisplayCommand(7, 2, "OLED display", "Write to a line")
       time.sleep(1.0)
       self.get_logger().info("Commands sent")


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    node.run()
    # No need to spin since run() finishes and we exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
