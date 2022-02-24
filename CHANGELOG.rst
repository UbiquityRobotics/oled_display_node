^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package oled_display_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-02-24)
------------------
* Forcing display type to SH1106 because auto detection can fail on some displays and we specify the SH1106 for production so it is best to not auto-detect
* Oled fixes feb2022 - Battery charge and motor power on off shown (`#20 <https://github.com/UbiquityRobotics/oled_display_node/issues/20>`_)
* Fix IP addr logic for proper IP addr on display.  Show CHRG if battery above 27V.  Better detection of display type which before some displays could be initialized wrong and show text backwards
* Adding motor power on or off in display
* Now with motor power on or off indication
* Making display type detect more robust. Adding motor power on or off on display
* Add LICENSE file
* Add specific bit-field error codes.  Fix the clear display for the 1306 display so junk chars on last 4 pixels get cleared in clearDisplay.   Fix spam error for when display stops working after it had worked earlier.  We will exit at this time.  We are not going to try to retry and re-init the display at this time, too complex in possible different error cases. (`#11 <https://github.com/UbiquityRobotics/oled_display_node/issues/11>`_)
* Remove a battery voltage log spam message and make it a ROS_DEBUG.   Correct battery status display so if low or ok the text does not shift from right to left.
  Adding blinking LOW battery indication.
* Adding blinking LOW battery indication.
  I2C read/write oled_display_node calls with lock enforced by the kernel
* I2C general read/write functions.
* Adding autodetect of SH1106 or SSD1306 OLED displays with default to the SH1106 which should be the only display we source but just in case we can do either
* Respond to review inputs. Added exit if no display is detected
* Adding battery voltage readout
* Add showing battery voltage on display if the battery_state topic is picked up by this node.  Trying to more fully initialize OLED display to fix some bugs in it not working on all units perhaps from failed config after power up
* Adding back in input from a review that somehow got regressed.  Simple change
* Contributors: Mark Johnston, Rohan Agrawal, Teodor

1.0.0 (2020-04-28)
------------------
* Remove /* in comment that can cause warnings
  Bug: https://github.com/UbiquityRobotics/oled_display_node/issues/4
* Refresh the IP addr and machine name lines every 10 seconds.  This allows for a changed IP address due to networking changes to show up worse case in 10 seconds.  Any other lines the app outside of the oled display driver is concerned must be refreshed by sending messages to the display node over the input topic as this auto-refresh does not want to make assumptions about user lines in use on the display
* README.md
* Refactoring to become oled_display_node
* Added CMake install rules to allow for package release
  This should install the main executable, the launch files and the
  demo script.
* Cleanup of package and make and more
* trying to fix the section on starting from launch file
* Cleanup and enhance README to point out we intend this display node to support 1.3 inch OLED display using the SH1106 controller chip
* Initial push of OLED display node with README
* Contributors: Mark Johnston, Mark Johston, Rohan Agrawal
