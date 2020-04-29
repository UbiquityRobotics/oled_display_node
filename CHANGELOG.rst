^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package oled_display_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
