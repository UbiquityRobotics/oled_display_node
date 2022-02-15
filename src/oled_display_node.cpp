/*
 * Copyright (c) 2019, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

// This ensures that the code is ran on Raspberry Pi architecture
#ifdef __x86_64__

#include <stdio.h>

int main(int argc, char** argv) {
    (void)fprintf(stderr, "The oled_display_node for the x86/64 is a fake!\n");
    return 1;
}

#endif // __x86_64__

#if defined(__arm__) || defined(__aarch64__)

/*
 * Display Output Subsystem
 *
 * This module implements a ROS node (process) that does Display Updates to an LCD display
 * Accept messages from a ROS Topic (message queue) and update the display.
 * Full update or substring updates starting at a given line and row are supported.
 */

#define  THIS_NODE_NAME    "oled_display_node"        // The Name for this ROS node. Used in ROS_INFO and more

// Default lines to output text to from this module
#define  DISP_LINE_HOSTNAME    0
#define  DISP_LINE_IP_ADDR     1
#define  DISP_LINE_BATT_VOLTS  3
#define  DISP_LINE_MOTOR_POWER 5 

// We are putting a battery low blinking feature to warn user of very low battery
// Our goal is to set a lower threshold than this (default 22.5V) in motor node to stop motor control
// once that point is hit
#define  BAT_LOW_LEVEL  23.5                     // Start to warn of low battery voltage with blinking display

// Define a level where we think the charger is plugged in at this time
#define  BAT_CHARGING_LEVEL  27.0                // Indicate battery is on charger for high voltages

// Type and I2C address of the display
// The small 1.3" OLED displays specified for production use the SH1106 controller chip
// The 0.96" OLED display tends to use the SSD1306 controller.
// The SH1106 is fairly compatible with SSD1306 basic, the difference is that the SH1106 control chip
// RAM space is 132*64, while SSD1306 space is 128*64.
// This code supports the 1.3" OLED with the SH1106 controller chip by default
// So besides different initialization bytes the display starts 2 bytes later in one vs the other
#define OLED_DISPLAY_TYPE  DISPLAY_TYPE_SH1106     // Or use DISPLAY_TYPE_AUTO although that sometimes fails
#define OLED_DISPLAY_ADDR  SH1106_OLED_I2C_ADDRESS
#define OLED_I2C_DEVICE    "/dev/i2c-1"          // SYSTEM SPECIFIC

// High level API defines to better document calls
#define DISP_WRITE_TEXT_CENTER      1
#define DISP_WRITE_TEXT_LEFT        0
#define DISP_TEXT_START_MODE    DISP_WRITE_TEXT_CENTER     // The default mode used for common writes


/************************************************************************************
 * The ROS Code usage requires the following comments be pressent:
 *
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

// These next few are for I2C and ioctls and file opens
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>

// Includes specific to the display driver and font
#include <oled_display_node/oled_display.h>
#include <oled_display_node/font8x8_basic.h>

#include <oled_display_node/DisplayOutput.h>
#include <sensor_msgs/BatteryState.h>

// Some limited state for display
double g_batteryVoltage = 0.0;
int32_t g_motorPowerActive = -1;

// Display context
dispCtx_t g_oledDisplayCtx;

// External Defs which we keep hidden in this node
extern  int dispOled_detectDisplayType(std::string devName, uint8_t i2cAddr);
extern  int dispOled_initCtx(std::string devName, dispCtx_t *dispCtx, int dispType, uint8_t i2cAddr);
extern  int dispOled_init(std::string devName, dispCtx_t *dispCtx, int dispType, uint8_t i2cAddr);
extern  int dispOled_clearDisplay(dispCtx_t *dispCtx);
extern  int dispOled_setCursor(dispCtx_t *dispCtx, int column, int line);
extern  int dispOled_writeText(dispCtx_t *dispCtx, uint8_t line, uint8_t segment,
                uint8_t center, const char *textStr);

/*
 * @name    getPopen
 * @brief   Converts the stream returned by the stdio function `popen` into a string.
 *
 * @param   input: the command to be executed by `popen`
 *
 * @return  Returns popen output in string format
 *
 * @note    This routine only uses the readable stream returned by `popen`
 */
std::string getPopen(std::string input) {
    std::string result("");;
    char c;
    FILE* fp;

    try {   // Catch any bad operations with this trap

    fp = popen(input.c_str(), "r");
    if (fp != NULL) {
        do {
            c = fgetc(fp);
            if (isascii(c)) {
                result += c;
            }
        } while (!feof(fp));
        pclose(fp);
    }

    } catch (...) {  }

    return result;
}

// getMainIpAddress()
// We will use any eth0 IP address that has 192.168.1 in it unless that is not found in which case use wlan0 IP
// For some systems with Lidar the ethernet IP is 192.168.42.x  so in that case wlan0 will be used rather than hard lan IP.
// One disadvantage is on older images we did not rename the ethernet to enet0 so we will not get those plugged in
// network cables but this is a robot and plugged in lan is not the typical usage, wlan0 is 'normal'
//
void  getMainIpAddress(std::string &ipAddress, int logFindings) {
  // ip -o -4 addr returns:  2: eth0    inet 192.168.1.164/24 brd 192.168.1.255 scope global eth0\       valid_lft forever 
  std::string  enetIpAddress = getPopen("ip -o -4 addr | grep eth0  | awk '{print $4}'");  // IP with /24 mask bits at end
  std::string  wlanIpAddress = getPopen("ip -o -4 addr | grep wlan0 | awk '{print $4}'");  // IP with /24 mask bits at end
  std::string  mainIpAddress("notfound");

  if (enetIpAddress.length() > 8) {
      mainIpAddress.assign(enetIpAddress.substr(0,enetIpAddress.find('/')));
      if (logFindings != 0) {
          ROS_INFO("%s The eth0 IP will be the displayed IP of %s", THIS_NODE_NAME, mainIpAddress.c_str());
      }
  } else {
      mainIpAddress.assign(wlanIpAddress.substr(0,wlanIpAddress.find('/')));
      if (logFindings != 0) {
          ROS_INFO("%s The wlan0 IP will be the displayed IP of %s", THIS_NODE_NAME, mainIpAddress.c_str());
      }
  }
  ipAddress.assign(mainIpAddress);
  return;
}

//  These commands are sent to initialize SSD1306.
//  There is not 'internal chip register' for this device and instead it uses a funky protocol.
//  A control byte is followed by a single byte or multiple bytes depending on the control byte.
//  The chip interprits these 'packets' as command(s) or data byte(s) based on the leading control byte
#define SSD1306_INIT_BYTE_COUNT         6
static uint8_t ssd1306_init_bytes[SSD1306_INIT_BYTE_COUNT] = {
        OLED_CONTROL_BYTE_CMD_STREAM,
                OLED_CMD_SET_CHARGE_PUMP,       0x14,
                OLED_CMD_SET_SEGMENT_REMAP,
                OLED_CMD_SET_COM_SCAN_MODE,
                OLED_CMD_DISPLAY_ON
};

#define SH1106_INIT_BYTE_COUNT  17
static uint8_t sh1106_init_bytes[SH1106_INIT_BYTE_COUNT] = {
        OLED_CONTROL_BYTE_CMD_STREAM,
                0x30,                           // Charge pump default
                0x40,                           // RAM display line of 0
                OLED_CMD_DISPLAY_OFF,
                OLED_CMD_SET_SEGMENT_REMAP,
                OLED_CMD_SET_COM_SCAN_MODE,
                OLED_CMD_SET_DISPLAY_OFFSET, 0, // Sets mapping of display start line
                OLED_CMD_DC_DC_CTRL_MODE, 0x8B, // Must have DISPLAY_OFF and follow tih 0x8B
                0x81, 0x80,         // Display contrast set to second byte
                OLED_CMD_DISPLAY_RAM,
                OLED_CMD_DISPLAY_NORMAL,
                OLED_CMD_SET_MUX_RATIO, 0x3F,   // Init multiplex ration to standard value
                OLED_CMD_DISPLAY_ON
};

/*
 * @name                i2c_read
 * @brief               Read one or more bytes from an I2C based device
 *
 * @param               i2cDevFile      Name of the I2C device
 * @param               i2c7bitAddr     7-bit I2C bus address
 * @param               pBuffer         User 8-bit buffer for data return
 * @param               numBytes        Number of bytes to be read from the chip
 * @param               chipRegAddr     Address register within chip. Set chipRegAddr to true if specified
 * @param               chipRegAddrFlag Set to false to suppress writing to slave internal address
 *
 * @return              Returns number of bytes read where 0 or less implies some form of failure
 */
static int i2c_read(const char *i2cDevFile, uint8_t i2c7bitAddr,
                    uint8_t *pBuffer, int numBytes, uint8_t chipRegAddr, bool chipRegAddrFlag)
{
    int       fd;                                     	  // File descriptor
    const int slaveAddress = i2c7bitAddr;             	  // Address of the I2C device
    int       retCode   = 0;
    int       bytesRead = 0;
    struct    i2c_msg msgs[2];                    	  // Low level representation of one segment of an I2C transaction
    struct    i2c_rdwr_ioctl_data msgset[1];	 	  // Set of transaction segments

    if ((fd = open(i2cDevFile, O_RDWR)) < 0) {   	  // Open port for reading and writing
      ROS_ERROR("Cannot open I2C def of %s with error %s", i2cDevFile, strerror(errno));
      retCode = IO_ERR_DEV_OPEN_FAILED;
      goto exitWithNoClose;
    }

    if (chipRegAddrFlag) {
        msgs[0].addr = slaveAddress;
        msgs[0].flags = 0;                       	  // Write bit
        msgs[0].len = 1;                         	  // Slave Address/byte written to I2C slave address
        msgs[0].buf = &chipRegAddr;              	  // Internal Chip Register Address
        msgs[1].addr = slaveAddress;
        msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;	  // Read bit or Combined transaction bit
        msgs[1].len = numBytes;                  	  // Number of bytes read
        msgs[1].buf = pBuffer;                   	  // Output read buffer

        msgset[0].msgs = msgs;
        msgset[0].nmsgs = 2;                     	  // number of transaction segments (write and read)

        // The ioctl here will execute I2C transaction with kernel enforced lock
        if (ioctl(fd, I2C_RDWR, &msgset) < 0) {
            ROS_ERROR("Failed to get bus access to I2C device %s!  ERROR: %s", i2cDevFile, strerror(errno));
            retCode = IO_ERR_IOCTL_ADDR_SET;
            goto exitWithFileClose;
        }
    }
    else {
	// The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
    	if (ioctl(fd, I2C_SLAVE, slaveAddress) != 0) {    // Set the port options and addr of the dev
      	  ROS_ERROR("Failed to get bus access to I2C device %s!  ERROR: %s", i2cDevFile, strerror(errno));
      	  retCode = IO_ERR_IOCTL_ADDR_SET;
          goto exitWithFileClose;
    	}

        bytesRead = read(fd, pBuffer, numBytes);
        if (bytesRead != numBytes) {             	  // Verify that the number of bytes we requested were read
          ROS_ERROR("Failed to read from I2C device %s!  ERROR: %s", i2cDevFile, strerror(errno));
          retCode = IO_ERR_READ_FAILED;
          goto exitWithFileClose;
        }
    }
    exitWithFileClose:
    close(fd);

    exitWithNoClose:

    // Read is odd in that + is num bytes read so make errors negative
    if (retCode == 0) {
        retCode = numBytes;
    } else {
        retCode = retCode * -1;
    }

    return retCode;
}

/*
 * @name                i2c_write
 * @brief               Write one or more bytes to the I2C based device
 *
 * @param               i2cDevFile       Name of the I2C Device
 * @param               i2c7bitAddr      7-bit I2C bus address
 * @param               pBuffer          User 8-bit buffer for data input
 * @param               numBytes         Number of bytes to be written to the chip
 *
 * @return              Returns 0 for ok. Non-zero are bit-encoded failures
 */

static int i2c_write(const char *i2cDevFile, uint8_t i2c7bitAddr, uint8_t *pBuffer, int numBytes)
{
    int        fd;                 		// File descriptor
    int        retCode = 0;
    const int  slaveAddress = i2c7bitAddr;      // Address of the I2C device

    // Open port for writing
    if ((fd = open(i2cDevFile, O_WRONLY)) < 0) {
      ROS_ERROR_ONCE("Cannot open I2C def of %s with error %s", i2cDevFile, strerror(errno));
      retCode = IO_ERR_DEV_OPEN_FAILED;
      goto exitWithNoClose;
    }

    // The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
    if (ioctl(fd, I2C_SLAVE, slaveAddress) != 0) {  // Set the port options and addr of the dev
      ROS_ERROR_ONCE("Failed to get bus access to I2C device %s!  ERROR: %s", i2cDevFile, strerror(errno));
      retCode = IO_ERR_IOCTL_ADDR_SET;
      goto exitWithFileClose;
    }

    if (write(fd, pBuffer, numBytes) != numBytes) {
        ROS_ERROR_ONCE("Failed to write to I2C device %s!  ERROR: %s", i2cDevFile, strerror(errno));
        retCode = IO_ERR_WRITE_FAILED;
        goto exitWithFileClose;
    }

    exitWithFileClose:
    close(fd);

    exitWithNoClose:

    return retCode;
}

/*
 * @name                dispOled_detectDisplayType
 * @brief               Determine display type or if it is on the I2C bus
 *
 * @param               devName         Name of the I2C output device
 * @param               i2c7bitAddr     7-bit I2C bus address
 *
 * @return              dispType        DISPLAY_TYPE_SSD1306,DISPLAY_TYPE_SH1106
 * @return              Returns DISPLAY_TYPE_SH1106 [DEFAULT] or DISPLAY_TYPE_SSD1306
 *                      Returns DISPLAY_TYPE_NONE for I2C error due to no device detected
 */

#define MX_DEV_NAME_LEN 32
int dispOled_detectDisplayType(std::string devName, uint8_t i2c7bitAddr, int *dispType)
{
    uint8_t buf[16];
    int     retCode = 0;
    int     retCount = 0;
    int     i = 0;
    int     vote1106 = 0;
    int     vote1306 = 0;
    char    device[MX_DEV_NAME_LEN];
    strncpy(&device[0], devName.c_str(), MX_DEV_NAME_LEN);
    device[(MX_DEV_NAME_LEN-1)] = 0;     // protect against long dev names

    // We have seen incorrect values read sometimes and because this chip relies on
    // a status register in the SH1106 we better read a few times and 'vote'
    for (i=0 ; i < 5 ; i++) {
        // Read the status register at chip addr 0 to decide on chip type - set flag to true
        retCount = i2c_read(&device[0], i2c7bitAddr, &buf[0], 1, 0x00, true);
        if (retCount < 0) {
            ROS_ERROR("Error 0x%x in reading OLED status register at 7bit I2CAddr 0x%x",
                retCount, i2c7bitAddr);
            *dispType = DISPLAY_TYPE_NONE;
            retCode = IO_ERR_READ_FAILED;
        } else if (retCount != 1) {
            ROS_ERROR("Cannot read byte from OLED status register at 7bit Addr 0x%x",
            i2c7bitAddr);
            *dispType = DISPLAY_TYPE_NONE;
            retCode = IO_ERR_READ_LENGTH;;
        } else {
            ROS_INFO("Read OLED status register as 0x%02x on pass %d", buf[0],i);
            if ((buf[0] & 0x07) == 0x06) {
                // We found lower 3 bit as a 6 but datasheet does not spec it
                vote1306++;
            } else {
                // Data sheet guarentees lower 3 bits as 0
                // We are going to vote by assumption that this is a SSD1306
                vote1106++;
            }
        }
        usleep(30000);
    }

    // count the votes and set display type
    if (vote1106 > vote1306) {
        *dispType = DISPLAY_TYPE_SH1106;
    } else {
        *dispType = DISPLAY_TYPE_SSD1306;
    }

    return retCode;
}

/*
 * @name                dispOled_initCtx
 * @brief               Initialize a display context
 *
 * @param               devName         Name of the I2C output device
 * @param               dispCtx         Context for display that this command will populate
 * @param               dispType        Type of display. DISPLAY_TYPE_SSD1306 or DISPLAY_TYPE_SH1106
 * @param               i2cAddr         7-bit I2C bus address
 *
 * @return              Returns 0 for ok or -1 for IO error
 */
int dispOled_initCtx(std::string devName, dispCtx_t *dispCtx, int dispType, uint8_t i2cAddr)
{
    int retCode = 0;

    dispCtx->dispType = dispType;
    dispCtx->i2cAddr = i2cAddr;
    strcpy(&dispCtx->devName[0], devName.c_str());

    // Set max lines and horizontal segments for the display in use
    switch (dispType) {
    case DISPLAY_TYPE_SSD1306:
        dispCtx->maxLine      = SSD1306_MAX_LINE;
        dispCtx->maxColumn    = SSD1306_MAX_COLUMN;
        dispCtx->maxVertPixel = SSD1306_MAX_VERT_PIXEL;
        dispCtx->maxHorzPixel = SSD1306_MAX_HORZ_PIXEL;
        dispCtx->horzOffset   = SSD1306_HORZ_OFFSET;
        dispCtx->endHorzPixel = SSD1306_END_HORZ_PIXEL;
        break;
    case DISPLAY_TYPE_SH1106:
        dispCtx->maxLine      = SH1106_MAX_LINE;
        dispCtx->maxColumn    = SH1106_MAX_COLUMN;
        dispCtx->maxVertPixel = SH1106_MAX_VERT_PIXEL;
        dispCtx->maxHorzPixel = SH1106_MAX_HORZ_PIXEL;
        dispCtx->horzOffset   = SH1106_HORZ_OFFSET;
        dispCtx->endHorzPixel = SH1106_END_HORZ_PIXEL;
        break;
    default:
        ROS_ERROR("%s: Unsupported display type of %d\n", THIS_NODE_NAME, dispType);
        retCode = IO_ERR_BAD_DISP_CONTEXT;
        break;
    }

    return 0;
}

/*
 * @name                dispOled_init
 * @brief               Initialize the small OLED display
 *
 * @param               devName         Name of the I2C output device
 * @param               dispCtx         Context for display that this command will populate
 * @param               displayType     Type of display. DISPLAY_TYPE_SSD1306 or DISPLAY_TYPE_SH1106
 * @param               i2cAddr         7-bit I2C bus address
 *
 * @return              Returns 0 for ok.  Non-zero indicates a fault
 */
int dispOled_init(std::string devName, dispCtx_t *dispCtx, int displayType, uint8_t i2cAddr)
{
    int retCode = 0;
    int dispType = displayType;

    // If this is called with NONE we try to autodetect the display
    if (dispType == DISPLAY_TYPE_AUTO) {
        // Auto-detect display. Detects if display present and type of OLED display
        retCode = dispOled_detectDisplayType(devName, i2cAddr, &dispType);
        if (retCode != 0) {
            return retCode;
        }
    }

    dispOled_initCtx(devName, dispCtx, dispType, i2cAddr);

    //Send all the commands to fully initialize the device.
    switch (dispCtx->dispType) {
    case DISPLAY_TYPE_SSD1306:
        // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
        ROS_INFO("%s Setup for SSD1306 controller on the OLED display.", THIS_NODE_NAME);
        retCode |= i2c_write(&dispCtx->devName[0], dispCtx->i2cAddr, &ssd1306_init_bytes[0], SSD1306_INIT_BYTE_COUNT);
        break;
    case DISPLAY_TYPE_SH1106:
        // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
        ROS_INFO("%s Setup for SH1106 controller on the OLED display.", THIS_NODE_NAME);
        retCode |= i2c_write(&dispCtx->devName[0], dispCtx->i2cAddr, &sh1106_init_bytes[0], SH1106_INIT_BYTE_COUNT);
        break;
    default:
        retCode = IO_ERR_BAD_DISP_CONTEXT;
        break;
    }
    if (retCode != 0) {
        ROS_ERROR_ONCE("%s: Setup for OLED display failed with error 0x%04x", THIS_NODE_NAME, retCode);
    }

    return retCode;
}

/*
 * @name                dispOled_setCursor
 * @brief               Move cursor to a given horizontal column and line
 *
 * @param               dispCtx         Context for display that holds type and hardware interface info
 * @param               column          The pixel resolution column from 0 to 127
 * @param               line            The line number where 0 is top line
 *
 * @return              Returns 0 for ok or -1 for IO error
 */
int dispOled_setCursor(dispCtx_t *dispCtx, int column, int line) {
        int retCode = 0;
        uint8_t curserSetup[8];

        switch (dispCtx->dispType) {
        case DISPLAY_TYPE_SSD1306:
                curserSetup[0] = OLED_CONTROL_BYTE_CMD_STREAM;
                curserSetup[1] = OLED_CMD_SET_COLUMN_RANGE;
                curserSetup[2] = column;            // Start of printing from left seg as 0
                curserSetup[3] = dispCtx->maxHorzPixel;     // last index of printing segments
                curserSetup[4] = OLED_CMD_SET_PAGE_RANGE;
                curserSetup[5] = line;                      // We assume only one line written to at a time
                curserSetup[6] = line;                      // We assume only one line written to at a time

                // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
                retCode |= i2c_write(&dispCtx->devName[0], dispCtx->i2cAddr, &curserSetup[0], 7);

        case DISPLAY_TYPE_SH1106:
                // SH1106 has different addressing than SSD1306
                curserSetup[0] = OLED_CONTROL_BYTE_CMD_STREAM;
                curserSetup[1] = 0xB0 | (line & 0xf);
                curserSetup[2] = 0x10 | (((column + dispCtx->horzOffset) & 0xf0) >> 4);  // Upper column address
                curserSetup[3] = 0x00 | ((column + dispCtx->horzOffset)  & 0xf);         // Lower column address

                // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
                retCode |= i2c_write(&dispCtx->devName[0], dispCtx->i2cAddr, &curserSetup[0], 4);
                break;
        default:
                break;
        }

        return retCode;
}

/*
 * @name                dispOled_clearDisplay
 * @brief               Clear the display
 *
 * @param               dispCtx         Context for display that holds type and hardware interface info
 *
 * @return              Returns 0 for ok or -1 for IO error
 */
int dispOled_clearDisplay(dispCtx_t *dispCtx) {
        int retCode = 0;

        uint8_t zero[140];
        zero[0] = OLED_CONTROL_BYTE_DATA_STREAM;;
        for (uint8_t idx = 1; idx < 136; idx++) {
                zero[idx] = 0;      // All 0 is blank vertical segments of 8 bits all across the row
        }
        for (uint8_t line = 0; line <= dispCtx->maxLine; line++) {

                retCode |= dispOled_setCursor(dispCtx, 0, line);
                if (retCode != 0) {
                        return retCode;
                }

                // Clear one line
                retCode |= i2c_write(&dispCtx->devName[0], dispCtx->i2cAddr, &zero[0], 
                    (dispCtx->maxHorzPixel+dispCtx->horzOffset)+dispCtx->endHorzPixel);
                if (retCode != 0) {
                        return retCode;
                }
        }

        return retCode;
}

/*
 * @name                dispOled_writeText
 * @brief               Send an ASCII text string to the display on a single line. No line feed support.
 *
 * @param               dispCtx         Context for display that holds type and hardware interface info
 * @param               line            Line number for start of the write. 0 is top line of display, 7 bottom
 * @param               segment         Where the text starts in terms of pixel count from the left being 0
 * @param               center      Set to non-zero to center the text on the given line
 * @param               textStr     The text string to be printed
 *
 * @return              Returns 0 for ok or -1 for IO error
 */
int dispOled_writeText(dispCtx_t *dispCtx, uint8_t line, uint8_t segment, uint8_t center, const char *textStr) {
    int           retCode = 0;
    const char    *text = textStr;
    uint8_t       text_len = strlen(text);
    uint8_t       dispData[DISPLAY_MAX_HORZ_PIXEL];
    int           dispDataIdx = 1;

    dispData[0] = OLED_CONTROL_BYTE_DATA_STREAM;     // Data starts on byte after this 1st one

        if (line > 7) {
                return -1;              // out of range line
        }
        if ((segment + (text_len * 8)) > dispCtx->maxHorzPixel) {
                return -2;              // out of range starting segment to end of the print with 8x8 font
        }
        int startSegment = segment;          // MOD: When MAX_SEGMENT was 0x7F this was just    segment & MAX_SEGMENT
        if (segment > (dispCtx->maxHorzPixel-1)) {
                segment = dispCtx->maxHorzPixel-1; // Cap this to max end of line segment
        }
        if (center != 0) {
                startSegment = (dispCtx->maxHorzPixel - (text_len * DISPLAY_CHAR_WIDTH)) / 2;
        }

        retCode |= dispOled_setCursor(dispCtx, startSegment, line);
        if (retCode != 0) {
                return retCode;
        }

        // Form pixels to send as data by lookup in font table
        for (uint8_t i = 0; i < text_len; i++) {
                // For each column of pixels for this char send a data byte which is one vertical column of pixels
                for (uint8_t charCol = 0; charCol < DISPLAY_CHAR_WIDTH; charCol++) {
                        dispData[dispDataIdx++] = font8x8_basic_tr[(int)(text[i])][charCol];
                }
        }

        // Write the data to display
	retCode |= i2c_write(&dispCtx->devName[0], dispCtx->i2cAddr, &dispData[0], dispDataIdx);

        return retCode;
}

// ==============================================  END OLED DISPLAY API CALLS =================================

// Optional helper to lock a semaphore if system requires a lock
int ipc_sem_lock(int semLock) {
    // THIS IS A STUB
    return 0;
}
// Optional helper to unlock a semaphore if system requires a lock
int ipc_sem_unlock(int semLock) {
    // THIS IS A STUB
    return 0;
}


/*
 * Update display hardware specific to our platform
 *
 * Use of one of 2 types of a small 7 row, 12 char per row OLED display
 *
 * Set row and column to non-zero to position cursor prior to text output
 *
 * Return of 0 is ok, negative values are failure cases
 * If the semLock is supplied we lock a semaphore for the update
 */
int  displayUpdate(std::string text, int attributes, int row, int column, int numChars, int semLock)
{
  int  messageLength = text.length();
  bool dbgPrint = false;

  if (numChars > 0) {
    messageLength = numChars;
  }

  ROS_INFO("%s: Write '%s' to row %d and column %d\n", THIS_NODE_NAME, text.c_str(), row, column);

  dispCtx_t oledDispCtx;
  char charBuf[120];
  int segment = column * DISPLAY_CHAR_WIDTH;
  int dispRow  = row;   // Bypass the system info area
  int maxChars = oledDispCtx.maxColumn - 1;
  int lineChars = messageLength;

  // We only initialize display context and do not re-initialize actual display
  dispOled_initCtx(OLED_I2C_DEVICE, &oledDispCtx, g_oledDisplayCtx.dispType, OLED_DISPLAY_ADDR);

  // Would be nice to account for non-zero cursor due to cursor control sometime too ...
  if (messageLength > oledDispCtx.maxColumn) {
    ROS_ERROR("%s: Unsupported character count of %d\n", THIS_NODE_NAME, messageLength);
    return -9;
  }

  if (lineChars > maxChars) lineChars = maxChars;    // Hard truncate max char count for this display
  strncpy(&charBuf[0], text.c_str(), lineChars);
  charBuf[lineChars] = 0;

  // Write out the characters to the display
  dispOled_writeText(&oledDispCtx, dispRow, segment, DISP_TEXT_START_MODE, &charBuf[0]);

  return 0;
}

// Driver to set the power on default message for the display
int displaySetStartupString(int line, std::string text, int semLock)
{
  std::string module = "displaySetStartupString";
  int  messageLength = text.length();
  bool dbgPrint = false;

  ROS_ERROR("%s: OLED display startup string not yet supported, ignore\n", THIS_NODE_NAME);
  return 0;
}

// Driver to set the LCD backlight brightness
int displaySetBrightness(int brightness, int semLock)
{
  std::string module = "displaySetBrightness";
  bool dbgPrint = false;

  ROS_ERROR("%s: OLED display brightness setting not supported, iignore\n", THIS_NODE_NAME);
  return 0;
}

/**
 * Receive messages for display output
 */
void displayApiCallback(const oled_display_node::DisplayOutput::ConstPtr& msg)
{
  ROS_INFO("%s heard display output msg: of actionType %d row %d column %d numChars %d attr 0x%x text %s comment %s]",
                THIS_NODE_NAME, msg->actionType, msg->row, msg->column, msg->numChars, msg->attributes,
                msg->text.c_str(), msg->comment.c_str());

  int i2cSemLockId = -9;

   // Now send data to the display

   switch (msg->actionType) {
     case oled_display_node::DisplayOutput::DISPLAY_STARTUP_STRING:
       displaySetStartupString(msg->row, msg->text.c_str(), i2cSemLockId);
       break;
     case oled_display_node::DisplayOutput::DISPLAY_SET_BRIGHTNESS:
       displaySetBrightness(msg->attributes, i2cSemLockId);
       break;
     case oled_display_node::DisplayOutput::DISPLAY_ALL:
     case oled_display_node::DisplayOutput::DISPLAY_SUBSTRING:
       displayUpdate(msg->text.c_str(), msg->attributes, msg->row, msg->column, msg->numChars, i2cSemLockId);
       break;
     default:
       break;
   }
}


/**
 * Receive messages for battery state
 */
void batteryStateApiCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{

  g_batteryVoltage = msg->voltage;

}

/**
 * Receive messages for battery state
 */
void motorPowerActiveApiCallback(const std_msgs::Bool::ConstPtr& msg)
{
  int32_t newPowerState;
  if (msg->data) {
    newPowerState = 1;
  } else {
    newPowerState = 0;
  }
  if (newPowerState != g_motorPowerActive) {
    ROS_INFO("%s Motor power active went from %d to %d", THIS_NODE_NAME, g_motorPowerActive, newPowerState);
  }

  g_motorPowerActive = newPowerState;
  return;
}





int main(int argc, char **argv)
{
  double updateDelay = 0.25;

  // The ros::init() function initializes ROS and needs to see argc and argv
  ros::init(argc, argv, THIS_NODE_NAME);

  // Setup a NodeHandle for the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  ROS_INFO("%s OLED Display node starting.", THIS_NODE_NAME);

  // Get our hostname on the network
  std::string hostname = getPopen("uname -n");

  // Here we will fetch the current IP address but we assume this node does not start till it is valid
  std::string displayedIpAddress;
  getMainIpAddress(displayedIpAddress, 1);

  char dispBuf[32];
  int  dispError = 0;

  ROS_INFO("%s Initialize OLED display.", THIS_NODE_NAME);
  dispError = dispOled_init(OLED_I2C_DEVICE, &g_oledDisplayCtx, OLED_DISPLAY_TYPE, OLED_DISPLAY_ADDR);

  if (dispError == 0) {
      dispOled_clearDisplay(&g_oledDisplayCtx);
      ros::Duration(1.0).sleep();
      dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_HOSTNAME, 0, DISP_TEXT_START_MODE, hostname.c_str());
      ros::Duration(updateDelay).sleep();
      dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_IP_ADDR, 0, DISP_TEXT_START_MODE, displayedIpAddress.c_str());
      ros::Duration(updateDelay).sleep();

      ROS_INFO("%s: Display subsystem ready! ", THIS_NODE_NAME);
      ROS_INFO("%s: Listening on topic /%s for messages of type %s", THIS_NODE_NAME,
      ROS_TOPIC_DISPLAY_NODE, "DisplayOutput" );
  } else {
      ROS_ERROR("%s: Display did not initialize properly and will not be used! ", THIS_NODE_NAME);
      return 0;
  }

  // Set to subscribe to the display topic and we then get callbacks for each message
  ros::Subscriber sub = nh.subscribe(ROS_TOPIC_DISPLAY_NODE, 1000, displayApiCallback);

  // Set to subscribe to the battery_state topic and we then get callbacks for each message
  ros::Subscriber sub2 = nh.subscribe("battery_state", 1000, batteryStateApiCallback);

  // Set to subscribe to the motor_power_active topic
  ros::Subscriber sub3 = nh.subscribe("motor_power_active", 1000, motorPowerActiveApiCallback);

  // We will refresh the lines from time to time in case IP addr has changed
  ros::Rate loop_rate(0.5);
  int32_t loopIdx = 0;

  // mainloop:
  while ((dispError == 0) && ros::ok())
  {
    loopIdx++;
    hostname = getPopen("uname -n");
    dispError |= dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_HOSTNAME, 0, DISP_TEXT_START_MODE, hostname.c_str());
    ros::Duration(updateDelay).sleep();
    getMainIpAddress(displayedIpAddress, 0);
    dispError |= dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_IP_ADDR, 0, DISP_TEXT_START_MODE, displayedIpAddress.c_str());
    ros::Duration(updateDelay).sleep();

    // If there is a battery_state topic and we get the callback also show battery voltage
    if (g_batteryVoltage > 0.0) {
        ROS_DEBUG("%s Battery voltage is now %4.1f volts.", THIS_NODE_NAME, g_batteryVoltage);

        // Control what shows up after the voltage reading.
        // If you change the text, use same number of chars so display does not move around on the line
        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << g_batteryVoltage;
	if (g_batteryVoltage >= BAT_CHARGING_LEVEL) {
            stream <<   " CHRG";
        }else if (g_batteryVoltage >= BAT_LOW_LEVEL) {
            stream <<   " OK  ";
        } else {
          if ((loopIdx & 1) == 0) {
              stream << " LOW ";
          } else {
              stream << "     ";
          }
        }
        std::string battText = "Bat " + stream.str();
        dispError |= dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_BATT_VOLTS, 1, DISP_TEXT_START_MODE, battText.c_str());
        ros::Duration(updateDelay).sleep();
    }


    // If there is a motor_power_active topic and we get the callback also show motor power status
    if (g_motorPowerActive >= 0) {
        ROS_DEBUG("%s motor power active is now %d", THIS_NODE_NAME, g_motorPowerActive);

        // If you change the text, use same number of chars so display does not move around on the line
        std::stringstream powStream;
	if (g_motorPowerActive > 0) {
            powStream <<   " ON";
        }else {
            powStream <<   "OFF";
        }
        std::string motPowerText = "Mot Power " + powStream.str();
        dispError |= dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_MOTOR_POWER, 1, DISP_TEXT_START_MODE, motPowerText.c_str());
        ros::Duration(updateDelay).sleep();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // ros::spin() will enter a loop allowing callbacks to happen. Exits on Ctrl-C
  ros::spin();

  return 0;
}

#endif // __arm__ || __aarch64__
