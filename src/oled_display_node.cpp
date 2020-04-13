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

/*
 * Display Output Subsystem
 *
 * This module implements a ROS node (process) that does Display Updates to an LCD display
 * Accept messages from a ROS Topic (message queue) and update the display.
 * Full update or substring updates starting at a given line and row are supported.
 */

#define  THIS_NODE_NAME    "oled_display_node"        // The Name for this ROS node. Used in ROS_INFO and more

// Type and I2C address of the display
// The small 1.3" OLED displays typically use the SH1106 controller chip
// The 0.96" OLED display tends to use the SSD1306 controller.
// The SH1106 is fairly compatible with SSD1306 basic, the difference is that the SH1106 control chip
// RAM space is 132*64, while SSD1306 space is 128*64.
// This code supports the 1.3" OLED with the SH1106 controller chip by default
// So besides different initialization bytes the display starts 2 bytes later in one vs the other
#define OLED_DISPLAY_TYPE  DISPLAY_TYPE_SH1106
#define OLED_DISPLAY_ADDR  SH1106_OLED_I2C_ADDRESS
#define OLED_I2C_DEVICE    "/dev/i2c-1"          // SYSTEM SPECIFIC

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

// These next few are for I2C and ioctls and file opens
#include <linux/i2c-dev.h>
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

// External Defs which we keep hidden in this node
extern  int dispOled_writeBytes(dispCtx_t *dispCtx, uint8_t *outBuf, int numChars);
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

#define SH1106_INIT_BYTE_COUNT  8
static uint8_t sh1106_init_bytes[SH1106_INIT_BYTE_COUNT] = {
        OLED_CONTROL_BYTE_CMD_STREAM,
                0x30,                           // Charge pump default
                0x40,                           // RAM display line of 0
                OLED_CMD_SET_SEGMENT_REMAP,
                OLED_CMD_SET_COM_SCAN_MODE,
                0x81, 0x80,         // Display contrast set to second byte
                OLED_CMD_DISPLAY_ON
};

/*
 * @name                dispOled_writeBytes
 *
 * @name                display_setCursor
 * @brief               Move cursor to a given horizontal column and line
 *
 * @param               dispCtx         Context for display that holds type and hardware interface info
 * @param               column          The pixel resolution column from 0 to 127
 * @param               line            The line number where 0 is top line
 *
 * @return              Returns 0 for ok or -1 for IO error
 *
 * @note        Caller is responsible for use of i2c_lock() and i2c_unlock() outside of this call
 */
int dispOled_writeBytes(dispCtx_t *dispCtx, uint8_t *outBuf, int numChars)
{
    int           fd;                                // File descrition
    const char    *fileName = &dispCtx->devName[0];  // Name of the port we will be using
    int           address   = dispCtx->i2cAddr;      // Address of the Modtronixs LCD display

    // Check that the display context seems reasonable and initialized
    if (dispCtx == NULL) {
        ROS_ERROR("%s: Write to OLED display with bad context\n", THIS_NODE_NAME);
        return -1;
    }
    switch (dispCtx->dispType) {
        case DISPLAY_TYPE_SSD1306:
        case DISPLAY_TYPE_SH1106:
            break;
        default:
            ROS_ERROR("%s: Write to OLED display with bad OLED display type\n", THIS_NODE_NAME);
            return -1;
            break;
    }

    if ((fd = open(fileName, O_RDWR)) < 0) {      // Open port for reading and writing
        ROS_ERROR("%s: Unable to open I2C device to talk to slave\n", THIS_NODE_NAME);
        return -2;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {      // Set the port options and addr of the dev
        ROS_ERROR("%s: Unable to get bus access to talk to slave\n", THIS_NODE_NAME);
        close (fd);
        return -3;
    }
    // we are now free to transmit to the I2C port 1 to latch 1

    if ((write(fd, outBuf, numChars)) != numChars) {    // Write commands to the i2c port
        ROS_ERROR("%s: Error writing proper byte count of %d bytes to i2c slave\n", THIS_NODE_NAME, numChars);
        close (fd);
        return -4;
    }
    close (fd);

    return 0;
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
        break;
    case DISPLAY_TYPE_SH1106:
        dispCtx->maxLine      = SH1106_MAX_LINE;
        dispCtx->maxColumn    = SH1106_MAX_COLUMN;
        dispCtx->maxVertPixel = SH1106_MAX_VERT_PIXEL;
        dispCtx->maxHorzPixel = SH1106_MAX_HORZ_PIXEL;
        dispCtx->horzOffset   = SH1106_HORZ_OFFSET;
        break;
    default:
        ROS_ERROR("%s: Unsupported display type of %d\n", THIS_NODE_NAME, dispType);
        retCode = -9;
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
 * @param               dispType        Type of display. DISPLAY_TYPE_SSD1306 or DISPLAY_TYPE_SH1106
 * @param               i2cAddr         7-bit I2C bus address
 *
 * @return              Returns 0 for ok or -1 for IO error
 */
int dispOled_init(std::string devName, dispCtx_t *dispCtx, int dispType, uint8_t i2cAddr)
{
    int retCode = 0;

    dispOled_initCtx(devName, dispCtx, dispType, i2cAddr);

    //Send all the commands to fully initialize the device.
    switch (dispCtx->dispType) {
    case DISPLAY_TYPE_SSD1306:
        // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
                retCode = dispOled_writeBytes(dispCtx, &ssd1306_init_bytes[0], SSD1306_INIT_BYTE_COUNT);
        break;
    case DISPLAY_TYPE_SH1106:
        // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
                retCode = dispOled_writeBytes(dispCtx, &sh1106_init_bytes[0], SH1106_INIT_BYTE_COUNT);
        break;
    default:
        retCode = -9;
        break;
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
                retCode = dispOled_writeBytes(dispCtx, &curserSetup[0], 7);

        case DISPLAY_TYPE_SH1106:
                // SH1106 has different addressing than SSD1306
                curserSetup[0] = OLED_CONTROL_BYTE_CMD_STREAM;
                curserSetup[1] = 0xB0 | (line & 0xf);
                curserSetup[2] = 0x00 | ((column + dispCtx->horzOffset) & 0xf);     // Lower column address
                curserSetup[3] = 0x10 | ((column + dispCtx->horzOffset) >> 4);      // Upper column address

                // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
                retCode = dispOled_writeBytes(dispCtx, &curserSetup[0], 4);
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

        uint8_t zero[128];
        zero[0] = OLED_CONTROL_BYTE_DATA_STREAM;;
        for (uint8_t idx = 1; idx < 128; idx++) {
                zero[idx] = 0;      // All 0 is blank vertical segments of 8 bits all across the row
        }
        for (uint8_t line = 0; line <= dispCtx->maxLine; line++) {

                retCode = dispOled_setCursor(dispCtx, 0, line);
                if (retCode != 0) {
                        return retCode;
                }

                // Clear one line
                retCode = dispOled_writeBytes(dispCtx, &zero[0], (dispCtx->maxHorzPixel+dispCtx->horzOffset));
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

        retCode = dispOled_setCursor(dispCtx, startSegment, line);
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
        retCode = dispOled_writeBytes(dispCtx, &dispData[0], dispDataIdx);

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

  dispCtx_t oledDisplayCtx;
  char charBuf[120];
  int segment = column * DISPLAY_CHAR_WIDTH;
  int dispRow  = row;   // Bypass the system info area
  int maxChars = oledDisplayCtx.maxColumn - 1;
  int lineChars = messageLength;

  // We only initialize display context and do not re-initialize actual display
  dispOled_initCtx(OLED_I2C_DEVICE, &oledDisplayCtx, OLED_DISPLAY_TYPE, OLED_DISPLAY_ADDR);

  // Would be nice to account for non-zero cursor due to cursor control sometime too ...
  if (messageLength > oledDisplayCtx.maxColumn) {
    ROS_ERROR("%s: Unsupported character count of %d\n", THIS_NODE_NAME, messageLength);
    return -9;
  }

  if (lineChars > maxChars) lineChars = maxChars;    // Hard truncate max char count for this display
  strncpy(&charBuf[0], text.c_str(), lineChars);
  charBuf[lineChars] = 0;

  // Write out the characters to the display
  dispOled_writeText(&oledDisplayCtx, dispRow, segment, 0, &charBuf[0]);

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
 * Receive mmessages for display output
 */
void displayApiCallback(const oled_display_node::DisplayOutput::ConstPtr& msg)
{
  ROS_DEBUG("%s heard display output msg: of actionType %d row %d column %d numChars %d attr 0x%x text %s comment %s]",
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

int main(int argc, char **argv)
{
   printf("ROS Node starting:%s \n", THIS_NODE_NAME);

  // The ros::init() function initializes ROS and needs to see argc and argv
  ros::init(argc, argv, THIS_NODE_NAME);

  // Setup a NodeHandle for the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  // Get our hostname on the network
  std::string hostname = getPopen("uname -n");

  // Here we will fetch the current IP address but we assume this node does not start till it is valid
  std::string firstIpAddress = getPopen("hostname -I | cut -f 1 -d' '");

  char dispBuf[32];
  int  dispInitError = 0;

  dispCtx_t oledDisplayCtx;
  dispInitError = dispOled_init(OLED_I2C_DEVICE, &oledDisplayCtx, OLED_DISPLAY_TYPE, OLED_DISPLAY_ADDR);

  if (dispInitError == 0) {
      dispOled_clearDisplay(&oledDisplayCtx);
      dispOled_writeText(&oledDisplayCtx, 0, 0, 1, hostname.c_str());
      dispOled_writeText(&oledDisplayCtx, 2, 0, 1, firstIpAddress.c_str());

      ROS_INFO("%s: Display subsystem ready! ", THIS_NODE_NAME);
      ROS_INFO("%s: Listening on topic /%s for messages of type %s", THIS_NODE_NAME,
      ROS_TOPIC_DISPLAY_NODE, "DisplayOutput" );
  } else {
      ROS_WARN("%s: Display did not initialize properly and will not be used! ", THIS_NODE_NAME);
  }

  // Set to subscribe to the display topic and we then get callbacks for each message
  ros::Subscriber sub = nh.subscribe(ROS_TOPIC_DISPLAY_NODE, 1000, displayApiCallback);

  // We will refresh the lines from time to time in case IP addr has changed
  ros::Rate loop_rate(0.1);

  // mainloop:
  while ((dispInitError == 0) && ros::ok())
  {
    dispOled_writeText(&oledDisplayCtx, 0, 0, 1, hostname.c_str());
    dispOled_writeText(&oledDisplayCtx, 2, 0, 1, firstIpAddress.c_str());

    ros::spinOnce();
    loop_rate.sleep();
  }

  // ros::spin() will enter a loop allowing callbacks to happen. Exits on Ctrl-C
  ros::spin();

  return 0;
}

