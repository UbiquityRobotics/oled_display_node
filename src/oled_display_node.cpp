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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>

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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "oled_display_node/msg/display_output.hpp"

#include <oled_display_node/oled_display.h>
#include <oled_display_node/font8x8_basic.h>

using namespace std::chrono_literals;

#define  THIS_NODE_NAME    "oled_display_node"

// Default lines to output text to from this module
#define  DISP_LINE_HOSTNAME    0
#define  DISP_LINE_IP_WLAN     1
#define  DISP_LINE_IP_ETH     3
#define  DISP_LINE_BATT_VOLTS  5
#define  DISP_LINE_MOTOR_POWER 7 

#define  BAT_LOW_LEVEL  23.5
#define  BAT_CHARGING_LEVEL  27.0

#define OLED_DISPLAY_TYPE  DISPLAY_TYPE_SH1106
#define OLED_DISPLAY_ADDR  SH1106_OLED_I2C_ADDRESS
#define OLED_I2C_DEVICE    "/dev/i2c-1"

#define DISP_WRITE_TEXT_CENTER      1
#define DISP_WRITE_TEXT_LEFT        0
#define DISP_TEXT_START_MODE    DISP_WRITE_TEXT_CENTER

// Helper functions for I2C and display (static or in anonymous namespace)
namespace {

std::string getPopen(std::string input) {
    std::string result("");
    char c;
    FILE* fp;

    try {
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
    } catch (...) { }

    return result;
}

void getIpAddressses(std::string &ethAddress, std::string &wlanAddress) {
    std::string enetIpAddress = getPopen("ip -o -4 addr | grep eth0  | awk '{print $4}'");
    std::string wlanIpAddress = getPopen("ip -o -4 addr | grep wlan0 | awk '{print $4}'");

    if (enetIpAddress.length() > 8) {
        ethAddress.assign(enetIpAddress.substr(0, enetIpAddress.find('/')));
    } else {
        ethAddress = "---";
    }
    if (wlanIpAddress.length() > 8) {
        wlanAddress.assign(wlanIpAddress.substr(0, wlanIpAddress.find('/')));
    } else {
        wlanAddress = "---";
    }
}

int i2c_read(const char *i2cDevFile, uint8_t i2c7bitAddr,
             uint8_t *pBuffer, int numBytes, uint8_t chipRegAddr, bool chipRegAddrFlag)
{
    int fd;
    const int slaveAddress = i2c7bitAddr;
    int retCode = 0;
    int bytesRead = 0;
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    if ((fd = open(i2cDevFile, O_RDWR)) < 0) {
        retCode = IO_ERR_DEV_OPEN_FAILED;
        return -retCode;
    }

    if (chipRegAddrFlag) {
        msgs[0].addr = slaveAddress;
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = &chipRegAddr;
        msgs[1].addr = slaveAddress;
        msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
        msgs[1].len = numBytes;
        msgs[1].buf = pBuffer;

        msgset[0].msgs = msgs;
        msgset[0].nmsgs = 2;

        if (ioctl(fd, I2C_RDWR, &msgset) < 0) {
            retCode = IO_ERR_IOCTL_ADDR_SET;
            close(fd);
            return -retCode;
        }
    } else {
        if (ioctl(fd, I2C_SLAVE, slaveAddress) != 0) {
            retCode = IO_ERR_IOCTL_ADDR_SET;
            close(fd);
            return -retCode;
        }

        bytesRead = read(fd, pBuffer, numBytes);
        if (bytesRead != numBytes) {
            retCode = IO_ERR_READ_FAILED;
            close(fd);
            return -retCode;
        }
    }
    close(fd);
    return (retCode == 0) ? numBytes : -retCode;
}

int i2c_write(const char *i2cDevFile, uint8_t i2c7bitAddr, uint8_t *pBuffer, int numBytes)
{
    int fd;
    int retCode = 0;
    const int slaveAddress = i2c7bitAddr;

    if ((fd = open(i2cDevFile, O_WRONLY)) < 0) {
        return IO_ERR_DEV_OPEN_FAILED;
    }

    if (ioctl(fd, I2C_SLAVE, slaveAddress) != 0) {
        close(fd);
        return IO_ERR_IOCTL_ADDR_SET;
    }

    if (write(fd, pBuffer, numBytes) != numBytes) {
        retCode = IO_ERR_WRITE_FAILED;
    }

    close(fd);
    return retCode;
}

uint8_t ssd1306_init_bytes[] = {
    OLED_CONTROL_BYTE_CMD_STREAM,
    OLED_CMD_SET_CHARGE_PUMP, 0x14,
    OLED_CMD_SET_SEGMENT_REMAP,
    OLED_CMD_SET_COM_SCAN_MODE,
    OLED_CMD_DISPLAY_ON
};

uint8_t sh1106_init_bytes[] = {
    OLED_CONTROL_BYTE_CMD_STREAM,
    0x30, 0x40, OLED_CMD_DISPLAY_OFF, OLED_CMD_SET_SEGMENT_REMAP,
    OLED_CMD_SET_COM_SCAN_MODE, OLED_CMD_SET_DISPLAY_OFFSET, 0,
    OLED_CMD_DC_DC_CTRL_MODE, 0x8B, 0x81, 0x80,
    OLED_CMD_DISPLAY_RAM, OLED_CMD_DISPLAY_NORMAL, OLED_CMD_SET_MUX_RATIO, 0x3F,
    OLED_CMD_DISPLAY_ON
};

int dispOled_detectDisplayType(std::string devName, uint8_t i2c7bitAddr, int *dispType)
{
    uint8_t buf[16];
    int vote1106 = 0;
    int vote1306 = 0;

    for (int i = 0; i < 5; i++) {
        int retCount = i2c_read(devName.c_str(), i2c7bitAddr, &buf[0], 1, 0x00, true);
        if (retCount == 1) {
            if ((buf[0] & 0x07) == 0x06) {
                vote1306++;
            } else {
                vote1106++;
            }
        }
        usleep(30000);
    }

    if (vote1106 > vote1306) {
        *dispType = DISPLAY_TYPE_SH1106;
    } else {
        *dispType = DISPLAY_TYPE_SSD1306;
    }
    return 0;
}

int dispOled_initCtx(std::string devName, dispCtx_t *dispCtx, int dispType, uint8_t i2cAddr)
{
    dispCtx->dispType = dispType;
    dispCtx->i2cAddr = i2cAddr;
    strncpy(dispCtx->devName, devName.c_str(), sizeof(dispCtx->devName));

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
        return IO_ERR_BAD_DISP_CONTEXT;
    }
    return 0;
}

int dispOled_init(std::string devName, dispCtx_t *dispCtx, int displayType, uint8_t i2cAddr)
{
    int dispType = displayType;
    if (dispType == DISPLAY_TYPE_AUTO) {
        dispOled_detectDisplayType(devName, i2cAddr, &dispType);
    }
    dispOled_initCtx(devName, dispCtx, dispType, i2cAddr);

    int retCode = 0;
    switch (dispCtx->dispType) {
    case DISPLAY_TYPE_SSD1306:
        retCode = i2c_write(dispCtx->devName, dispCtx->i2cAddr, ssd1306_init_bytes, sizeof(ssd1306_init_bytes));
        break;
    case DISPLAY_TYPE_SH1106:
        retCode = i2c_write(dispCtx->devName, dispCtx->i2cAddr, sh1106_init_bytes, sizeof(sh1106_init_bytes));
        break;
    default:
        retCode = IO_ERR_BAD_DISP_CONTEXT;
        break;
    }
    return retCode;
}

int dispOled_setCursor(dispCtx_t *dispCtx, int column, int line) {
    int retCode = 0;
    uint8_t cursorSetup[8];

    switch (dispCtx->dispType) {
    case DISPLAY_TYPE_SSD1306:
        cursorSetup[0] = OLED_CONTROL_BYTE_CMD_STREAM;
        cursorSetup[1] = OLED_CMD_SET_COLUMN_RANGE;
        cursorSetup[2] = column;
        cursorSetup[3] = dispCtx->maxHorzPixel;
        cursorSetup[4] = OLED_CMD_SET_PAGE_RANGE;
        cursorSetup[5] = line;
        cursorSetup[6] = line;
        retCode = i2c_write(dispCtx->devName, dispCtx->i2cAddr, cursorSetup, 7);
        break;
    case DISPLAY_TYPE_SH1106:
        cursorSetup[0] = OLED_CONTROL_BYTE_CMD_STREAM;
        cursorSetup[1] = 0xB0 | (line & 0xf);
        cursorSetup[2] = 0x10 | (((column + dispCtx->horzOffset) & 0xf0) >> 4);
        cursorSetup[3] = 0x00 | ((column + dispCtx->horzOffset)  & 0xf);
        retCode = i2c_write(dispCtx->devName, dispCtx->i2cAddr, cursorSetup, 4);
        break;
    }
    return retCode;
}

int dispOled_clearDisplay(dispCtx_t *dispCtx) {
    int retCode = 0;
    uint8_t zero[140];
    zero[0] = OLED_CONTROL_BYTE_DATA_STREAM;
    for (uint8_t idx = 1; idx < 136; idx++) {
        zero[idx] = 0;
    }
    for (uint8_t line = 0; line <= dispCtx->maxLine; line++) {
        retCode |= dispOled_setCursor(dispCtx, 0, line);
        if (retCode != 0) return retCode;
        retCode |= i2c_write(dispCtx->devName, dispCtx->i2cAddr, zero, 
                             (dispCtx->maxHorzPixel+dispCtx->horzOffset)+dispCtx->endHorzPixel);
        if (retCode != 0) return retCode;
    }
    return retCode;
}

int dispOled_writeText(dispCtx_t *dispCtx, uint8_t line, uint8_t segment, uint8_t center, const char *textStr) {
    int retCode = 0;
    uint8_t text_len = strlen(textStr);
    uint8_t dispData[DISPLAY_MAX_HORZ_PIXEL];
    int dispDataIdx = 1;

    dispData[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    if (line > 7) return -1;
    if ((segment + (text_len * 8)) > dispCtx->maxHorzPixel) return -2;
    
    int startSegment = segment;
    if (segment > (dispCtx->maxHorzPixel-1)) segment = dispCtx->maxHorzPixel-1;
    if (center != 0) {
        startSegment = (dispCtx->maxHorzPixel - (text_len * DISPLAY_CHAR_WIDTH)) / 2;
    }

    retCode = dispOled_setCursor(dispCtx, startSegment, line);
    if (retCode != 0) return retCode;

    for (uint8_t i = 0; i < text_len; i++) {
        for (uint8_t charCol = 0; charCol < DISPLAY_CHAR_WIDTH; charCol++) {
            dispData[dispDataIdx++] = font8x8_basic_tr[(int)(textStr[i])][charCol];
        }
    }
    retCode = i2c_write(dispCtx->devName, dispCtx->i2cAddr, dispData, dispDataIdx);
    return retCode;
}

} // anonymous namespace

class OledDisplayNode : public rclcpp::Node {
public:
    OledDisplayNode() : Node(THIS_NODE_NAME) {
        this->declare_parameter("update_delay_ms", 250);
        update_delay_ms_ = this->get_parameter("update_delay_ms").as_int();

        RCLCPP_INFO(this->get_logger(), "OLED Display node starting.");

        int dispError = dispOled_init(OLED_I2C_DEVICE, &g_oledDisplayCtx, OLED_DISPLAY_TYPE, OLED_DISPLAY_ADDR);

        if (dispError == 0) {
            dispOled_clearDisplay(&g_oledDisplayCtx);
            
            std::string hostname = getPopen("uname -n");
            std::string ethAddr, wlanAddr;
            getIpAddressses(ethAddr, wlanAddr);

            dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_HOSTNAME, 0, DISP_TEXT_START_MODE, hostname.c_str());
            usleep(update_delay_ms_ * 1000);
            dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_IP_WLAN, 0, DISP_TEXT_START_MODE, wlanAddr.c_str());
            usleep(update_delay_ms_ * 1000);
            dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_IP_ETH, 0, DISP_TEXT_START_MODE, ethAddr.c_str());

            RCLCPP_INFO(this->get_logger(), "Display subsystem ready!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Display did not initialize properly!");
            // In ROS 2, we might want to throw or shutdown, but here we'll just log and have no display
        }

        sub_display_ = this->create_subscription<oled_display_node::msg::DisplayOutput>(
            "display_node", 10, std::bind(&OledDisplayNode::displayCallback, this, std::placeholders::_1));

        sub_battery_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10, std::bind(&OledDisplayNode::batteryCallback, this, std::placeholders::_1));

        sub_motor_power_ = this->create_subscription<std_msgs::msg::Bool>(
            "motor_power_active", 10, std::bind(&OledDisplayNode::motorPowerCallback, this, std::placeholders::_1));

        sub_firmware_ = this->create_subscription<std_msgs::msg::String>(
            "firmware_version", 10, std::bind(&OledDisplayNode::firmwareCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(2s, std::bind(&OledDisplayNode::timerCallback, this));
    }

private:
    void displayCallback(const oled_display_node::msg::DisplayOutput::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Heard display msg: action %d row %d col %d text %s",
                    msg->action_type, msg->row, msg->column, msg->text.c_str());

        switch (msg->action_type) {
            case oled_display_node::msg::DisplayOutput::DISPLAY_ALL:
            case oled_display_node::msg::DisplayOutput::DISPLAY_SUBSTRING:
                displayUpdate(msg->text, msg->row, msg->column, msg->num_chars);
                break;
            default:
                break;
        }
    }

    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        std::stringstream percent;
        percent << (int)((msg->percentage * 100) + 0.5) << "%";
        
        std::string pStr = percent.str();
        if (pStr.length() == 4) battery_percentage_ = " " + pStr;
        else if (pStr.length() == 3) battery_percentage_ = "  " + pStr;
        else if (pStr.length() == 2) battery_percentage_ = "   " + pStr;
        else battery_percentage_ = "     ";

        battery_voltage_ = msg->voltage;
    }

    void motorPowerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        motor_power_active_ = msg->data ? "ON " : "OFF";
    }

    void firmwareCallback(const std_msgs::msg::String::SharedPtr msg) {
        firmware_version_ = msg->data.substr(0, msg->data.find(' '));
    }

    void timerCallback() {
        loop_idx_++;
        std::string hostname = getPopen("uname -n");
        dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_HOSTNAME, 0, DISP_TEXT_START_MODE, hostname.c_str());
        usleep(update_delay_ms_ * 1000);

        std::string ethAddr, wlanAddr;
        getIpAddressses(ethAddr, wlanAddr);
        dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_IP_WLAN, 0, DISP_TEXT_START_MODE, wlanAddr.c_str());
        usleep(update_delay_ms_ * 1000);
        dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_IP_ETH, 0, DISP_TEXT_START_MODE, ethAddr.c_str());
        usleep(update_delay_ms_ * 1000);

        if (battery_voltage_ > 0.0) {
            std::stringstream stream;
            stream << std::fixed << std::setprecision(1) << battery_voltage_;

            if (battery_voltage_ <= BAT_LOW_LEVEL && (loop_idx_ % 2) == 0){
                stream << "V  LOW";
            } else if (battery_voltage_ >= BAT_CHARGING_LEVEL && (loop_idx_ % 3) != 0){
                stream << "V CHRG";
            } else {
                stream << "V" << battery_percentage_;
            }

            std::string battText = "Bat:" + stream.str();
            dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_BATT_VOLTS, 1, DISP_TEXT_START_MODE, battText.c_str());
            usleep(update_delay_ms_ * 1000);
        }

        std::stringstream infoStream;
        infoStream << "Mot:" << motor_power_active_ << " FW:" << firmware_version_;
        dispOled_writeText(&g_oledDisplayCtx, DISP_LINE_MOTOR_POWER, 1, DISP_TEXT_START_MODE, infoStream.str().c_str());
    }

    void displayUpdate(std::string text, int row, int column, int num_chars) {
        int messageLength = text.length();
        if (num_chars > 0) messageLength = num_chars;

        dispCtx_t oledDispCtx;
        dispOled_initCtx(OLED_I2C_DEVICE, &oledDispCtx, g_oledDisplayCtx.dispType, OLED_DISPLAY_ADDR);

        if (messageLength > oledDispCtx.maxColumn) return;

        char charBuf[120];
        int lineChars = (messageLength > oledDispCtx.maxColumn - 1) ? oledDispCtx.maxColumn - 1 : messageLength;
        strncpy(charBuf, text.c_str(), lineChars);
        charBuf[lineChars] = 0;

        dispOled_writeText(&oledDispCtx, row, column * DISPLAY_CHAR_WIDTH, DISP_TEXT_START_MODE, charBuf);
    }

    rclcpp::Subscription<oled_display_node::msg::DisplayOutput>::SharedPtr sub_display_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_motor_power_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_firmware_;
    rclcpp::TimerBase::SharedPtr timer_;

    dispCtx_t g_oledDisplayCtx;
    std::string battery_percentage_ = "    ";
    std::string firmware_version_ = "---";
    std::string motor_power_active_ = "OFF";
    double battery_voltage_ = 0.0;
    int32_t loop_idx_ = 0;
    int update_delay_ms_ = 250;
};

int main(int argc, char **argv) {
#ifdef __x86_64__
    (void)fprintf(stderr, "The oled_display_node for the x86/64 is a fake!\n");
    return 1;
#else
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OledDisplayNode>());
    rclcpp::shutdown();
    return 0;
#endif
}
