// Tiny I2C based tiny 128x64 OLED display from www.heltec.cn and others
//
// The SH1106 is 1.3" and has a 132x64 pixel area.  The SSD1306 is 0.96" and 128x64 pixels can be thought of as
// being horizontally centered.   So column 0 on the SSD1306 needs to be column 2 on the SH1106
//
// Great data sheet notes:  http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html
// Basics for code below:   https://github.com/yanbe/ssd1306-esp-idf-i2c/blob/master/main/main.c
//
// This code put together to be used for Mark-Toys.com projects.  Use at your own risk.
// The code is considered to be a starting point and not a finished and polished codebase
//

#ifndef OLED_DISPLAY_H_
#define OLED_DISPLAY_H_


#define DISPLAY_TYPE_NONE         0
#define DISPLAY_TYPE_AUTO         0
#define DISPLAY_TYPE_SSD1306      1    // 0.96" 128x64
#define DISPLAY_TYPE_SH1106       2    // 1.3" diagonal 128x64

// ROS level defines specific to this node and its messages are here
#define  ROS_TOPIC_DISPLAY_NODE     "display_node"

// max segment index for (pixel) on right of display (seg 0 is 1st on left)
// This must be a power of 2 minus 1 for code to work properly
#define DISPLAY_MAX_HORZ_PIXEL  130    // An absolute max horizontal pixel count

// Characteristics of each display we will support
#define SH1106_MAX_HORZ_PIXEL   127    // Maximum horizontal pixel on the display
#define SH1106_MAX_VERT_PIXEL    64    // Maximum vertical pixel on the display
#define SH1106_MAX_LINE           7    // Maximum lines on the display
#define SH1106_MAX_COLUMN        15    // Maximum columns of characters on the display
#define SH1106_HORZ_OFFSET        2    // Pixel offset from left most pixel to start text
#define SH1106_END_HORZ_PIXEL     0    // Pixels at end of horizontal line that are not used

#define SSD1306_MAX_HORZ_PIXEL  127    // Maximum horizontal pixel on the display
#define SSD1306_MAX_VERT_PIXEL   64    // Maximum vertical pixel on the display
#define SSD1306_MAX_LINE          7    // Maximum lines on the display
#define SSD1306_MAX_COLUMN       15    // Maximum columns of characters on the display
#define SSD1306_HORZ_OFFSET       0    // Pixel offset from left most pixel to start text
#define SSD1306_END_HORZ_PIXEL    4    // Pixels at end of horizontal line that are not used

#define DISPLAY_CHAR_WIDTH  8
#define DISPLAY_CHAR_HEIGHT 8

// Some bit-encoded errors for access to the OLED display
#define IO_ERR_DEV_OPEN_FAILED     0x02
#define IO_ERR_IOCTL_ADDR_SET      0x04
#define IO_ERR_WRITE_FAILED        0x08
#define IO_ERR_READ_FAILED         0x10
#define IO_ERR_READ_LENGTH         0x20
#define IO_ERR_BAD_DISP_CONTEXT    0x80



// Display Context used to support different OLED display types
typedef struct {
    int    dispType;
    int    devFd;
    char   devName[64];
    int    i2cAddr;
    int    maxLine;
    int    maxColumn;
    int    maxVertPixel;
    int    maxHorzPixel;
    int    horzOffset;
    int    endHorzPixel;
} dispCtx_t;

// Following definitions are bollowed from
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define SSD1306_OLED_I2C_ADDRESS   0x3C
#define SH1106_OLED_I2C_ADDRESS    0x3C

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DC_DC_CTRL_MODE        0xAD    // Must have display off and must follow this with 0x8B
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14

#endif /* OLED_DISPLAY_H_ */
