#ifndef LCD_H
#define LCD_H
/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#define CLUT_TABLE_MAX 	16u
#define BLACK_DATA       (0xFF000000u)
#define BLUE_DATA        (0xFF0000FFu)
#define GREEN_DATA       (0xFF00FF00u)
#define RED_DATA         (0xFFFF0000u)
#define WHITE_DATA       (0xFFFFFFFFu)
#define CYAN_DATA        (0xFF00FFFFu)
#define YELLOW_DATA      (0xFFFFFF00u)
#define MAGENTA_DATA     (0xFFFF00FFu)
#define NAVY_DATA        (0xFF000080u)
#define DARKGREEN_DATA   (0xFF006400u)
#define DEEPSKYBLUE_DATA (0xFF00BFFFu)
#define PURPLE_DATA      (0xFF800080u)
#define GRAY_DATA        (0xFF808080u)
#define BROWN_DATA       (0xFFA52A2Au)
#define GOLD_DATA        (0xFFFFD780u)
#define TRANSPARENT_DATA (0x00FFFFFFu)

typedef enum _RGB_enum_t {
	BLACK,
	BLUE,
	GREEN,
	RED,
	WHITE,
	CYAN,
	YELLOW,
	MAGENTA,
	NAVY,
	DARKGREEN,
	DEEPSKYBLUE,
	PURPLE,
	GRAY,
	BROWN,
	GOLD,
	TRANSPARENT,
} RGB_enum_t;

// NOTE: We have to use 768 instead of 800 because for CLUT4, the number of bytes for each line must be exactly divisible by 32
//#define LCD_WIDTH     (800u)
#define LCD_WIDTH     (768u)
#define LCD_HEIGHT    (480u)

#define LCD_FORMAT_YCBCR422 0
#define LCD_FORMAT_RGB565 1
#define LCD_FORMAT_CLUT4 2

//#define LCD_FORMAT LCD_FORMAT_RGB565
#define LCD_FORMAT LCD_FORMAT_CLUT4

#define CHAR_PIX_WIDTH 6
#define CHAR_PIX_HEIGHT 8

#define FONTDATA_WIDTH    (12)
#define FONTDATA_HEIGHT   (24)

#define ARRAY_SIZE(array) \
    (sizeof(array) / sizeof(*array))
    

void LcdClear(char *layer, int lines);
void LcdWriteChar(uint8_t code, uint32_t x, uint32_t y, RGB_enum_t color, char *layer);
void LcdWriteString(uint8_t *pcode, uint32_t x, uint32_t y, RGB_enum_t color, char *layer);
void LcdWriteLine(int32_t x1, int32_t y1, int32_t x2, int32_t y2, RGB_enum_t color, char *layer);
void DrawPoint(int32_t x, int32_t y , RGB_enum_t color , char *layer);
void DrawLine(int32_t x1, int32_t y1, int32_t x2, int32_t y2, RGB_enum_t color, char *layer );

#endif
