#pragma once

#include "stdint.h"
#include "../../pins_config.h"

#define TFT_MADCTL 0x36
#define TFT_MAD_MY 0x80
#define TFT_MAD_MX 0x40
#define TFT_MAD_MV 0x20
#define TFT_MAD_ML 0x10
#define TFT_MAD_BGR 0x08
#define TFT_MAD_MH 0x04
#define TFT_MAD_RGB 0x00

#define TFT_INVOFF 0x20
#define TFT_INVON 0x21

#define TFT_RES_H digitalWrite(TFT_QSPI_RST, 1);
#define TFT_RES_L digitalWrite(TFT_QSPI_RST, 0);
#define TFT_CS_H digitalWrite(TFT_QSPI_CS, 1);
#define TFT_CS_L digitalWrite(TFT_QSPI_CS, 0);

typedef struct
{
    uint8_t cmd;
    uint8_t data[4];
    uint8_t len;
} lcd_cmd_t;

void CO5300_init(void);
void lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void lcd_setRotation(uint8_t r);
void lcd_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void lcd_fill(uint16_t xsta,
              uint16_t ysta,
              uint16_t xend,
              uint16_t yend,
              uint16_t color);
void lcd_PushColors(uint16_t x,
                    uint16_t y,
                    uint16_t width,
                    uint16_t high,
                    uint16_t *data);
void lcd_PushColors(uint16_t *data, uint32_t len);
void lcd_sleep();

//nikthefix added functions
void lcd_brightness(uint8_t bright); 
void lcd_set_colour_enhance(uint8_t enh);
void lcd_display_off();
void lcd_display_on();
void lcd_display_invert_on();
void lcd_display_invert_off();
void lcd_display_set_colour_enhance_low_byte(uint8_t ce_low_byte);
void lcd_display_set_colour_enhance_high_byte(uint8_t ce_high_byte);      
void lcd_display_high_brightness_mode_on(uint8_t hb_en=0b00000110); 
void lcd_display_high_brightness_mode_off(uint8_t hb_en=0b00000100); 
