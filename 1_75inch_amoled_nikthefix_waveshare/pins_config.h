#pragma once

/***********************config*************************/


//Display - H0175Y003AM - 1.75 inches
#define SPI_FREQUENCY         40000000 //
#define TFT_SPI_MODE          SPI_MODE0
#define TFT_SPI_HOST          SPI2_HOST

#define EXAMPLE_LCD_H_RES     466 
#define EXAMPLE_LCD_V_RES     466 

//#define LVGL_LCD_BUFFER_SIZE  EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES // full frame buffer is slightly faster
#define LVGL_LCD_BUFFER_SIZE  EXAMPLE_LCD_H_RES * 10 
#define SEND_BUF_SIZE         EXAMPLE_LCD_H_RES * 10

#define TFT_QSPI_CS           12
#define TFT_QSPI_SCK          38
#define TFT_QSPI_D0           4
#define TFT_QSPI_D1           5
#define TFT_QSPI_D2           6
#define TFT_QSPI_D3           7
#define TFT_QSPI_RST          39
#define LCD_VCI_EN            -1
#define TFT_TE                -1

// IIC
#define IIC_SDA 15
#define IIC_SCL 14

// TOUCH - CST9217
#define TOUCH_INT 11
#define TOUCH_RST 40

// Battery Voltage ADC
#define BATTERY_VOLTAGE_ADC_DATA -1

// SD
#define SD_CS -1
#define SD_MOSI -1
#define SD_MISO -1
#define SD_SCLK -1

// PCF8563
#define PCF8563_INT -1



