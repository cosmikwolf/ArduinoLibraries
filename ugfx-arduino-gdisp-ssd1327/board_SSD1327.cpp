//#include <Arduino.h>
#include <SPI.h>
//#include <Wire.h>

#include "board_SSD1327.h"

// GPIO Pins
#include "ugfx-arduino-gdisp-SSD1327.h"
//#include "ugfx-arduino-SSD1327-pins.h"

#define GPIO_RESET SSD1327_pins.reset
#define GPIO_CS SSD1327_pins.cs
#define GPIO_DC SSD1327_pins.dc

#define SPILOWSPEED 1400000
#define SPIHIGHSPEED 14000000
#define SPIBITORDER MSBFIRST
#define SPIMODE SPI_MODE0

static SPISettings spiSettings(SPILOWSPEED, SPIBITORDER, SPIMODE);

static inline void cmdmode()
{
  digitalWrite(GPIO_DC, 0);
}

static inline void datamode()
{
  digitalWrite(GPIO_DC, 1);
}

void SSD1327_init_board(void) {
  Serial.println("init board");
  pinMode(GPIO_RESET, OUTPUT);
  pinMode(GPIO_RESET, 1);
  pinMode(GPIO_CS, OUTPUT);
  digitalWrite(GPIO_CS, 1);
  pinMode(GPIO_DC, OUTPUT);
  datamode();
}

void SSD1327_post_init_board(void) {
  // speed up SPI (12 MHz)
  spiSettings = SPISettings(SPIHIGHSPEED, SPIBITORDER, SPIMODE);
}

void SSD1327_setpin_reset(int state) {
  if(state)
    digitalWrite(GPIO_RESET, 0);
  else
    digitalWrite(GPIO_RESET, 1);
}

void SSD1327_aquirebus(void) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(GPIO_CS, 0);
}

void SSD1327_releasebus(void) {
  digitalWrite(GPIO_CS, 1);
  SPI.endTransaction();
}

void SSD1327_write_cmd(uint8_t cmd) {
  cmdmode();
  SPI.transfer(cmd);
  datamode();
}

void SSD1327_write_data(uint8_t data) {
  SPI.transfer(data);
}
