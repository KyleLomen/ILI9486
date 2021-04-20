/**
 * \file ILI9486.h
 * \author Kyle Lomen (kylelomen@gmail.com)
 * \brief
 * \version 0.1
 * \date 04/20/2021
 *
 *
 */

#ifndef __ILI9486_H__
#define __ILI9486_H__

#include <Adafruit_GFX.h>

class ILI9486: public Adafruit_GFX {
 public:
  ILI9486();
  ~ILI9486(void);

  bool begin(uint8_t switchvcc = SSD1306_SWITCHCAPVCC, uint8_t i2caddr = 0, bool reset = true, bool periphBegin = true);
  void display(void);
  void clearDisplay(void);
  void invertDisplay(bool i);
  void dim(bool dim);
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void         startscrollright(uint8_t start, uint8_t stop);
  void         startscrollleft(uint8_t start, uint8_t stop);
  void         startscrolldiagright(uint8_t start, uint8_t stop);
  void         startscrolldiagleft(uint8_t start, uint8_t stop);
  void         stopscroll(void);
  void         ssd1306_command(uint8_t c);
  bool         getPixel(int16_t x, int16_t y);
  uint8_t*     getBuffer(void);

 private:
}

#endif  // __ILI9486_H__
