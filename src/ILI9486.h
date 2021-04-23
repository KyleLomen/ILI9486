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

#define ILI9486_TFTWIDTH  320  ///< ILI9486 max TFT width
#define ILI9486_TFTHEIGHT 480  ///< ILI9486 max TFT height

#define ILI9486_NOP     0x00  ///< No-op register
#define ILI9486_SWRESET 0x01  ///< Software reset register
#define ILI9486_RDDID   0x04  ///< Read display identification information
#define ILI9486_RDDST   0x09  ///< Read Display Status

#define ILI9486_SLPIN  0x10  ///< Enter Sleep Mode
#define ILI9486_SLPOUT 0x11  ///< Sleep Out
#define ILI9486_PTLON  0x12  ///< Partial Mode ON
#define ILI9486_NORON  0x13  ///< Normal Display Mode ON

#define ILI9486_RDMODE     0x0A  ///< Read Display Power Mode
#define ILI9486_RDMADCTL   0x0B  ///< Read Display MADCTL
#define ILI9486_RDPIXFMT   0x0C  ///< Read Display Pixel Format
#define ILI9486_RDIMGFMT   0x0D  ///< Read Display Image Format
#define ILI9486_RDSELFDIAG 0x0F  ///< Read Display Self-Diagnostic Result

#define ILI9486_INVOFF 0x20  ///< Display Inversion OFF
#define ILI9486_INVON  0x21  ///< Display Inversion ON
//#define ILI9486_GAMMASET 0x26 ///< Gamma Set
#define ILI9486_DISPOFF 0x28  ///< Display OFF
#define ILI9486_DISPON  0x29  ///< Display ON

#define ILI9486_CASET 0x2A  ///< Column Address Set
#define ILI9486_PASET 0x2B  ///< Page Address Set
#define ILI9486_RAMWR 0x2C  ///< Memory Write
#define ILI9486_RAMRD 0x2E  ///< Memory Read

#define ILI9486_PTLAR    0x30  ///< Partial Area
#define ILI9486_VSCRDEF  0x33  ///< Vertical Scrolling Definition
#define ILI9486_MADCTL   0x36  ///< Memory Access Control
#define ILI9486_VSCRSADD 0x37  ///< Vertical Scrolling Start Address
#define ILI9486_PIXFMT   0x3A  ///< COLMOD: Pixel Format Set

#define ILI9486_FRMCTR1 0xB1  ///< Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9486_FRMCTR2 0xB2  ///< Frame Rate Control (In Idle Mode/8 colors)
#define ILI9486_FRMCTR3 0xB3  ///< Frame Rate control (In Partial Mode/Full Colors)
#define ILI9486_INVCTR  0xB4  ///< Display Inversion Control
#define ILI9486_DFUNCTR 0xB6  ///< Display Function Control

#define ILI9486_PWCTR1 0xC0  ///< Power Control 1
#define ILI9486_PWCTR2 0xC1  ///< Power Control 2
#define ILI9486_PWCTR3 0xC2  ///< Power Control 3
#define ILI9486_PWCTR4 0xC3  ///< Power Control 4
#define ILI9486_PWCTR5 0xC4  ///< Power Control 5
#define ILI9486_VMCTR1 0xC5  ///< VCOM Control 1
#define ILI9486_VMCTR2 0xC7  ///< VCOM Control 2

#define ILI9486_RDID1 0xDA  ///< Read ID 1
#define ILI9486_RDID2 0xDB  ///< Read ID 2
#define ILI9486_RDID3 0xDC  ///< Read ID 3
#define ILI9486_RDID4 0xD3  ///< Read ID 4

#define ILI9486_GMCTRP1 0xE0  ///< Positive Gamma Correction
#define ILI9486_GMCTRN1 0xE1  ///< Negative Gamma Correction
//#define ILI9486_PWCTR6     0xFC

#define ILI9486_DGAMMA1 0xE2  ///< Digital Gamma Control 1
#define ILI9486_DGAMMA2 0xE3  ///< Digital Gamma Control 2

// Color definitions
#define ILI9486_BLACK       0x0000  ///<   0,   0,   0
#define ILI9486_NAVY        0x000F  ///<   0,   0, 123
#define ILI9486_DARKGREEN   0x03E0  ///<   0, 125,   0
#define ILI9486_DARKCYAN    0x03EF  ///<   0, 125, 123
#define ILI9486_MAROON      0x7800  ///< 123,   0,   0
#define ILI9486_PURPLE      0x780F  ///< 123,   0, 123
#define ILI9486_OLIVE       0x7BE0  ///< 123, 125,   0
#define ILI9486_LIGHTGREY   0xC618  ///< 198, 195, 198
#define ILI9486_DARKGREY    0x7BEF  ///< 123, 125, 123
#define ILI9486_BLUE        0x001F  ///<   0,   0, 255
#define ILI9486_GREEN       0x07E0  ///<   0, 255,   0
#define ILI9486_CYAN        0x07FF  ///<   0, 255, 255
#define ILI9486_RED         0xF800  ///< 255,   0,   0
#define ILI9486_MAGENTA     0xF81F  ///< 255,   0, 255
#define ILI9486_YELLOW      0xFFE0  ///< 255, 255,   0
#define ILI9486_WHITE       0xFFFF  ///< 255, 255, 255
#define ILI9486_ORANGE      0xFD20  ///< 255, 165,   0
#define ILI9486_GREENYELLOW 0xAFE5  ///< 173, 255,  41
#define ILI9486_PINK        0xFC18  ///< 255, 130, 198

class ILI9486: public Adafruit_GFX {
 public:
  ILI9486(uint8_t d0, uint8_t wr, uint8_t dc, int16_t rd = -1, int16_t cs = -1, int16_t rst = -1);

  ILI9486(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t wr,
          uint8_t dc, int16_t rd = -1, int16_t cs = -1, int16_t rst = -1);

  bool begin(bool reset = true);
  void drawPixel(int16_t x, int16_t y, uint16_t color);

  virtual void setRotation(uint8_t x);
  virtual void invertDisplay(bool i);

  virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) { fillRect(x, y, 1, h, color); }
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) { fillRect(x, y, w, 1, color); }
  virtual void fillScreen(uint16_t color) { fillRect(0, 0, _width, _height, color); }
  uint8_t      readcommand8(uint8_t cmd);
  uint16_t     readcommand16(uint8_t cmd);
  void         sendCommand(uint8_t cmd);
  void         sendCommand8(uint8_t cmd, uint8_t data);
  void         sendCommand16(uint8_t cmd, uint16_t data);
  void         sendCommand(uint8_t cmd, const uint8_t data[], uint8_t n);
  inline void  writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  uint16_t     color565(uint8_t r, uint8_t g, uint8_t b);
  virtual void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

 private:
  void     CS_HIGH();
  void     CS_LOW();
  void     DC_HIGH();
  void     DC_LOW();
  void     WRITE(uint8_t d);
  void     WRITE_STROBE();
  uint8_t  READ();
  uint8_t  READ_8();
  uint16_t READ_16();
  void     READ_STROBE();
  void     WRITE_COMMAND(uint8_t cmd);
  void     WRITE_8(uint8_t d);
  void     WRITE_16(uint16_t d);
  void     startTransaction();
  void     endTransaction();

  const uint8_t _dataPins[8], _writePin, _dataCommandPin;
  const int16_t _readPin, _chipSelectPin, _resetPin;
};

#endif  // __ILI9486_H__
