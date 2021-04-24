#include "ILI9486.h"

#include <Adafruit_GFX.h>
#include <Arduino.h>

#define dataSetupTime     20
#define dataHoldTime      20
#define readDelay         500
#define outputDisableTime 100
#define writeDelay        50
#define chipSelectDelay   400

#define MADCTL_MY  0x80  ///< Bottom to top
#define MADCTL_MX  0x40  ///< Right to left
#define MADCTL_MV  0x20  ///< Reverse Mode
#define MADCTL_ML  0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00  ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08  ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04  ///< LCD refresh right to left

#if !defined(ARDUINO_TEENSY40)
static inline void delayNanoseconds(uint32_t nsec);
static inline void delayNanoseconds(uint32_t nsec) {
  delayMicroseconds((nsec + 999) / 1000);
}
#endif

ILI9486::ILI9486(uint8_t d0, uint8_t wr, uint8_t dc, int16_t rd, int16_t cs, int16_t rst)
    : ILI9486(d0, d0 + 1, d0 + 2, d0 + 3, d0 + 4, d0 + 5, d0 + 6, d0 + 7, wr, dc, rd, cs, rst) {}
ILI9486::ILI9486(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t wr,
                 uint8_t dc, int16_t rd, int16_t cs, int16_t rst)
    : Adafruit_GFX(ILI9486_TFTWIDTH, ILI9486_TFTHEIGHT),
      _dataPins{d0, d1, d2, d3, d4, d5, d6, d7},
      _writePin(wr),
      _dataCommandPin(dc),
      _readPin(rd),
      _chipSelectPin(cs),
      _resetPin(rst) {}

bool ILI9486::begin(bool reset) {
  for(uint8_t p : _dataPins) {
    pinMode(p, OUTPUT);
    digitalWriteFast(p, LOW);
  }

  if(_readPin > 0) {
    pinMode(_readPin, OUTPUT);
    digitalWriteFast(_readPin, HIGH);
  }
  pinMode(_writePin, OUTPUT);
  digitalWriteFast(_writePin, HIGH);
  pinMode(_dataCommandPin, OUTPUT);
  digitalWriteFast(_dataCommandPin, HIGH);
  if(_chipSelectPin > 0) {
    pinMode(_chipSelectPin, OUTPUT);
    digitalWriteFast(_chipSelectPin, HIGH);
  }

  if(reset) {
    if(_resetPin > 0) {
      pinMode(_resetPin, OUTPUT);
      digitalWriteFast(_resetPin, LOW);
      digitalWriteFast(_resetPin, HIGH);
    }
    else {
      sendCommand(ILI9486_SWRESET);
    }
    delay(150);
  }
  sendCommand(ILI9486_SLPOUT);
  delay(5);

  setRotation(0);
  sendCommand8(ILI9486_PIXFMT, 0x55);
  sendCommand(ILI9486_CASET, (const uint8_t[]){0x00, 0x00, 0x01, 0x3F}, 4);
  sendCommand(ILI9486_PASET, (const uint8_t[]){0x00, 0x00, 0x01, 0xDF}, 4);
  sendCommand(ILI9486_VSCRDEF, (const uint8_t[]){0x00, 0x00, 0x01, 0xE0, 0x00, 0x00}, 6);
  sendCommand16(ILI9486_VSCRSADD, 0x0000);
  sendCommand(ILI9486_NORON);
  invertDisplay(false);
  sendCommand(ILI9486_DISPON);

  sendCommand16(ILI9486_PWCTR1, 0x0d0d);                                      //Power Control 1 [0E 0E]
  sendCommand16(ILI9486_PWCTR2, 0x4300);                                      //Power Control 2 [43 00]
  sendCommand8(ILI9486_PWCTR3, 0x00);                                         //Power Control 3 [33]
  sendCommand(ILI9486_VMCTR1, (const uint8_t[]){0x00, 0x48, 0x00, 0x48}, 4);  //VCOM  Control 1 [00 40 00 40]
  sendCommand8(ILI9486_INVCTR, 0x00);                                         //Inversion Control [00]
  sendCommand(ILI9486_DFUNCTR, (const uint8_t[]){0x02, 0x02, 0x3B}, 3);       // Display Function Control [02 02 3B]
  sendCommand(ILI9486_GMCTRP1,
              (const uint8_t[]){0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00},
              15);
  sendCommand(ILI9486_GMCTRN1,
              (const uint8_t[]){0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00},
              15);
  return true;
}

void ILI9486::setAddrWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd) {
  uint8_t buffer[4];
  buffer[0] = xStart >> 8;
  buffer[1] = xStart & 0xFF;
  buffer[2] = xEnd >> 8;
  buffer[3] = xEnd & 0xFF;
  sendCommand(ILI9486_CASET, buffer, 4);
  buffer[0] = yStart >> 8;
  buffer[1] = yStart & 0xFF;
  buffer[2] = yEnd >> 8;
  buffer[3] = yEnd & 0xFF;
  sendCommand(ILI9486_PASET, buffer, 4);
}

void ILI9486::CS_HIGH() {
  if(_chipSelectPin > 0)
    digitalWriteFast(_chipSelectPin, HIGH);
}

void ILI9486::CS_LOW() {
  if(_chipSelectPin > 0)
    digitalWriteFast(_chipSelectPin, LOW);
}

void ILI9486::DC_HIGH() {
  digitalWriteFast(_dataCommandPin, HIGH);
}

void ILI9486::DC_LOW() {
  digitalWriteFast(_dataCommandPin, LOW);
}

void ILI9486::WRITE(uint8_t d) {
  digitalWriteFast(_dataPins[0], d & (1 << 0));
  digitalWriteFast(_dataPins[1], d & (1 << 1));
  digitalWriteFast(_dataPins[2], d & (1 << 2));
  digitalWriteFast(_dataPins[3], d & (1 << 3));
  digitalWriteFast(_dataPins[4], d & (1 << 4));
  digitalWriteFast(_dataPins[5], d & (1 << 5));
  digitalWriteFast(_dataPins[6], d & (1 << 6));
  digitalWriteFast(_dataPins[7], d & (1 << 7));
}

void ILI9486::WRITE_STROBE() {
  digitalWriteFast(_writePin, LOW);
  //TODO: Add delay?
  digitalWriteFast(_writePin, HIGH);
}

uint8_t ILI9486::READ() {
  if(_readPin > 0) {
    uint8_t d = 0;
    d |= digitalReadFast(_dataPins[0]) << 0;
    d |= digitalReadFast(_dataPins[1]) << 1;
    d |= digitalReadFast(_dataPins[2]) << 2;
    d |= digitalReadFast(_dataPins[3]) << 3;
    d |= digitalReadFast(_dataPins[4]) << 4;
    d |= digitalReadFast(_dataPins[5]) << 5;
    d |= digitalReadFast(_dataPins[6]) << 6;
    d |= digitalReadFast(_dataPins[7]) << 7;
    return d;
  }
  else
    return 0;
}

uint8_t ILI9486::READ_8() {
  uint8_t data;
  for(uint8_t p : _dataPins) {
    pinMode(p, INPUT);
  }
  DC_HIGH();
  READ_STROBE();
  READ_STROBE();
  data = READ();
  for(uint8_t p : _dataPins) {
    pinMode(p, OUTPUT);
  }
  return data;
}

uint16_t ILI9486::READ_16() {
  uint16_t data = 0;
  for(uint8_t p : _dataPins) {
    pinMode(p, INPUT);
  }
  DC_HIGH();
  READ_STROBE();
  READ_STROBE();
  data |= READ() << 8;
  READ_STROBE();
  data |= READ();
  for(uint8_t p : _dataPins) {
    pinMode(p, OUTPUT);
  }
  return data;
}

void ILI9486::READ_STROBE() {
  if(_readPin > 0) {
    digitalWriteFast(_readPin, LOW);
    // TODO: Add delay?
    digitalWriteFast(_readPin, HIGH);
  }
}

void ILI9486::WRITE_COMMAND(uint8_t cmd) {
  DC_LOW();
  WRITE(cmd);
  WRITE_STROBE();
}

void ILI9486::WRITE_8(uint8_t d) {
  DC_HIGH();
  WRITE(d);
  WRITE_STROBE();
}

void ILI9486::WRITE_16(uint16_t d) {
  DC_HIGH();
  WRITE(d >> 8);
  WRITE_STROBE();
  WRITE(d & 0xFF);
  WRITE_STROBE();
}

void ILI9486::drawPixel(int16_t x, int16_t y, uint16_t color) {
  setAddrWindow(x, y, x, y);
  sendCommand16(ILI9486_RAMWR, color);
  //writeFillRectPreclipped(x, y, 1, 1, color);
}

void ILI9486::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  int16_t end;
  if(w < 0) {
    w = -w;
    x -= w;
  }  //+ve w
  end = x + w;
  if(x < 0)
    x = 0;
  if(end > width())
    end = width();
  w = end - x;
  if(h < 0) {
    h = -h;
    y -= h;
  }  //+ve h
  end = y + h;
  if(y < 0)
    y = 0;
  if(end > height())
    end = height();
  h = end - y;
  writeFillRectPreclipped(x, y, w, h, color);
}

inline void ILI9486::writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  setAddrWindow(x, y, x + w - 1, y + h - 1);

  startTransaction();
  WRITE_COMMAND(ILI9486_RAMWR);
  if((color >> 8) == (color & 0xFF)) {
    DC_HIGH();
    WRITE(color & 0xFF);
    for(; w > 0; w--) {
      for(uint16_t end = h; end > 0; end--) {
        WRITE_STROBE();
        WRITE_STROBE();
      }
    }
  }
  else {
    for(; w > 0; w--) {
      for(uint16_t end = h; end > 0; end--) {
        WRITE_16(color);
      }
    }
  }
  endTransaction();
}

void ILI9486::setRotation(uint8_t m) {
  rotation = m % 4;  // can't be higher than 3
  switch(rotation) {
    case 0:
      m       = (MADCTL_MV | MADCTL_BGR);
      _width  = WIDTH;
      _height = HEIGHT;
      break;
    case 1:
      m       = (MADCTL_MX | MADCTL_BGR);
      _width  = HEIGHT;
      _height = WIDTH;
      break;
    case 2:
      m       = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
      _width  = WIDTH;
      _height = HEIGHT;
      break;
    case 3:
      m       = (MADCTL_MY | MADCTL_BGR);
      _width  = HEIGHT;
      _height = WIDTH;
      break;
  }

  sendCommand8(ILI9486_MADCTL, m);
}

void ILI9486::invertDisplay(bool i) {
  if(i) {
    sendCommand(ILI9486_INVON);
  }
  else {
    sendCommand(ILI9486_INVOFF);
  }
}

void ILI9486::startTransaction() {
  CS_LOW();
}

void ILI9486::endTransaction() {
  CS_HIGH();
}

uint8_t ILI9486::readcommand8(uint8_t cmd) {
  uint8_t value;
  startTransaction();
  WRITE_COMMAND(cmd);
  value = READ_8();
  endTransaction();
  return value;
}

uint16_t ILI9486::readcommand16(uint8_t cmd) {
  uint16_t value;
  startTransaction();
  WRITE_COMMAND(cmd);
  value = READ_16();
  endTransaction();
  return value;
}

void ILI9486::sendCommand(uint8_t cmd) {
  sendCommand(cmd, nullptr, 0);
}

void ILI9486::sendCommand8(uint8_t cmd, uint8_t data) {
  sendCommand(cmd, &data, 1);
}

void ILI9486::sendCommand16(uint8_t cmd, uint16_t data) {
  uint8_t buffer[2] = {(uint8_t)(data >> 8), uint8_t(data & 0xFF)};
  sendCommand(cmd, buffer, 2);
}

void ILI9486::sendCommand(uint8_t cmd, const uint8_t data[], uint8_t n) {
  startTransaction();
  WRITE_COMMAND(cmd);
  for(int i = 0; i < n; i++) {
    WRITE_8(*data);
    data++;
  }
  endTransaction();
}
