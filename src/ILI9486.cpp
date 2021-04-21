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
static inline void __delayNanoseconds(uint32_t nsec) __attribute__((weakref("delayNanoseconds")));
static inline void __delayNanoseconds(uint32_t nsec) {
  delayMicroseconds((nsec + 999) / 1000);
}
#endif

ILI9486::ILI9486(uint8_t d0, uint8_t rd, uint8_t wr, uint8_t dc, int8_t cs, int8_t rst)
    : ILI9486(d0, d0 + 1, d0 + 2, d0 + 3, d0 + 4, d0 + 5, d0 + 6, d0 + 7, rd, wr, dc, cs, rst) {}
ILI9486::ILI9486(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t rd,
                 uint8_t wr, uint8_t dc, int8_t cs, int8_t rst)
    : Adafruit_GFX(ILI9486_TFTWIDTH, ILI9486_TFTHEIGHT) {
  _dataPins[0] = d0;
  _dataPins[1] = d1;
  _dataPins[2] = d2;
  _dataPins[3] = d3;
  _dataPins[4] = d4;
  _dataPins[5] = d5;
  _dataPins[6] = d6;
  _dataPins[7] = d7;
  for(uint8_t p : _dataPins) {
    pinMode(p, INPUT);
  }

  _readPin = rd;
  pinMode(_readPin, OUTPUT);
  digitalWrite(_readPin, HIGH);

  _writePin = wr;
  pinMode(_writePin, OUTPUT);
  digitalWrite(_writePin, HIGH);

  _dataCommandPin = dc;
  pinMode(_dataCommandPin, OUTPUT);
  digitalWrite(_dataCommandPin, HIGH);

  if(cs != -1) {
    _useChipSelect = true;
    _chipSelectPin = cs;
    pinMode(_chipSelectPin, OUTPUT);
    digitalWrite(_chipSelectPin, HIGH);
  }
  else {
    _useChipSelect = false;
  }
  if(rst != -1) {
    _useReset = true;
    _resetPin = rst;
    pinMode(_resetPin, OUTPUT);
    digitalWrite(_resetPin, LOW);
  }
  else {
    _useReset = false;
  }
}

bool ILI9486::begin(bool reset) {
  if(reset) {
    if(_useReset) {
      digitalWrite(_resetPin, HIGH);
    }
    else {
      sendFrame(ILI9486_SWRESET);
    }
    delay(150);
  }
  sendFrame(ILI9486_SLPOUT);
  delay(5);

  setRotation(0);
  sendFrame(ILI9486_PIXFMT, 1, (const uint8_t[]){0x55});
  sendFrame(ILI9486_CASET, 4, (const uint8_t[]){0x00, 0x00, 0x01, 0x3F});
  sendFrame(ILI9486_PASET, 4, (const uint8_t[]){0x00, 0x00, 0x01, 0xDF});
  sendFrame(ILI9486_VSCRDEF, 6, (const uint8_t[]){0x00, 0x00, 0x01, 0xE0, 0x00, 0x00});
  sendFrame(ILI9486_VSCRSADD, 2, (const uint8_t[]){0x00, 0x00});
  sendFrame(ILI9486_NORON);
  invertDisplay(false);
  sendFrame(ILI9486_DISPON);

  sendFrame(ILI9486_PWCTR1, 2, (const uint8_t[]){0x0d, 0x0d});              //Power Control 1 [0E 0E]
  sendFrame(ILI9486_PWCTR2, 2, (const uint8_t[]){0x43, 0x00});              //Power Control 2 [43 00]
  sendFrame(ILI9486_PWCTR3, 1, (const uint8_t[]){0x00});                    //Power Control 3 [33]
  sendFrame(ILI9486_VMCTR1, 4, (const uint8_t[]){0x00, 0x48, 0x00, 0x48});  //VCOM  Control 1 [00 40 00 40]
  sendFrame(ILI9486_INVCTR, 1, (const uint8_t[]){0x00});                    //Inversion Control [00]
  sendFrame(ILI9486_DFUNCTR, 3, (const uint8_t[]){0x02, 0x02, 0x3B});       // Display Function Control [02 02 3B]
  sendFrame(ILI9486_GMCTRP1, 15,
            (const uint8_t[]){0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00});
  sendFrame(ILI9486_GMCTRN1, 15,
            (const uint8_t[]){0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00});
  return true;
}

void ILI9486::setBounds(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd) {
  uint8_t buffer[4];
  buffer[0] = xStart >> 8;
  buffer[1] = xStart & 0xFF;
  buffer[2] = xEnd >> 8;
  buffer[3] = xEnd & 0xFF;
  sendFrame(ILI9486_CASET, 4, buffer);
  buffer[0] = yStart >> 8;
  buffer[1] = yStart & 0xFF;
  buffer[2] = yEnd >> 8;
  buffer[3] = yEnd & 0xFF;
  sendFrame(ILI9486_PASET, 4, buffer);
}

void ILI9486::drawPixel(int16_t x, int16_t y, uint16_t color) {
  setBounds(x, y, x, y);
  sendFrame(ILI9486_RAMWR, 2, (uint8_t*)&color);
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
  setBounds(x, y, x + w - 1, y + h - 1);

  startTransaction();
  sendCommand(ILI9486_RAMWR);
  static bool hasRun = false;
  uint32_t    start  = micros();
  for(; w > 0; w--) {
    for(end = h; end > 0; end--) {
      sendData(color >> 8);
      sendData(color & 0xFF);
      if(!hasRun) {
        uint32_t stop = micros();
        Serial.print("Time taken: ");
        Serial.println(stop - start);
        hasRun = true;
      }
    }
  }
  endTransaction();
}

void ILI9486::setRotation(uint8_t m) {
  rotation = m % 4;  // can't be higher than 3
  switch(rotation) {
    case 0:
      m       = (MADCTL_MX | MADCTL_BGR);
      _width  = WIDTH;
      _height = HEIGHT;
      break;
    case 1:
      m       = (MADCTL_MV | MADCTL_BGR);
      _width  = HEIGHT;
      _height = WIDTH;
      break;
    case 2:
      m       = (MADCTL_MY | MADCTL_BGR);
      _width  = WIDTH;
      _height = HEIGHT;
      break;
    case 3:
      m       = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
      _width  = HEIGHT;
      _height = WIDTH;
      break;
  }

  sendFrame(ILI9486_MADCTL, 1, &m);
}

void ILI9486::invertDisplay(bool i) {
  if(i) {
    sendFrame(ILI9486_INVON);
  }
  else {
    sendFrame(ILI9486_INVOFF);
  }
}

void ILI9486::sendCommand(uint8_t command) {
  for(uint8_t p : _dataPins) {
    pinMode(p, OUTPUT);
  }
  digitalWrite(_dataCommandPin, LOW);
  delayNanoseconds(dataHoldTime);
  digitalWrite(_writePin, LOW);
  for(int i = 0; i < 8; i++) {
    if(command & (1 << i))
      digitalWrite(_dataPins[i], HIGH);
    else
      digitalWrite(_dataPins[i], LOW);
  }
  delayNanoseconds(dataSetupTime);
  digitalWrite(_writePin, HIGH);
  delayNanoseconds(writeDelay);
}

void ILI9486::sendData(uint8_t data) {
  for(uint8_t p : _dataPins) {
    pinMode(p, OUTPUT);
  }
  digitalWrite(_dataCommandPin, HIGH);
  delayNanoseconds(dataHoldTime);
  digitalWrite(_writePin, LOW);
  for(int i = 0; i < 8; i++) {
    if(data & (1 << i))
      digitalWrite(_dataPins[i], HIGH);
    else
      digitalWrite(_dataPins[i], LOW);
  }
  delayNanoseconds(dataSetupTime);
  digitalWrite(_writePin, HIGH);
  delayNanoseconds(writeDelay);
}

uint8_t ILI9486::readData() {
  uint8_t data = 0;
  for(uint8_t p : _dataPins) {
    pinMode(p, INPUT);
  }
  digitalWrite(_dataCommandPin, HIGH);
  delayNanoseconds(dataHoldTime);
  digitalWrite(_readPin, LOW);
  delayNanoseconds(readDelay);
  digitalWrite(_readPin, HIGH);
  for(int i = 0; i < 8; i++) {
    if(digitalRead(_dataPins[i]))
      data |= 1 << i;
  }
  delayNanoseconds(outputDisableTime);
  return data;
}

void ILI9486::startTransaction() {
  if(_useChipSelect) {
    digitalWrite(_chipSelectPin, LOW);
    delayNanoseconds(chipSelectDelay);
  }
}

void ILI9486::endTransaction() {
  for(uint8_t p : _dataPins) {
    pinMode(p, INPUT);
  }
  if(_useChipSelect) {
    digitalWrite(_chipSelectPin, HIGH);
    delayNanoseconds(chipSelectDelay);
  }
}

void ILI9486::sendFrame(uint8_t command, uint8_t length, const uint8_t data[]) {
  startTransaction();
  sendCommand(command);
  for(int i = 0; i < length; i++) {
    sendData(data[i]);
  }
  endTransaction();
}
void ILI9486::sendFrame(uint8_t command) {
  startTransaction();
  sendCommand(command);
  endTransaction();
}

uint8_t ILI9486::readcommand8(uint8_t cmd) {
  uint8_t value;
  startTransaction();
  sendCommand(cmd);
  readData();
  value = readData();
  endTransaction();
  return value;
}
