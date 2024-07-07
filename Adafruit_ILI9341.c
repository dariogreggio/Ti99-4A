/*!
 * @file Adafruit_ILI9341.cpp
 *
 * @mainpage Adafruit ILI9341 TFT Displays
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's ILI9341 driver for the
 * Arduino platform.
 *
 * This library works with the Adafruit 2.8" Touch Shield V2 (SPI)
 *    http://www.adafruit.com/products/1651
 *
 * Adafruit 2.4" TFT LCD with Touchscreen Breakout w/MicroSD Socket - ILI9341
 *    https://www.adafruit.com/product/2478
 *
 * 2.8" TFT LCD with Touchscreen Breakout Board w/MicroSD Socket - ILI9341
 *    https://www.adafruit.com/product/1770
 *
 * 2.2" 18-bit color TFT LCD display with microSD card breakout - ILI9340
 *    https://www.adafruit.com/product/1770
 *
 * TFT FeatherWing - 2.4" 320x240 Touchscreen For All Feathers
 *    https://www.adafruit.com/product/3315
 *
 * These displays use SPI to communicate, 4 or 5 pins are required
 * to interface (RST is optional).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
 * Adafruit_GFX</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 * 
 * G.Dar C version on 30/6/2020
 *
 */

#include "z80_pic.h"
#include "Adafruit_ILI9341.h"
//#include "pins_arduino.h"
//#include "lcd.h"
#include <limits.h>
//#include "Print.h"
#ifndef __PIC32
#include <libpic30.h>
#endif
#include "Adafruit_GFX.h"

//#define SPI_DEFAULT_FREQ 24000000 ///< Default SPI data clock frequency


uint8_t rotation;

extern const char CopyrightString[];
extern const unsigned char c64logo[];

// -------------------------------------------------------------------------
// Lowest-level hardware-interfacing functions. Many of these are inline and
// compile to different things based on #defines -- typically just a few
// instructions. Others, not so much, those are not inlined.

/*!
    @brief  Start an SPI transaction if using the hardware SPI interface to
            the display. If using an earlier version of the Arduino platform
            (before the addition of SPI transactions), this instead attempts
            to set up the SPI clock and mode. No action is taken if the
            connection is not hardware SPI-based. This does NOT include a
            chip-select operation -- see startWrite() for a function that
            encapsulated both actions.
*/
inline void SPI_BEGIN_TRANSACTION(void) {
  
  }

/*!
    @brief  End an SPI transaction if using the hardware SPI interface to
            the display. No action is taken if the connection is not
            hardware SPI-based or if using an earlier version of the Arduino
            platform (before the addition of SPI transactions). This does
            NOT include a chip-deselect operation -- see endWrite() for a
            function that encapsulated both actions.
*/
inline void SPI_END_TRANSACTION(void) {
  
  }

#if defined(__PIC32MM__)
#define WRITE_LATB(n) 
#else
#define WRITE_LATB(n) {\
  LATEbits.LATE6=n & 1 ? 1 : 0;\
  LATEbits.LATE7=n & 2 ? 1 : 0;\
  LATBbits.LATB13=n & 4 ? 1 : 0;\
  LATBbits.LATB14=n & 8 ? 1 : 0;\
  LATBbits.LATB15=n & 16 ? 1 : 0;\
  LATFbits.LATF4=n & 32 ? 1 : 0;\
  LATFbits.LATF5=n & 64 ? 1 : 0;\
  LATEbits.LATE5=n & 128 ? 1 : 0;\
  }
#endif
#if defined(__PIC32MM__)
#define READ_PORTB(n) {n=0;}
#else
#define READ_PORTB(n) {\
  n = PORTEbits.RE6 ? 1 : 0;\
  n |= PORTEbits.RE7 ? 2 : 0;\
  n |= PORTBbits.RB13 ? 4 : 0;\
  n |= PORTBbits.RB14 ? 8 : 0;\
  n |= PORTBbits.RB15 ? 16 : 0;\
  n |= PORTFbits.RF4 ? 32 : 0;\
  n |= PORTFbits.RF5 ? 64 : 0;\
  n |= PORTEbits.RE5 ? 128 : 0;\
  }
#endif


/*!
    @brief  Write a single command byte to the display. Chip-select and
            transaction must have been previously set -- this ONLY sets
            the device to COMMAND mode, issues the byte and then restores
            DATA mode. There is no corresponding explicit writeData()
            function -- just use spiWrite().
    @param  cmd  8-bit command to write.
*/
void writeCommand(uint8_t cmd) {
  
  SPI_DC_LOW();
#if defined(__AVR__)
    *tft8.writePort = b;
#elif defined(USE_FAST_PINIO)
    if (!tft8.wide)
      *tft8.writePort = b;
    else
      *(volatile uint16_t *)tft8.writePort = b;
#endif
  WRITE_LATB(cmd);
  TFT_WR_STROBE();
  SPI_DC_HIGH();
  }



/*!
    @brief   Adafruit_SPITFT constructor for parallel display connection.
    @param   w         Display width in pixels at default rotation (0).
    @param   h         Display height in pixels at default rotation (0).
    @param   busWidth  If tft16 (enumeration in header file), is a 16-bit
                       parallel connection, else 8-bit.
                       16-bit isn't fully implemented or tested yet so
                       applications should pass "tft8bitbus" for now...needed to
                       stick a required enum argument in there to
                       disambiguate this constructor from the soft-SPI case.
                       Argument is ignored on 8-bit architectures (no 'wide'
                       support there since PORTs are 8 bits anyway).
    @param   d0        Arduino pin # for data bit 0 (1+ are extrapolated).
                       The 8 (or 16) data bits MUST be contiguous and byte-
                       aligned (or word-aligned for wide interface) within
                       the same PORT register (might not correspond to
                       Arduino pin sequence).
    @param   wr        Arduino pin # for write strobe (required).
    @param   dc        Arduino pin # for data/command select (required).
    @param   cs        Arduino pin # for chip-select (optional, -1 if unused,
                       tie CS low).
    @param   rst       Arduino pin # for display reset (optional, display reset
                       can be tied to MCU reset, default of -1 means unused).
    @param   rd        Arduino pin # for read strobe (optional, -1 if unused).
    @note    Output pins are not initialized; application typically will need
             to call subclass' begin() function, which in turn calls this
             library's initSPI() function to initialize pins.
             Yes, the name is a misnomer...this library originally handled
             only SPI displays, parallel being a recent addition (but not
             wanting to break existing code).
*/
int Adafruit_SPITFT(uint16_t w, uint16_t h, enum tftBusWidth busWidth,
                int8_t d0, int8_t wr, int8_t dc, int8_t cs,
                int8_t rst, int8_t rd) {
  
#if defined(USE_FAST_PINIO)
#if defined(HAS_PORT_SET_CLR)
#if defined(CORE_TEENSY)
  tft8.wrPortSet = portSetRegister(wr);
  tft8.wrPortClr = portClearRegister(wr);
#if !defined(KINETISK)
  dcPinMask = digitalPinToBitMask(dc);
#endif
  dcPortSet = portSetRegister(dc);
  dcPortClr = portClearRegister(dc);
  if (cs >= 0) {
#if !defined(KINETISK)
    csPinMask = digitalPinToBitMask(cs);
#endif
    csPortSet = portSetRegister(cs);
    csPortClr = portClearRegister(cs);
  } else { // see comments below
#if !defined(KINETISK)
    csPinMask = 0;
#endif
    csPortSet = dcPortSet;
    csPortClr = dcPortClr;
  }
  if (rd >= 0) { // if read-strobe pin specified...
#if defined(KINETISK)
    tft8.rdPinMask = 1;
#else // !KINETISK
    tft8.rdPinMask = digitalPinToBitMask(rd);
#endif
    tft8.rdPortSet = portSetRegister(rd);
    tft8.rdPortClr = portClearRegister(rd);
  } else {
    tft8.rdPinMask = 0;
    tft8.rdPortSet = dcPortSet;
    tft8.rdPortClr = dcPortClr;
  }
  // These are all uint8_t* pointers -- elsewhere they're recast
  // as necessary if a 'wide' 16-bit interface is in use.
  tft8.writePort = portOutputRegister(d0);
  tft8.readPort = portInputRegister(d0);
  tft8.dirSet = portModeRegister(d0);
  tft8.dirClr = portModeRegister(d0);
#else  // !CORE_TEENSY
  tft8.wrPinMask = digitalPinToBitMask(wr);
  tft8.wrPortSet = &(PORT->Group[g_APinDescription[wr].ulPort].OUTSET.reg);
  tft8.wrPortClr = &(PORT->Group[g_APinDescription[wr].ulPort].OUTCLR.reg);
  dcPinMask = digitalPinToBitMask(dc);
  dcPortSet = &(PORT->Group[g_APinDescription[dc].ulPort].OUTSET.reg);
  dcPortClr = &(PORT->Group[g_APinDescription[dc].ulPort].OUTCLR.reg);
  if (cs >= 0) {
    csPinMask = digitalPinToBitMask(cs);
    csPortSet = &(PORT->Group[g_APinDescription[cs].ulPort].OUTSET.reg);
    csPortClr = &(PORT->Group[g_APinDescription[cs].ulPort].OUTCLR.reg);
  } else {
    // No chip-select line defined; might be permanently tied to GND.
    // Assign a valid GPIO register (though not used for CS), and an
    // empty pin bitmask...the nonsense bit-twiddling might be faster
    // than checking _cs and possibly branching.
    csPortSet = dcPortSet;
    csPortClr = dcPortClr;
    csPinMask = 0;
  }
  if (rd >= 0) { // if read-strobe pin specified...
    tft8.rdPinMask = digitalPinToBitMask(rd);
    tft8.rdPortSet = &(PORT->Group[g_APinDescription[rd].ulPort].OUTSET.reg);
    tft8.rdPortClr = &(PORT->Group[g_APinDescription[rd].ulPort].OUTCLR.reg);
  } else {
    tft8.rdPinMask = 0;
    tft8.rdPortSet = dcPortSet;
    tft8.rdPortClr = dcPortClr;
  }
  // Get pointers to PORT write/read/dir bytes within 32-bit PORT
  uint8_t dBit = g_APinDescription[d0].ulPin; // d0 bit # in PORT
  PortGroup *p = (&(PORT->Group[g_APinDescription[d0].ulPort]));
  uint8_t offset = dBit / 8; // d[7:0] byte # within PORT
  if (tft8.wide)
    offset &= ~1; // d[15:8] byte # within PORT
  // These are all uint8_t* pointers -- elsewhere they're recast
  // as necessary if a 'wide' 16-bit interface is in use.
  tft8.writePort = (volatile uint8_t *)&(p->OUT.reg) + offset;
  tft8.readPort = (volatile uint8_t *)&(p->IN.reg) + offset;
  tft8.dirSet = (volatile uint8_t *)&(p->DIRSET.reg) + offset;
  tft8.dirClr = (volatile uint8_t *)&(p->DIRCLR.reg) + offset;
#endif // end !CORE_TEENSY
#else  // !HAS_PORT_SET_CLR
  tft8.wrPort = (PORTreg_t)portOutputRegister(digitalPinToPort(wr));
  tft8.wrPinMaskSet = digitalPinToBitMask(wr);
  dcPort = (PORTreg_t)portOutputRegister(digitalPinToPort(dc));
  dcPinMaskSet = digitalPinToBitMask(dc);
  if (cs >= 0) {
    csPort = (PORTreg_t)portOutputRegister(digitalPinToPort(cs));
    csPinMaskSet = digitalPinToBitMask(cs);
  } else {
    // No chip-select line defined; might be permanently tied to GND.
    // Assign a valid GPIO register (though not used for CS), and an
    // empty pin bitmask...the nonsense bit-twiddling might be faster
    // than checking _cs and possibly branching.
    csPort = dcPort;
    csPinMaskSet = 0;
  }
  if (rd >= 0) { // if read-strobe pin specified...
    tft8.rdPort = (PORTreg_t)portOutputRegister(digitalPinToPort(rd));
    tft8.rdPinMaskSet = digitalPinToBitMask(rd);
  } else {
    tft8.rdPort = dcPort;
    tft8.rdPinMaskSet = 0;
  }
  csPinMaskClr = ~csPinMaskSet;
  dcPinMaskClr = ~dcPinMaskSet;
  tft8.wrPinMaskClr = ~tft8.wrPinMaskSet;
  tft8.rdPinMaskClr = ~tft8.rdPinMaskSet;
  tft8.writePort = (PORTreg_t)portOutputRegister(digitalPinToPort(d0));
  tft8.readPort = (PORTreg_t)portInputRegister(digitalPinToPort(d0));
  tft8.portDir = (PORTreg_t)portModeRegister(digitalPinToPort(d0));
#endif // end !HAS_PORT_SET_CLR
#endif // end USE_FAST_PINIO
  
  return 1;
  }

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9341 driver with software SPI
    @param    cs    Chip select pin #
    @param    dc    Data/Command pin #
    @param    mosi  SPI MOSI pin #
    @param    sclk  SPI Clock pin #
    @param    rst   Reset pin # (optional, pass -1 if unused)
    @param    miso  SPI MISO pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
int Adafruit_ILI9341_SPISW(int8_t cs, int8_t dc, int8_t mosi,
                           int8_t sclk, int8_t rst, int8_t miso) {
  
  return Adafruit_SPITFT(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT, 0, cs, dc, mosi, sclk,
                      rst, miso); 
  }

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9341 driver with hardware SPI using the
            default SPI peripheral.
    @param  cs   Chip select pin # (OK to pass -1 if CS tied to GND).
    @param  dc   Data/Command pin # (required).
    @param  rst  Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
int Adafruit_ILI9341_SPIHW(int8_t cs, int8_t dc, int8_t rst) {
  
  return Adafruit_SPITFT(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT, 0, cs, dc, rst, 0, 0, 0);
  }

#if !defined(ESP8266)
/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9341 driver with hardware SPI using
            a specific SPI peripheral (not necessarily default).
    @param  spiClass  Pointer to SPI peripheral (e.g. &SPI or &SPI1).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and
                      CS is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
/*Adafruit_ILI9341::Adafruit_ILI9341(SPIClass *spiClass, int8_t dc, int8_t cs,
                                   int8_t rst)
    : Adafruit_SPITFT(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT, spiClass, cs, dc,
                      rst) {}
 * */
#endif // end !ESP8266

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9341 driver using parallel interface.
    @param  busWidth  If tft16 (enumeration in Adafruit_SPITFT.h), is a
                      16-bit interface, else 8-bit.
    @param  d0        Data pin 0 (MUST be a byte- or word-aligned LSB of a
                      PORT register -- pins 1-n are extrapolated from this).
    @param  wr        Write strobe pin # (required).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and CS
                      is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
    @param  rd        Read strobe pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
int Adafruit_ILI9341_8(enum tftBusWidth busWidth, int8_t d0, int8_t wr,
                                   int8_t dc, int8_t cs, int8_t rst, int8_t rd) {
  
#if defined(__PIC32MM__)
#else  
  TRISEbits.TRISE6=0;
  TRISEbits.TRISE7=0;
  TRISBbits.TRISB13=0;
  TRISBbits.TRISB14=0;
  TRISBbits.TRISB15=0;
  TRISFbits.TRISF4=0;
  TRISFbits.TRISF5=0;
  TRISEbits.TRISE5=0;
  TRISBbits.TRISB2=0;
  TRISBbits.TRISB3=0;
  TRISBbits.TRISB4=0;
  TRISBbits.TRISB5=0;
  TRISBbits.TRISB10=0;
#endif
  

  CNPUB = 0x0000;   // OCCHIO agli altri usi...
  CNPDB = 0x0000;
  ANSELB= 0x0000;
  m_LCDRSTBit=0;
  LATBbits.LATB2=1;     // CS 
  LATBbits.LATB4=LATBbits.LATB5=1;     // WR; RD non usato circa
  __delay_ms(200);
  ClrWdt();
  m_LCDRSTBit=1;
  return Adafruit_SPITFT(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT, busWidth, d0, wr, dc,
                      cs, rst, rd); 
  }

// clang-format off
static const uint8_t PROGMEM initcmd[] = {
//  0xEF, 3, 0x03, 0x80, 0x02, boh cosa voleva essere??
  ILI9341_PWCTRB /*0xCF*/, 3, 0x00, /*0x81 da doc*/ 0xC1, 0x30,
  ILI9341_PWRONSEQ /*0xED*/, 4, 0x64, 0x03, 0x12, 0x81,
  ILI9341_DRVTIMA /*0xE8*/, 3, 0x85, 0x00, 0x78,
  ILI9341_PWCTRA /*0xCB*/, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  ILI9341_PUMPCTRL /*0xF7*/, 1, 0x20,
  ILI9341_DRVTIMB /*0xEA*/, 2, 0x00, 0x00,
  ILI9341_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  ILI9341_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
  ILI9341_VMCTR2  , 1, 0x86,             // VCM control2
  ILI9341_MADCTL  , 1, MADCTL_MX | MADCTL_MV /*| MADCTL_BGR*/ /*0x60*/,      // Memory Access Control (0x60 = top-left LANDSCAPE; 8=inverte RGB-BGR) v. anche setRotation
  ILI9341_VSCRSADD, 1, 0x00,             // Vertical scroll zero
  ILI9341_PIXFMT  , 1, 0x55,             // 16bit/pixel
  ILI9341_FRMCTR1 , 2, 0x00, 0x18,        // il doc mette 1B...
  ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control QUA IL DOC dice che ci sarebbe un 4° parm, che non si sa come impostare... PCDIV
  ILI9341_ENABLE3G /*0xF2*/, 1, 0x00,     // 3Gamma Function Disable
  ILI9341_GAMMASET , 1, 0x01,             // Gamma curve selected
  ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT  , 0x80,                // Exit Sleep
  ILI9341_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
  };
// clang-format on

/**************************************************************************/
/*!
    @brief   Initialize ILI9341 chip
    Connects to the ILI9341 over SPI and sends initialization procedure commands
    @param    freq  Desired SPI clock frequency
*/
/**************************************************************************/
void begin(uint32_t freq) {

  if(!freq)
    freq = SPI_DEFAULT_FREQ;
//  initSPI(freq);

/*  if(_rst < 0) {                 // If no hardware reset pin...
    sendCommand(ILI9341_SWRESET); // Engage software reset
    delay(150);
    } */

  uint8_t cmd, x, numArgs;
  const uint8_t *addr = initcmd;
  while((cmd = pgm_read_byte(addr++)) > 0) {
    x = pgm_read_byte(addr++);
    numArgs = x & 0x7F;
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;
    if(x & 0x80)
      __delay_ms(150);
    }

  _width = ILI9341_TFTWIDTH;
  _height = ILI9341_TFTHEIGHT;
  }

/**************************************************************************/
/*!
    @brief   Set origin of (0,0) and orientation of TFT display
    @param   m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void setRotation(uint8_t m) {
  
  rotation = m % 4; // can't be higher than 3
  switch(rotation) {
    case 0:
      m = (MADCTL_MX /*| MADCTL_BGR*/);   // perché CAZZO mettevano BGR che poi i colori definiti sono RGB??! f* ada die ivrea ;)
      _width = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 1:
      m = (MADCTL_MV /*| MADCTL_BGR*/);
      _width = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
    case 2:
      m = (MADCTL_MY /*| MADCTL_BGR*/);
      _width = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 3:
      m = (MADCTL_MX | MADCTL_MY | MADCTL_MV /*| MADCTL_BGR*/);
      _width = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
    }

  sendCommand(ILI9341_MADCTL, &m, 1);
  }

/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void invertDisplay(BOOL invert) {
  
  sendCommand(invert ? ILI9341_INVON : ILI9341_INVOFF, NULL, 0);
  }

/**************************************************************************/
/*!
    @brief   Scroll display memory
    @param   y How many pixels to scroll display by
*/
/**************************************************************************/
void scrollTo(uint16_t y) {
  uint8_t data[2];
  
  data[0] = y >> 8;
  data[1] = y & 0xff;
  sendCommand(ILI9341_VSCRSADD, (uint8_t *)data, 2);
  }

/**************************************************************************/
/*!
    @brief   Set the height of the Top and Bottom Scroll Margins
    @param   top The height of the Top scroll margin
    @param   bottom The height of the Bottom scroll margin
 */
/**************************************************************************/
void setScrollMargins(uint16_t top, uint16_t bottom) {
  
  // TFA+VSA+BFA must equal 320
  if(top + bottom <= ILI9341_TFTHEIGHT) {
    uint16_t middle = ILI9341_TFTHEIGHT - top + bottom;
    uint8_t data[6];
    data[0] = top >> 8;
    data[1] = top & 0xff;
    data[2] = middle >> 8;
    data[3] = middle & 0xff;
    data[4] = bottom >> 8;
    data[5] = bottom & 0xff;
    sendCommand(ILI9341_VSCRDEF, (uint8_t *)data, 6);
    }
  }


/*!
    @brief  Issue a single 16-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the word. Despite the name,
            this function is used even if display connection is parallel;
            name was maintained for backward compatibility. Naming is also
            not consistent with the 8-bit version, spiWrite(). Sorry about
            that. Again, staying compatible with outside code.
    @param  w  16-bit value to write.
*/
void SPI_WRITE16(uint16_t w) {

#if defined(__AVR__)
    *tft8.writePort = w >> 8;
    TFT_WR_STROBE();
    *tft8.writePort = w;
#elif defined(USE_FAST_PINIO)
    if(!tft8.wide) {
      *tft8.writePort = w >> 8;
      TFT_WR_STROBE();
      *tft8.writePort = w;
      } 
    else {
      *(volatile uint16_t *)tft8.writePort = w;
      }
#endif
  WRITE_LATB(w >> 8);     // v. ENDIAN bit di Interface Mode (0xF6)
  TFT_WR_STROBE();
  WRITE_LATB(w);
  TFT_WR_STROBE();
  }

/**************************************************************************/
/*!
    @brief   Set the "address window" - the rectangle we will write to RAM with
   the next chunk of      SPI data writes. The ILI9341 will automatically wrap
   the data as each row is filled
    @param   x1  TFT memory 'x' origin
    @param   y1  TFT memory 'y' origin
    @param   w   Width of rectangle
    @param   h   Height of rectangle
*/
/**************************************************************************/
void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
  uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);
  
  writeCommand(ILI9341_CASET); // Column address set
  SPI_WRITE16(x1);
  SPI_WRITE16(x2);
  writeCommand(ILI9341_PASET); // Row address set
  SPI_WRITE16(y1);
  SPI_WRITE16(y2);
  writeCommand(ILI9341_RAMWR); // Write to RAM
  }


/*!
@brief   Send Command handles complete sending of commands and data
@param   commandByte       The Command Byte
@param   dataBytes         A pointer to the Data bytes to send
@param   numDataBytes      The number of bytes we should send
*/
void sendCommand(uint8_t commandByte, uint8_t *dataBytes, uint8_t numDataBytes) {
  int i;
  
  SPI_BEGIN_TRANSACTION();
  START_WRITE();
  SPI_DC_LOW();          // Command mode
//  spiWrite(commandByte); // Send the command byte
#if defined(__AVR__)
  *tft8.writePort = b;
#elif defined(USE_FAST_PINIO)
  if (!tft8.wide)
    *tft8.writePort = b;
  else
    *(volatile uint16_t *)tft8.writePort = b;
#endif
  WRITE_LATB(commandByte);
  TFT_WR_STROBE();

  SPI_DC_HIGH();
  for(i=0; i<numDataBytes; i++) {
/*    if(tft8.wide) {
      SPI_WRITE16(*(uint16_t *)dataBytes);
      dataBytes += 2;
      } 
    else {*/
#if defined(__AVR__)
  *tft8.writePort = b;
#elif defined(USE_FAST_PINIO)
  if (!tft8.wide)
    *tft8.writePort = b;
  else
    *(volatile uint16_t *)tft8.writePort = b;
#endif
    WRITE_LATB(*dataBytes);
    TFT_WR_STROBE();
  
    dataBytes++;
//      }
    }

  END_WRITE();
  SPI_END_TRANSACTION();
  }

/*!
 @brief  sendCommand16 handles complete sending of
         commands and data for 16-bit parallel displays. Currently somewhat
         rigged for the NT35510, which has the odd behavior of wanting
         commands 16-bit, but subsequent data as 8-bit values, despite
         the 16-bit bus (high byte is always 0). Also seems to require
         issuing and incrementing address with each transfer.
 @param  commandWord   The command word (16 bits)
 @param  dataBytes     A pointer to the data bytes to send
 @param  numDataBytes  The number of bytes we should send
 */
void sendCommand16(uint16_t commandWord, const uint8_t *dataBytes,
                  uint8_t numDataBytes) {
  int i;
  
  SPI_BEGIN_TRANSACTION();
  START_WRITE();

  if(numDataBytes == 0) {
    SPI_DC_LOW();             // Command mode
    SPI_WRITE16(commandWord); // Send the command word
    SPI_DC_HIGH();            // Data mode
    }
  for(i=0; i<numDataBytes; i++) {
    SPI_DC_LOW();             // Command mode
    SPI_WRITE16(commandWord); // Send the command word
    SPI_DC_HIGH();            // Data mode
    commandWord++;
    SPI_WRITE16((uint16_t)pgm_read_byte(dataBytes++));
    }

  END_WRITE();
  SPI_END_TRANSACTION();
  }


/*!
 @brief   Read 8 bits of data from display configuration memory (not RAM).
 This is highly undocumented/supported and should be avoided,
 function is only included because some of the examples use it.
 @param   commandByte
 The command register to read data from.
 @param   index
 The byte index into the command to read from.
 @return  Unsigned 8-bit data read from display register.
 */
/**************************************************************************/
uint8_t _readcommand8(uint8_t commandByte, uint8_t index) {
  uint8_t result;
  
  START_WRITE();
  SPI_DC_LOW(); // Command mode
#if defined(__AVR__)
  *tft8.writePort = b;
#elif defined(USE_FAST_PINIO)
  if (!tft8.wide)
    *tft8.writePort = b;
  else
    *(volatile uint16_t *)tft8.writePort = b;
#endif
  WRITE_LATB(commandByte);
  TFT_WR_STROBE();
//  spiWrite(commandByte);
    
  SPI_DC_HIGH(); // Data mode
  do {
//    result = spiRead();
    
//    if(!tft8.wide) {                             // 8-bit TFT connection
#if defined(HAS_PORT_SET_CLR)
      *tft8.dirClr = 0xFF;                        // Set port to input state
      w = *tft8.readPort;                         // Read value from port
      *tft8.dirSet = 0xFF;                        // Restore port to output
#else  // !HAS_PORT_SET_CLR
//      *tft8.portDir = 0xFF;                        // Set port to input state
      READ_PORTB(result);                          // Read value from port
//      *tft8.portDir = 0x00;                        // Restore port to output
#endif // end HAS_PORT_SET_CLR
/*      } 
    else {                                      // 16-bit TFT connection
#if defined(HAS_PORT_SET_CLR)
      *(volatile uint16_t *)tft8.dirClr = 0xFFFF; // Input state
      w = *(volatile uint16_t *)tft8.readPort;    // 16-bit read
      *(volatile uint16_t *)tft8.dirSet = 0xFFFF; // Output state
#else  // !HAS_PORT_SET_CLR
      *(volatile uint16_t *)tft8.portDir = 0x0000; // Input state
      w = *(volatile uint16_t *)tft8.readPort;     // 16-bit read
      *(volatile uint16_t *)tft8.portDir = 0xFFFF; // Output state
#endif // end !HAS_PORT_SET_CLR
      }
  */
    TFT_RD_HIGH();                                 // Read line HIGH

    } while(index--); // Discard bytes up to index'th
  END_WRITE();
  return result;
  }

/*!
 @brief   Read 16 bits of data from display register.
          For 16-bit parallel displays only.
 @param   addr  Command/register to access.
 @return  Unsigned 16-bit data.
 */
uint16_t _readcommand16(uint16_t addr) {
  
#if defined(USE_FAST_PINIO) // NOT SUPPORTED without USE_FAST_PINIO
  uint16_t result = 0;
  if ((connection == TFT_PARALLEL) && tft8.wide) {
    startWrite();
    SPI_DC_LOW(); // Command mode
    SPI_WRITE16(addr);
    SPI_DC_HIGH(); // Data mode
    TFT_RD_LOW();  // Read line LOW
#if defined(HAS_PORT_SET_CLR)
    *(volatile uint16_t *)tft8.dirClr = 0xFFFF;   // Input state
    result = *(volatile uint16_t *)tft8.readPort; // 16-bit read
    *(volatile uint16_t *)tft8.dirSet = 0xFFFF;   // Output state
#else                                             // !HAS_PORT_SET_CLR
    *(volatile uint16_t *)tft8.portDir = 0x0000;    // Input state
    result = *(volatile uint16_t *)tft8.readPort;   // 16-bit read
    *(volatile uint16_t *)tft8.portDir = 0xFFFF;    // Output state
#endif                                            // end !HAS_PORT_SET_CLR
    TFT_RD_HIGH();                                // Read line HIGH
    endWrite();
  }
  return result;
#else
  return 0;
#endif // end !USE_FAST_PINIO
  }

/**************************************************************************/
/*!
    @brief  Read 8 bits of data from ILI9341 configuration memory. NOT from RAM!
            This is highly undocumented/supported, it's really a hack but kinda works?
    @param    commandByte  The command register to read data from
    @param    index  The byte index into the command to read from
    @return   Unsigned 8-bit data read from ILI9341 register
 */
/**************************************************************************/
uint8_t readcommand8(uint8_t commandByte, uint8_t index) {
  uint8_t data = 0x10 + index;
  
  sendCommand(0xD9, &data, 1); // Set Index Register
  
  return _readcommand8(commandByte,1);
  }



/*!
    @brief  Issue a single 16-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the word.
            Thus operates ONLY on 'wide' (16-bit) parallel displays!
    @param  w  16-bit value to write.
*/
void _write16(uint16_t w) {
  
#if defined(USE_FAST_PINIO)
    if (tft8.wide)
      *(volatile uint16_t *)tft8.writePort = w;
#endif
  TFT_WR_STROBE();
  }

/*!
    @brief   Read a single 16-bit value from the display. Chip-select and
             transaction must have been previously set -- this ONLY reads
             the byte. This operates ONLY on 'wide' (16-bit) parallel
             displays!
    @return  Unsigned 16-bit value read (always zero if USE_FAST_PINIO is
             not supported by the MCU architecture).
*/
uint16_t _read16(void) {
  uint16_t w=0;
  
#if defined(USE_FAST_PINIO)
  TFT_RD_LOW();    // Read line LOW
  if (tft8.wide) { // 16-bit TFT connection
#if defined(HAS_PORT_SET_CLR)
    *(volatile uint16_t *)tft8.dirClr = 0xFFFF; // Input state
    w = *(volatile uint16_t *)tft8.readPort;    // 16-bit read
    *(volatile uint16_t *)tft8.dirSet = 0xFFFF; // Output state
#else                                               // !HAS_PORT_SET_CLR
    *(volatile uint16_t *)tft8.portDir = 0x0000; // Input state
    w = *(volatile uint16_t *)tft8.readPort;     // 16-bit read
    *(volatile uint16_t *)tft8.portDir = 0xFFFF; // Output state
#endif                                              // end !HAS_PORT_SET_CLR
    }
  TFT_RD_HIGH(); // Read line HIGH
#else                // !USE_FAST_PINIO
  w = 0; // Parallel TFT is NOT SUPPORTED without USE_FAST_PINIO
#endif               // end !USE_FAST_PINIO
  
  return w;
  }

/*!
    @brief  Write a single command word to the display. Chip-select and
            transaction must have been previously set -- this ONLY sets
            the device to COMMAND mode, issues the byte and then restores
            DATA mode. This operates ONLY on 'wide' (16-bit) parallel
            displays!
    @param  cmd  16-bit command to write.
*/
void _writeCommand16(uint16_t cmd) {
  
  SPI_DC_LOW();
  _write16(cmd);
  SPI_DC_HIGH();
  }



void drawPixel(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT16 color) {

	if(boundaryCheck(x,y)) 
		return;
	if((x < 0) || (y < 0)) 
		return;
	//setAddr(x,y,x+1,y+1);
	Pixel(x, y, color);
	}


void drawFastVLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T h, UINT16 color) {

	// Rudimentary clipping
	if(boundaryCheck(x,y))
		return;
	if(((y + h) - 1) >= _height) 
		h = _height-y;
//	setAddr(x,y,x,(y+h)-1);
	VLine(x, y, h, color);
	}

BOOL boundaryCheck(UGRAPH_COORD_T x,UGRAPH_COORD_T y) {

	if((x >= _width) || (y >= _height)) 
		return TRUE;
	return FALSE;
	}

void drawFastHLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UINT16 color) {

	// Rudimentary clipping
	if(boundaryCheck(x,y)) 
		return;
	if(((x+w) - 1) >= _width)  
		w = _width-x;
//	setAddr(x,y,(x+w)-1,y);
	HLine(x, y, w, color);
	}


// fill a rectangle
void fillRect(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color) {

	if(boundaryCheck(x,y)) 
		return;
	if(((x + w) - 1) >= _width)  
		w = _width  - x;
	if(((y + h) - 1) >= _height) 
		h = _height - y;
  START_WRITE();
	setAddrWindow(x,y,w,h);
	for(y=h; y>0; y--) {
		for(x=w; x>1; x--) {
			writedata16(color);
			}
		writedata16(color);
		}
  END_WRITE();
	}

inline void writeFillRectPreclipped(int16_t x, int16_t y,
  int16_t w, int16_t h, UINT16 color) {
  
  setAddrWindow(x, y, w, h);
  writeColor(color, (uint32_t)w * h);
  }

void drawLine(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0,UGRAPH_COORD_T x1, UGRAPH_COORD_T y1, UINT16 color) {
	BOOL steep;
	GRAPH_COORD_T dx,dy;
	GRAPH_COORD_T err;
	GRAPH_COORD_T ystep;
	GRAPH_COORD_T xbegin;

	if(y0==y1) {
		if(x1>x0) {
			drawFastHLine(x0, y0, x1-x0+1, color);
			} 
		else if(x1 < x0) {
			drawFastHLine(x1, y0, x0-x1+1, color);
			} 
		else {
			drawPixel(x0, y0, color);
			}
		return;
		} 
	else if(x0==x1) {
		if(y1>y0) {
			drawFastVLine(x0, y0, y1-y0+1, color);
			} 
		else {
			drawFastVLine(x0, y1, y0-y1+1, color);
			}
		return;
		}

	steep = abs(y1-y0) > abs(x1-x0);
	if(steep) {
		_swap(&x0, &y0);
		_swap(&x1, &y1);
		}
	if(x0>x1) {
		_swap(&x0, &x1);
		_swap(&y0, &y1);
		}

	dx = x1-x0;
	dy = abs(y1-y0);

	err = dx/2;

	if(y0<y1) {
		ystep = 1;
		} 
	else {
		ystep = -1;
		}

	
	xbegin = x0;
	if(steep) {
		for(; x0<=x1; x0++) {
			err -= dy;
			if(err < 0) {
				int16_t len = x0-xbegin;
				if(len) {
					VLine(y0, xbegin, len + 1, color);
					} 
				else {
					Pixel(y0, x0, color);
					}
				xbegin = x0+1;
				y0 += ystep;
				err += dx;
				}
			}
		if (x0 > xbegin + 1) {
			VLine(y0, xbegin, x0 - xbegin, color);
			}
		} 
	else {
		for(; x0<=x1; x0++) {
			err -= dy;
			if(err < 0) {
				int16_t len = x0-xbegin;
				if(len) {
					HLine(xbegin, y0, len+1, color);
					} 
				else {
					Pixel(x0, y0, color);
					}
				xbegin = x0+1;
				y0 += ystep;
				err += dx;
				}
			}
		if(x0 > xbegin+1) {
			HLine(xbegin, y0, x0-xbegin, color);
			}
		}
//	writecommand(CMD_NOP);
	}


void drawRect(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color) {

	HLine(x, y, w, color);
	HLine(x, y+h-1, w, color);
	VLine(x, y, h, color);
	VLine(x+w-1, y, h, color);
//	writecommand(CMD_NOP);
	}

void fillScreen(UINT16 color) {
	uint16_t px,py;

  START_WRITE();
	setAddrWindow(0,0,_width,_height);
	for(py=0; py<_height; py++) {
    for(px=0; px<_width; px++) {
      writedata16(color);
#ifdef USA_SPI_HW
      ClrWdt();
#endif
  		}
		}
  END_WRITE();
//	writecommand(CMD_NOP);
	}

void __attribute__((always_inline)) HLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UINT16 color) {

  START_WRITE();
	setAddrWindow(x, y, w, 1);
	do { writedata16(color); } while (--w > 0);
  END_WRITE();
	}

void __attribute__((always_inline)) VLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T h, UINT16 color) {

  START_WRITE();
	setAddrWindow(x, y, 1, h);
	do { writedata16(color); } while (--h > 0);
  END_WRITE();
	}
		
void __attribute__((always_inline)) Pixel(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT16 color) {

  START_WRITE();
	setAddrWindow(x, y, 1, 1);
	writedata16(color);
  END_WRITE();
	}

/*!
    @brief  Draw a single pixel to the display at requested coordinates.
            Not self-contained; should follow a START_WRITE() call.
    @param  x      Horizontal position (0 = left).
    @param  y      Vertical position   (0 = top).
    @param  color  16-bit pixel color in '565' RGB format.
*/
void writePixel(int16_t x, int16_t y, UINT16 color) {
  
  if((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) {
    START_WRITE();
    setAddrWindow(x, y, 1, 1);
    SPI_WRITE16(color);
    END_WRITE();
    }
  }

/*!
    @brief  Issue a series of pixels from memory to the display. Not self-
            contained; should follow START_WRITE() and setAddrWindow() calls.
    @param  colors     Pointer to array of 16-bit pixel values in '565' RGB
                       format.
    @param  len        Number of elements in 'colors' array.
    @param  block      If true (default case if unspecified), function blocks
                       until DMA transfer is complete. This is simply IGNORED
                       if DMA is not enabled. If false, the function returns
                       immediately after the last DMA transfer is started,
                       and one should use the dmaWait() function before
                       doing ANY other display-related activities (or even
                       any SPI-related activities, if using an SPI display
                       that shares the bus with other devices).
    @param  bigEndian  If using DMA, and if set true, bitmap in memory is in
                       big-endian order (most significant byte first). By
                       default this is false, as most microcontrollers seem
                       to be little-endian and 16-bit pixel values must be
                       byte-swapped before issuing to the display (which tend
                       to be big-endian when using SPI or 8-bit parallel).
                       If an application can optimize around this -- for
                       example, a bitmap in a uint16_t array having the byte
                       values already reordered big-endian, this can save
                       some processing time here, ESPECIALLY if using this
                       function's non-blocking DMA mode. Not all cases are
                       covered...this is really here only for SAMD DMA and
                       much forethought on the application side.
*/
void writePixels(uint16_t *colors, uint32_t len,
  BOOL block, BOOL bigEndian) {

  if(!len) 
    return; // Avoid 0-byte transfers

  // All other cases (bitbang SPI or non-DMA hard SPI or parallel),
  // use a loop with the normal 16-bit data write function:
  while(len--) {
    SPI_WRITE16(*colors++);
    }
  
  }

/*!
    @brief  Issue a series of pixels, all the same color. Not self-
            contained; should follow START_WRITE() and setAddrWindow() calls.
    @param  color  16-bit pixel color in '565' RGB format.
    @param  len    Number of pixels to draw.
*/
void writeColor(UINT16 color, uint32_t len) {

  if(!len) 
    return; // Avoid 0-byte transfers

  uint8_t hi = color >> 8, lo = color;
  uint16_t b;

  do {
    uint32_t pixelsThisPass = len;
    if(pixelsThisPass > 20000)
      pixelsThisPass = 20000;
    len -= pixelsThisPass;
//    yield(); // Periodic yield() on long fills
    while(pixelsThisPass--) {
      SPI_WRITE16(color);
      }
    } while(len);
  }


void writedata16(uint16_t d) {

  SPI_DC_HIGH();
  SPI_WRITE16(d);
	} 

void writedata16x2(uint16_t d1,uint16_t d2) {

  SPI_DC_HIGH();
  SPI_WRITE16(d1);
  SPI_WRITE16(d2);
	} 

void homeAddress(void) {
	
	setAddrWindow(0,0,_width,_height);
	}


void setCursor(UGRAPH_COORD_T x, UGRAPH_COORD_T y) {

	if(boundaryCheck(x,y)) 
		return;
	setAddrWindow(0,0,x,y);
	cursor_x = x;
	cursor_y = y;
	}

void _swap(UGRAPH_COORD_T *a, UGRAPH_COORD_T *b) {
	UGRAPH_COORD_T t = *a; 
	*a = *b; 
	*b = t; 
	}


extern const unsigned char logo_msx[];

void drawBG(void) {
	char buffer[22];

	gfx_drawRect(0,0,_TFTWIDTH-1,_TFTHEIGHT-1,GREEN);
	gfx_drawRect(1,1,_TFTWIDTH-3,_TFTHEIGHT-3,LIGHTGREEN);

	setTextSize(1);
	setTextColor(BRIGHTGREEN);
	LCDWrite(buffer);
	LCDXY(14,2);
	LCDWrite(CopyrightString);


	setTextColor(BRIGHTCYAN);
	LCDXY(22,28);
	sprintf(buffer,"booting...");
	LCDWrite(buffer);

#ifdef MSX  
  drawBitmap4(30,34,logo_msx);
#endif
  
	ClrWdt();
  __delay_ms(1000); 
	ClrWdt();

	}




//retrocompatibilità con lcd.c ecc
BYTE LCDInit(void) {		// su SPI

  Adafruit_ILI9341_8(8, 9, 10, 11, 12, 13, 14);
	begin(0);
  __delay_ms(200);
  
  ClrWdt();
  
  //setRotation();    // fare questo al posto di toccare WIDTH e MADCTL? ma quella che voglio io top-left non ce l'hanno...
  
#if defined(m_LCDBLBit)
  m_LCDBLBit=1;
#endif
  
	LCDCls();

// init done
	setTextWrap(1);
//	setTextColor2(WHITE, BLACK);

	drawBG();
  
  ShortDelay(250000);
  ClrWdt();
  ShortDelay(250000);
  ClrWdt();

	return 1;
	}

void LCDXY(BYTE x, BYTE y) {	setCursor(((UGRAPH_COORD_T)x)*textsize*6,((UGRAPH_COORD_T)y)*textsize*8);	}

void LCDCls() {	clearScreen();	LCDXY(0,0); /*display();*/ }	// serve?? 

void LCDWrite(const char *s) {
  
  while(*s)
    cwrite(*s++);
  }

