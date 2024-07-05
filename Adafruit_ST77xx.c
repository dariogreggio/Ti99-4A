/**************************************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  Works with the Adafruit 1.8" TFT Breakout w/SD card
    ----> http://www.adafruit.com/products/358
  The 1.8" TFT shield
    ----> https://www.adafruit.com/product/802
  The 1.44" TFT breakout
    ----> https://www.adafruit.com/product/2088
  as well as Adafruit raw 1.8" TFT display
    ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams.
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional).

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/

#include "Adafruit_ST77xx.h"
#include <limits.h>
#ifndef ARDUINO_STM32_FEATHER
//  #include "pins_arduino.h"
//  #include "wiring_private.h"
#endif
//#include "io_cfg.h"
//#include "SPI.h"


WORD invertOnCommand,invertOffCommand;
WORD _xstart,_ystart;
DWORD _freq;


uint8_t _colstart = 0, ///< Some displays need this changed to offset
        _rowstart = 0, ///< Some displays need this changed to offset
        spiMode   = SPI_MODE0; ///< Certain display needs MODE3 instead

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ST77XX driver with software SPI
    @param  w     Display width in pixels at default rotation setting (0)
    @param  h     Display height in pixels at default rotation setting (0)
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  mosi  SPI MOSI pin #
    @param  sclk  SPI Clock pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
    @param  miso  SPI MISO pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
int Adafruit_ST77xx_1(uint16_t w, uint16_t h, int8_t cs,
  int8_t dc, int8_t mosi, int8_t sclk, int8_t rst, int8_t miso) {
  
  Adafruit_SPITFT_1(w, h, cs, dc, mosi, sclk, rst, miso);
  }

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ST77XX driver with hardware SPI
    @param  w     Display width in pixels at default rotation setting (0)
    @param  h     Display height in pixels at default rotation setting (0)
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
int Adafruit_ST77xx_2(uint16_t w, uint16_t h, int8_t cs,
  int8_t dc, int8_t rst) {
  
  Adafruit_SPITFT_2(w, h, cs, dc, rst);
  }

#if !defined(ESP8266)
/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ST77XX driver with selectable hardware SPI
    @param  w     Display width in pixels at default rotation setting (0)
    @param  h     Display height in pixels at default rotation setting (0)
    @param  spiClass A pointer to an SPI device to use (e.g. &SPI1)
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
/*Adafruit_ST77xx_SPI(uint16_t w, uint16_t h, SPIClass *spiClass,
  int8_t cs, int8_t dc, int8_t rst) {
  
  Adafruit_SPITFT(w, h, spiClass, cs,  dc, rst);
}
 * */
#endif // end !ESP8266




// Possible values for Adafruit_SPITFT.connection:
#define TFT_HARD_SPI 0  ///< Display interface = hardware SPI
#define TFT_SOFT_SPI 1  ///< Display interface = software SPI
#define TFT_PARALLEL 2  ///< Display interface = 8- or 16-bit parallel


// CONSTRUCTORS ------------------------------------------------------------

/*!
    @brief   Adafruit_SPITFT constructor for software (bitbang) SPI.
    @param   w     Display width in pixels at default rotation setting (0).
    @param   h     Display height in pixels at default rotation setting (0).
    @param   cs    Arduino pin # for chip-select (-1 if unused, tie CS low).
    @param   dc    Arduino pin # for data/command select (required).
    @param   mosi  Arduino pin # for bitbang SPI MOSI signal (required).
    @param   sck   Arduino pin # for bitbang SPI SCK signal (required).
    @param   rst   Arduino pin # for display reset (optional, display reset
                   can be tied to MCU reset, default of -1 means unused).
    @param   miso  Arduino pin # for bitbang SPI MISO signal (optional,
                   -1 default, many displays don't support SPI read).
    @return  Adafruit_SPITFT object.
    @note    Output pins are not initialized; application typically will
             need to call subclass' begin() function, which in turn calls
             this library's initSPI() function to initialize pins.
*/
void Adafruit_SPITFT_1(uint16_t w, uint16_t h,
  int8_t cs, int8_t dc, int8_t mosi, int8_t sck, int8_t rst, int8_t miso) {
  
  Adafruit_GFX(w, h);
/*
  connection=TFT_SOFT_SPI;
  _rst=rst;
  _cs=cs;
  _dc=dc;
  
    swspi._sck  = sck;
    swspi._mosi = mosi;
    swspi._miso = miso;
 */   
  
  Adafruit_ST77xx_begin(0); 
  
  }

/*!
    @brief   Adafruit_SPITFT constructor for hardware SPI using the board's
             default SPI peripheral.
    @param   w     Display width in pixels at default rotation setting (0).
    @param   h     Display height in pixels at default rotation setting (0).
    @param   cs    Arduino pin # for chip-select (-1 if unused, tie CS low).
    @param   dc    Arduino pin # for data/command select (required).
    @param   rst   Arduino pin # for display reset (optional, display reset
                   can be tied to MCU reset, default of -1 means unused).
    @return  Adafruit_SPITFT object.
    @note    Output pins are not initialized; application typically will
             need to call subclass' begin() function, which in turn calls
             this library's initSPI() function to initialize pins.
*/
#if defined(ESP8266) // See notes below
Adafruit_SPITFT::Adafruit_SPITFT(uint16_t w, uint16_t h, int8_t cs,
  int8_t dc, int8_t rst) : Adafruit_GFX(w, h),
  connection(TFT_HARD_SPI), _rst(rst), _cs(cs), _dc(dc) {
    hwspi._spi = &SPI;
}
#else  // !ESP8266
void Adafruit_SPITFT_2(uint16_t w, uint16_t h, int8_t cs, int8_t dc, int8_t rst) {
  
//  Adafruit_SPITFT_1(w, h, &SPI, cs, dc, rst);
    // This just invokes the hardware SPI constructor below,
    // passing the default SPI device (&SPI).
  }
#endif // end !ESP8266

#if !defined(ESP8266)
// ESP8266 compiler freaks out at this constructor -- it can't disambiguate
// beteween the SPIClass pointer (argument #3) and a regular integer.
// Solution here it to just not offer this variant on the ESP8266. You can
// use the default hardware SPI peripheral, or you can use software SPI,
// but if there's any library out there that creates a 'virtual' SPIClass
// peripheral and drives it with software bitbanging, that's not supported.
/*!
    @brief   Adafruit_SPITFT constructor for hardware SPI using a specific
             SPI peripheral.
    @param   w         Display width in pixels at default rotation (0).
    @param   h         Display height in pixels at default rotation (0).
    @param   spiClass  Pointer to SPIClass type (e.g. &SPI or &SPI1).
    @param   cs        Arduino pin # for chip-select (-1 if unused, tie CS low).
    @param   dc        Arduino pin # for data/command select (required).
    @param   rst       Arduino pin # for display reset (optional, display reset
                       can be tied to MCU reset, default of -1 means unused).
    @return  Adafruit_SPITFT object.
    @note    Output pins are not initialized in constructor; application
             typically will need to call subclass' begin() function, which
             in turn calls this library's initSPI() function to initialize
             pins. EXCEPT...if you have built your own SERCOM SPI peripheral
             (calling the SPIClass constructor) rather than one of the
             built-in SPI devices (e.g. &SPI, &SPI1 and so forth), you will
             need to call the begin() function for your object as well as
             pinPeripheral() for the MOSI, MISO and SCK pins to configure
             GPIO manually. Do this BEFORE calling the display-specific
             begin or init function. Unfortunate but unavoidable.
*/
void Adafruit_SPITFT_3(uint16_t w, uint16_t h, SPIClass *spiClass,
  int8_t cs, int8_t dc, int8_t rst) {
  
  Adafruit_GFX(w, h);
  /*
#ifdef USA_SPI_HW
  connection=TFT_HARD_SPI;
  _rst=rst;
  _cs=cs;
  _dc=dc;
  
    hwspi._spi = spiClass;
 #endif
   */
  }
#endif // end !ESP8266

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
    @return  Adafruit_SPITFT object.
    @note    Output pins are not initialized; application typically will need
             to call subclass' begin() function, which in turn calls this
             library's initSPI() function to initialize pins.
             Yes, the name is a misnomer...this library originally handled
             only SPI displays, parallel being a recent addition (but not
             wanting to break existing code).
*/
void Adafruit_SPITFT_4(uint16_t w, uint16_t h, enum tftBusWidth busWidth,
  int8_t d0, int8_t wr, int8_t dc, int8_t cs, int8_t rst, int8_t rd) {
  
  Adafruit_GFX(w, h);
  
  /*
  connection=TFT_PARALLEL;
  _rst=rst;
  _cs=cs;
  _dc=dc;
  
    tft8._d0  = d0;
    tft8._wr  = wr;
    tft8._rd  = rd;
    tft8.wide = (busWidth == tft16bitbus);
    */
  }

// end constructors -------


// CLASS MEMBER FUNCTIONS --------------------------------------------------

// begin() and setAddrWindow() MUST be declared by any subclass.

/*!
    @brief  Configure microcontroller pins for TFT interfacing. Typically
            called by a subclass' begin() function.
    @param  freq     SPI frequency when using hardware SPI. If default (0)
                     is passed, will fall back on a device-specific value.
                     Value is ignored when using software SPI or parallel
                     connection.
    @param  spiMode  SPI mode when using hardware SPI. MUST be one of the
                     values SPI_MODE0, SPI_MODE1, SPI_MODE2 or SPI_MODE3
                     defined in SPI.h. Do NOT attempt to pass '0' for
                     SPI_MODE0 and so forth...the values are NOT the same!
                     Use ONLY the defines! (Pity it's not an enum.)
    @note   Another anachronistically-named function; this is called even
            when the display connection is parallel (not SPI). Also, this
            could probably be made private...quite a few class functions
            were generously put in the public section.
*/
void initSPI(uint32_t freq, uint8_t spiMode) {

  if(!freq) 
    freq = SPI_DEFAULT_FREQ;  // If no freq specified, use default

  // Init basic control pins common to all connection types
#ifndef USA_SPI_HW

	_initError = 0b00000000;

	SPISDOTris=0;				// SDO è output
	SPISCKTris=0;				// SCK è output
	SPICSTris=0;
	LCDDCTris=0;
#if defined(LCDRSTTris)
  LCDRSTTris=0;
#endif
  
  
#else

	SPISDOTris=0;				// SDO è output
	SPISCKTris=0;				// SCK è output
	SPICSTris=0;
	LCDDCTris=0;
  
//  OpenSPI1(ENABLE_SCK_PIN /*QUA serve se no non ho gli IRQ SPI! */ & ENABLE_SDO_PIN & SPI_MODE16_ON & 
//		SPI_SMP_ON & SPI_CKE_ON & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_LOW & MASTER_ENABLE_ON &
//		SEC_PRESCAL_4_1 & PRI_PRESCAL_1_1,		// 16MHz circa, 14.7.15; in teoria si può andare a 21-22 con prescaler=3 ...
//		FRAME_ENABLE_OFF & FRAME_SYNC_OUTPUT & FRAME_POL_ACTIVE_LOW & FRAME_SYNC_EDGE_PRECEDE & FIFO_BUFFER_ENABLE /* mettere ENABLE*/,
//		SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR & BUF_ELE_COUNT_0 & BUF_INT_SEL_6 & SPI_SHFTREG_EMPTY & RX_FIFO_EMPTY
//		);
//	SetPriorityIntSPI1(5);		// STESSA del timer! per shadow

  switch(spiMode) {
    case 0:
      SPI2CON= 0b00000000000000000000000100110000;    // master, no SDI, mode 0 (v tcpip stack: 0x0120)
      break;
    case 1:
      SPI2CON= 0b00000000000000000000000000110000;    //no fancy stuff
      break;
    case 2:
      SPI2CON= 0b00000000000000000000000101110000;    //
      break;
    case 3:
      SPI2CON= 0b00000000000000000000000001110000;    //
      break;
    }
  SPI2CONbits.STXISEL=0b00;     // tx irq
  
  SPI2CON2= 0b00000000000000000000000000000000;    // no special length; SPITUREN irq per dma?
  SPI2STAT= 0b00000000000000000000000000000000;    // 
  SPI2BRG=1 /* (GetPeripheralClock() / freq /2) -1 */;      // 1= 25MHz 8/11/19; 0= 50MHz!! 9/11/19
  // ma a 50MHz a volte è instabile... specie la "pixel()" ... (altre cose sembrano andare)
  
  SPI2CONbits.ON=1;
#endif
  


#if defined(SPI_HAS_TRANSACTION)
        hwspi.settings = SPISettings(freq, MSBFIRST, spiMode);
#else
#endif
//        hwspi._mode    = spiMode; // Save spiMode value for later
        
        // Call hwspi._spi->begin() ONLY if this is among the 'established'
        // SPI interfaces in variant.h. For DIY roll-your-own SERCOM SPIs,
        // begin() and pinPeripheral() calls MUST be made in one's calling
        // code, BEFORE the screen-specific begin/init function is called.
        // Reason for this is that SPI::begin() makes its own calls to
        // pinPeripheral() based on g_APinDescription[n].ulPinType, which
        // on non-established SPI interface pins will always be PIO_DIGITAL
        // or similar, while we need PIO_SERCOM or PIO_SERCOM_ALT...it's
        // highly unique between devices and variants for each pin or
        // SERCOM so we can't make those calls ourselves here. And the SPI
        // device needs to be set up before calling this because it's
        // immediately followed with initialization commands. Blargh.
        if(
#if !defined(SPI_INTERFACES_COUNT)
            1
#endif
#if SPI_INTERFACES_COUNT > 0
             (hwspi._spi == &SPI)
#endif
          )
        {
//            hwspi._spi->begin();
        }
  
        
  // TFT_PARALLEL

  m_SPICSBit=1;

    // Toggle _rst low to reset
#if defined(m_LCDRSTBit)
  m_LCDRSTBit=1;
  __delay_ms(100);
  m_LCDRSTBit=0;
  __delay_ms(100);
  m_LCDRSTBit=1;
  __delay_ms(200);
#else
  __delay_ms(200);
#endif

  }


/**************************************************************************/
/*!
    @brief  Companion code to the initialization tables. Reads and issues
            a series of LCD commands stored in PROGMEM byte array.
    @param  addr  Flash memory array with commands and data to send
*/
/**************************************************************************/
void displayInit(const uint8_t *addr) {
  uint8_t  numCommands, cmd, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    cmd = pgm_read_byte(addr++);         // Read command
    numArgs  = pgm_read_byte(addr++);    // Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   // If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            // Mask out delay bit
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) 
        ms = 500;     // If 255, delay for 500 ms
      __delay_ms(ms);
      }
    }
  }

/**************************************************************************/
/*!
    @brief  Initialize ST77xx chip. Connects to the ST77XX over SPI and
            sends initialization procedure commands
    @param  freq  Desired SPI clock frequency
*/
/**************************************************************************/
void Adafruit_ST77xx_begin(uint32_t freq) {
  
  if(!freq) {
    freq = SPI_DEFAULT_FREQ;
    }
  _freq = freq;

  invertOnCommand  = ST77XX_INVON;
  invertOffCommand = ST77XX_INVOFF;

  initSPI(freq, spiMode);
  }

/**************************************************************************/
/*!
    @brief  Initialization code common to all ST77XX displays
    @param  cmdList  Flash memory array with commands and data to send
*/
/**************************************************************************/
void commonInit(const uint8_t *cmdList) {
  
  Adafruit_ST77xx_begin(10000000UL);

  if(cmdList) {
    displayInit(cmdList);
    }
  }

/**************************************************************************/
/*!
  @brief  SPI displays set an address window rectangle for blitting pixels
  @param  x  Top left corner x coordinate
  @param  y  Top left corner y coordinate
  @param  w  Width of window
  @param  h  Height of window
*/
/**************************************************************************/
void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  
  x += _xstart;
  y += _ystart;
  uint32_t xa = ((uint32_t)x << 16) | (x+w-1);
  uint32_t ya = ((uint32_t)y << 16) | (y+h-1); 

  writeCommand(ST77XX_CASET); // Column addr set
  SPI_WRITE32(xa);

  writeCommand(ST77XX_RASET); // Row addr set
  SPI_WRITE32(ya);

  writeCommand(ST77XX_RAMWR); // write to RAM
  }

/**************************************************************************/
/*!
    @brief  Set origin of (0,0) and orientation of TFT display
    @param  m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_ST77xx_setRotation(uint8_t m) {   // ce n'è anche una in ST7735...
  uint8_t madctl = 0;

  rotation = m % 4; // can't be higher than 3

  switch(rotation) {
   case 0:
     madctl  = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 1:
     madctl  = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  case 2:
     madctl  = ST77XX_MADCTL_RGB;
     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 3:
     madctl  = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  }
  
  sendCommand(ST77XX_MADCTL, &madctl, 1);
  }

/**************************************************************************/
/*!
    @brief  Set origin of (0,0) of display with offsets
    @param  col  The offset from 0 for the column address
    @param  row  The offset from 0 for the row address
*/
/**************************************************************************/
void Adafruit_ST77xx_setColRowStart(int8_t col, int8_t row) {
  
  _colstart = col;
  _rowstart = row;
  }


/**************************************************************************/
/*!
 @brief  Change whether display is on or off
 @param  enable True if you want the display ON, false OFF
 */
/**************************************************************************/
void Adafruit_ST77xx_enableDisplay(BOOL enable) {
  
  sendCommand(enable ? ST77XX_DISPON : ST77XX_DISPOFF,NULL,0);
  }


////////// stuff not actively being used, but kept for posterity
/*

 uint8_t Adafruit_ST77xx::spiread(void) {
 uint8_t r = 0;
 if (_sid > 0) {
 r = shiftIn(_sid, _sclk, MSBFIRST);
 } else {
 //SID_DDR &= ~_BV(SID);
 //int8_t i;
 //for (i=7; i>=0; i--) {
 //  SCLK_PORT &= ~_BV(SCLK);
 //  r <<= 1;
 //  r |= (SID_PIN >> SID) & 0x1;
 //  SCLK_PORT |= _BV(SCLK);
 //}
 //SID_DDR |= _BV(SID);
 
 }
 return r;
 }
 
 void Adafruit_ST77xx::dummyclock(void) {
 
 if (_sid > 0) {
 digitalWrite(_sclk, LOW);
 digitalWrite(_sclk, HIGH);
 } else {
 // SCLK_PORT &= ~_BV(SCLK);
 //SCLK_PORT |= _BV(SCLK);
 }
 }
 uint8_t Adafruit_ST77xx::readdata(void) {
 *portOutputRegister(rsport) |= rspin;
 
 *portOutputRegister(csport) &= ~ cspin;
 
 uint8_t r = spiread();
 
 *portOutputRegister(csport) |= cspin;
 
 return r;
 
 } 
 
 uint8_t Adafruit_ST77xx::readcommand8(uint8_t c) {
 digitalWrite(_rs, LOW);
 
 *portOutputRegister(csport) &= ~ cspin;
 
 spiwrite(c);
 
 digitalWrite(_rs, HIGH);
 pinMode(_sid, INPUT); // input!
 digitalWrite(_sid, LOW); // low
 spiread();
 uint8_t r = spiread();
 
 
 *portOutputRegister(csport) |= cspin;
 
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 
 uint16_t Adafruit_ST77xx::readcommand16(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Adafruit_ST77xx::readcommand32(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
