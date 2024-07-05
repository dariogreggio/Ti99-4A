#include <xc.h>
#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#include "swspi.h"


uint8_t tabcolor,rotation;
UINT8	sleep;
UINT8	_initError;


// CONSTRUCTORS ************************************************************

/*!
    @brief  Instantiate Adafruit ST7735 driver with software SPI
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  mosi  SPI MOSI pin #
    @param  sclk  SPI Clock pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
*/
int Adafruit_ST7735_1(int8_t cs, int8_t dc, int8_t mosi,
  int8_t sclk, int8_t rst) {
  
  return Adafruit_ST77xx_1(ST7735_TFTWIDTH_128,
    ST7735_TFTHEIGHT_160, cs, dc, mosi, sclk, rst, -1);
  }

/*!
    @brief  Instantiate Adafruit ST7735 driver with default hardware SPI
    @param  cs   Chip select pin #
    @param  dc   Data/Command pin #
    @param  rst  Reset pin # (optional, pass -1 if unused)
*/
int Adafruit_ST7735_2(int8_t cs, int8_t dc, int8_t rst) {
  
  return Adafruit_ST77xx_2(ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160, cs, dc, rst);
  }

#if !defined(ESP8266)
/*!
    @brief  Instantiate Adafruit ST7735 driver with selectable hardware SPI
    @param  spiClass  Pointer to an SPI device to use (e.g. &SPI1)
    @param  cs        Chip select pin #
    @param  dc        Data/Command pin #
    @param  rst       Reset pin # (optional, pass -1 if unused)
*/
/* int Adafruit_ST7735_SPI(SPIClass *spiClass, int8_t cs, int8_t dc,
  int8_t rst) {
  
  Adafruit_ST77xx(ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160,
    spiClass, cs, dc, rst);
  }
 * */
#endif // end !ESP8266

// SCREEN INITIALIZATION ***************************************************

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.

static const uint8_t PROGMEM
  Bcmd[] = {                        // Init commands for 7735B screens
    18,                             // 18 commands in list:
    ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, no args, w/delay
      50,                           //     50 ms delay
    ST77XX_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, no args, w/delay
      255,                          //     255 = max (500 ms) delay
    ST77XX_COLMOD,  1+ST_CMD_DELAY, //  3: Set color mode, 1 arg + delay:
      0x05,                         //     16-bit color
      10,                           //     10 ms delay
    ST7735_FRMCTR1, 3+ST_CMD_DELAY, //  4: Frame rate control, 3 args + delay:
      0x00,                         //     fastest refresh
      0x06,                         //     6 lines front porch
      0x03,                         //     3 lines back porch
      10,                           //     10 ms delay
    ST77XX_MADCTL,  1,              //  5: Mem access ctl (directions), 1 arg:
      0x08,                         //     Row/col addr, bottom-top refresh
    ST7735_DISSET5, 2,              //  6: Display settings #5, 2 args:
      0x15,                         //     1 clk cycle nonoverlap, 2 cycle gate
                                    //     rise, 3 cycle osc equalize
      0x02,                         //     Fix on VTL
    ST7735_INVCTR,  1,              //  7: Display inversion control, 1 arg:
      0x0,                          //     Line inversion
    ST7735_PWCTR1,  2+ST_CMD_DELAY, //  8: Power control, 2 args + delay:
      0x02,                         //     GVDD = 4.7V
      0x70,                         //     1.0uA
      10,                           //     10 ms delay
    ST7735_PWCTR2,  1,              //  9: Power control, 1 arg, no delay:
      0x05,                         //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3,  2,              // 10: Power control, 2 args, no delay:
      0x01,                         //     Opamp current small
      0x02,                         //     Boost frequency
    ST7735_VMCTR1,  2+ST_CMD_DELAY, // 11: Power control, 2 args + delay:
      0x3C,                         //     VCOMH = 4V
      0x38,                         //     VCOML = -1.1V
      10,                           //     10 ms delay
    ST7735_PWCTR6,  2,              // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16,              // 13: Gamma Adjustments (pos. polarity), 16 args + delay:
      0x09, 0x16, 0x09, 0x20,       //     (Not entirely necessary, but provides
      0x21, 0x1B, 0x13, 0x19,       //      accurate colors)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+ST_CMD_DELAY, // 14: Gamma Adjustments (neg. polarity), 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E,       //     (Not entirely necessary, but provides
      0x22, 0x1D, 0x18, 0x1E,       //      accurate colors)
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                           //     10 ms delay
    ST77XX_CASET,   4,              // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,                   //     XSTART = 2
      0x00, 0x81,                   //     XEND = 129
    ST77XX_RASET,   4,              // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,                   //     XSTART = 1
      0x00, 0x81,                   //     XEND = 160
    ST77XX_NORON,     ST_CMD_DELAY, // 17: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST77XX_DISPON,    ST_CMD_DELAY, // 18: Main screen turn on, no args, delay
      255 },                        //     255 = max (500 ms) delay

  Rcmd1[] = {                       // 7735R init, part 1 (red or green tab)
    15,                             // 15 commands in list:
    ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
      150,                          //     150 ms delay
    ST77XX_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, 0 args, w/delay
      255,                          //     500 ms delay
    ST7735_FRMCTR1, 3,              //  3: Framerate ctrl - normal mode, 3 arg:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3,              //  4: Framerate ctrl - idle mode, 3 args:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6,              //  5: Framerate - partial mode, 6 args:
      0x01, 0x2C, 0x2D,             //     Dot inversion mode
      0x01, 0x2C, 0x2D,             //     Line inversion mode
    ST7735_INVCTR,  1,              //  6: Display inversion ctrl, 1 arg:
      0x07,                         //     No inversion
    ST7735_PWCTR1,  3,              //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                         //     -4.6V
      0x84,                         //     AUTO mode
    ST7735_PWCTR2,  1,              //  8: Power control, 1 arg, no delay:
      0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
    ST7735_PWCTR3,  2,              //  9: Power control, 2 args, no delay:
      0x0A,                         //     Opamp current small
      0x00,                         //     Boost frequency
    ST7735_PWCTR4,  2,              // 10: Power control, 2 args, no delay:
      0x8A,                         //     BCLK/2,
      0x2A,                         //     opamp current small & medium low
    ST7735_PWCTR5,  2,              // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1,  1,              // 12: Power control, 1 arg, no delay:
      0x0E,
    ST77XX_INVOFF,  0,              // 13: Don't invert display, no args
    ST77XX_MADCTL,  1,              // 14: Mem access ctl (directions), 1 arg:
      0xC8,                         //     row/col addr, bottom-top refresh
    ST77XX_COLMOD,  1,              // 15: set color mode, 1 arg, no delay:
      0x05 },                       //     16-bit color

  Rcmd2green[] = {                  // 7735R init, part 2 (green tab only)
    2,                              //  2 commands in list:
    ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,                   //     XSTART = 0
      0x00, 0x7F+0x02,              //     XEND = 127
    ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,                   //     XSTART = 0
      0x00, 0x9F+0x01 },            //     XEND = 159

  Rcmd2red[] = {                    // 7735R init, part 2 (red tab only)
    2,                              //  2 commands in list:
    ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F,                   //     XEND = 127
    ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x9F },                 //     XEND = 159

  Rcmd2green144[] = {               // 7735R init, part 2 (green 1.44 tab)
    2,                              //  2 commands in list:
    ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F,                   //     XEND = 127
    ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F },                 //     XEND = 127

  Rcmd2green160x80[] = {            // 7735R init, part 2 (mini 160x80)
    2,                              //  2 commands in list:
    ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x4F,                   //     XEND = 79
    ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x9F },                 //     XEND = 159

  Rcmd3[] = {                       // 7735R init, part 3 (red or green tab)
    4,                              //  4 commands in list:
    ST7735_GMCTRP1, 16      ,       //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
      0x02, 0x1c, 0x07, 0x12,       //     (Not entirely necessary, but provides
      0x37, 0x32, 0x29, 0x2d,       //      accurate colors)
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      ,       //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
      0x03, 0x1d, 0x07, 0x06,       //     (Not entirely necessary, but provides
      0x2E, 0x2C, 0x29, 0x2D,       //      accurate colors)
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST77XX_NORON,     ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST77XX_DISPON,    ST_CMD_DELAY, //  4: Main screen turn on, no args w/delay
      100 };                        //     100 ms delay

/**************************************************************************/
/*!
    @brief  Initialization code common to all ST7735B displays
*/
/**************************************************************************/
void Adafruit_ST7735_initB(void) {
  
  commonInit(Bcmd);
  setRotation(0);
  }

/**************************************************************************/
/*!
    @brief  Initialization code common to all ST7735R displays
    @param  options  Tab color from adafruit purchase
*/
/**************************************************************************/
void Adafruit_ST7735_initR(uint8_t options) {
  
  commonInit(Rcmd1);
  if(options == INITR_GREENTAB) {
    displayInit(Rcmd2green);
    _colstart = 2;
    _rowstart = 1;
  } else if((options == INITR_144GREENTAB) || (options == INITR_HALLOWING)) {
    _height   = ST7735_TFTHEIGHT_128;
    _width    = ST7735_TFTWIDTH_128;
    displayInit(Rcmd2green144);
    _colstart = 2;
    _rowstart = 3; // For default rotation 0
  } else if(options == INITR_MINI160x80) {
    _height   = ST7735_TFTHEIGHT_160;
    _width    = ST7735_TFTWIDTH_80;
    displayInit(Rcmd2green160x80);
    _colstart = 24;
    _rowstart = 0;
    }
  else {
    // colstart, rowstart left at default '0' values
    displayInit(Rcmd2red);
    }
  displayInit(Rcmd3);

  // Black tab, change MADCTL color filter
  if((options == INITR_BLACKTAB) || (options == INITR_MINI160x80)) {
    uint8_t data = 0xC0;
    sendCommand(ST77XX_MADCTL, &data, 1);
    }

  if(options == INITR_HALLOWING) {
    // Hallowing is simply a 1.44" green tab upside-down:
    tabcolor = INITR_144GREENTAB;
    setRotation(2);
    }
  else {
    tabcolor = options;
    setRotation(1 /*0*/);
    }
  }

// OTHER FUNCTIONS *********************************************************

/**************************************************************************/
/*!
    @brief  Set origin of (0,0) and orientation of TFT display
    @param  m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void setRotation(uint8_t m) {    // ce n'è anche una in ST77xx...
  uint8_t madctl = 0;

  rotation = m & 3; // can't be higher than 3

  // For ST7735 with GREEN TAB (including HalloWing)...
  if((tabcolor == INITR_144GREENTAB) || (tabcolor == INITR_HALLOWING)) {
    // ..._rowstart is 3 for rotations 0&1, 1 for rotations 2&3
    _rowstart = (rotation < 2) ? 3 : 1;
    }

  switch(rotation) {
    case 0:
      if((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
        madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
        }
      else {
        madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST7735_MADCTL_BGR;
        }

     if (tabcolor == INITR_144GREENTAB) {
       _height = ST7735_TFTHEIGHT_128;
       _width  = ST7735_TFTWIDTH_128;
     } else if (tabcolor == INITR_MINI160x80)  {
       _height = ST7735_TFTHEIGHT_160;
       _width  = ST7735_TFTWIDTH_80;
     } else {
       _height = ST7735_TFTHEIGHT_160;
       _width  = ST7735_TFTWIDTH_128;
     }
     _xstart   = _colstart;
     _ystart   = _rowstart;
     break;
     
   case 1:
     if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
       madctl = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
     } else {
       madctl = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST7735_MADCTL_BGR;
     }

     if(tabcolor == INITR_144GREENTAB)  {
       _width  = ST7735_TFTHEIGHT_128;
       _height = ST7735_TFTWIDTH_128;
     } else if (tabcolor == INITR_MINI160x80)  {
       _width  = ST7735_TFTHEIGHT_160;
       _height = ST7735_TFTWIDTH_80;
     } else {
       _width  = ST7735_TFTHEIGHT_160;
       _height = ST7735_TFTWIDTH_128;
     }
     _ystart   = _colstart;
     _xstart   = _rowstart;
     break;
     
  case 2:
     if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
       madctl = ST77XX_MADCTL_RGB;
     } else {
       madctl = ST7735_MADCTL_BGR;
     }

     if (tabcolor == INITR_144GREENTAB) {
       _height = ST7735_TFTHEIGHT_128;
       _width  = ST7735_TFTWIDTH_128;
     } else if (tabcolor == INITR_MINI160x80)  {
       _height = ST7735_TFTHEIGHT_160;
       _width  = ST7735_TFTWIDTH_80;
     } else {
       _height = ST7735_TFTHEIGHT_160;
       _width  = ST7735_TFTWIDTH_128;
     }
     _xstart   = _colstart;
     _ystart   = _rowstart;
     break;
     
   case 3:
     if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
       madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
     } else {
       madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST7735_MADCTL_BGR;
     }

     if (tabcolor == INITR_144GREENTAB)  {
       _width  = ST7735_TFTHEIGHT_128;
       _height = ST7735_TFTWIDTH_128;
     } else if (tabcolor == INITR_MINI160x80)  {
       _width  = ST7735_TFTHEIGHT_160;
       _height = ST7735_TFTWIDTH_80;
     } else {
       _width  = ST7735_TFTHEIGHT_160;
       _height = ST7735_TFTWIDTH_128;
     }
     _ystart   = _colstart;
     _xstart   = _rowstart;
     break;
    }

  sendCommand(ST77XX_MADCTL, &madctl, 1);
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
  int16_t w, int16_t h, uint16_t color) {
  
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
				INT16 len = x0-xbegin;
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
				INT16 len = x0-xbegin;
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


void drawRect(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color){

	HLine(x, y, w, color);
	HLine(x, y+h-1, w, color);
	VLine(x, y, h, color);
	VLine(x+w-1, y, h, color);
//	writecommand(CMD_NOP);
	}

void fillScreen(UINT16 color) {
	UINT16 px,py;

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
void writePixel(int16_t x, int16_t y, uint16_t color) {
  
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
void writeColor(uint16_t color, uint32_t len) {

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


void writedata16(UINT16 d) {

  m_LCDDCBit=1;
	m_SPICSBit=0;
  SPI_WRITE16(d);
	m_SPICSBit=1;
	} 

void writedata16x2(UINT16 d1,UINT16 d2) {

  m_LCDDCBit=1;
	m_SPICSBit=0;
  SPI2CONbits.ENHBUF=1;
  SPI2CONbits.MODE16=1;     // USARE!!
  SPI2STATbits.SPIROV = 0;  // Reset overflow bit
  SPI2BUF = d1;     // Write to SPI buffer
  SPI2BUF = d2;     // Write to SPI buffer
  while(SPI2STATbits.SPIBUSY == 1 /*(SPI2STATLbits.SPITBF == 1)*/ /* || (SPI2STATLbits.SPIRBF == 0) */ )
    ClrWdt();
  SPI2BUF;
  SPI2CONbits.ENHBUF=0;
  SPI2CONbits.MODE16=0;
	m_SPICSBit=1;
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

void sendCommand(uint8_t commandByte, uint8_t *dataBytes, uint8_t numDataBytes) {
  int i;
  
  SPI_BEGIN_TRANSACTION();
  SPI_CS_LOW();
  
  SPI_DC_LOW(); // Command mode
  spiWrite(commandByte); // Send the command byte
  
  SPI_DC_HIGH();
  for(i=0; i<numDataBytes; i++) {
    spiWrite(*dataBytes++); // Send the data bytes
    }
  
  SPI_CS_HIGH();
  SPI_END_TRANSACTION();
  }

uint8_t readcommand8(uint8_t commandByte, uint8_t index) {
  uint8_t result;
  
  START_WRITE();
  SPI_DC_LOW();     // Command mode
  spiWrite(commandByte);
  SPI_DC_HIGH();    // Data mode
  do {
    result = spiRead();
    } while(index--); // Discard bytes up to index'th
  END_WRITE();
  return result;
  }

inline void SPI_BEGIN_TRANSACTION(void) {
  
#if defined(SPI_HAS_TRANSACTION)
        hwspi._spi->beginTransaction(hwspi.settings);
#else // No transactions, configure SPI manually...

#endif // end !SPI_HAS_TRANSACTION
  }

inline void SPI_END_TRANSACTION(void) {
  
#if defined(SPI_HAS_TRANSACTION)
    if(connection == TFT_HARD_SPI) {
        hwspi._spi->endTransaction();
    }
#endif
  }

void __attribute__((always_inline)) startWrite(void) {
  
  SPI_BEGIN_TRANSACTION();
  SPI_CS_LOW();
  }

/*!
    @brief  Call after issuing command(s) or data to display. Performs
            chip-deselect (if required) and ends an SPI transaction (if
            using hardware SPI and transactions are supported). Required
            for all display types; not an SPI-specific function.
*/
void __attribute__((always_inline)) endWrite(void) {
  
  SPI_CS_HIGH();
  SPI_END_TRANSACTION();
  }

/*!
    @brief  Issue a single 8-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the byte. This is another of
            those functions in the library with a now-not-accurate name
            that's being maintained for compatibility with outside code.
            This function is used even if display connection is parallel.
    @param  b  8-bit value to write.
*/
void spiWrite(uint8_t c) {
  
#ifndef USA_SPI_HW
	DoSPIMO(c);
#else
  SPI2STATbits.SPIROV = 0;  // Reset overflow bit
  SPI2BUF = c;     // Write character to SPI buffer
  while(SPI2STATbits.SPIBUSY == 1 /*(SPI2STATLbits.SPITBF == 1)*/ /* || (SPI2STATLbits.SPIRBF == 0) */ )
    ClrWdt();
  SPI2BUF;
//  while(SPI2STATbits.SPITBF)   // Wait until transmission is started
//    ;
//		}
#endif
  }

void SPI_WRITE16(uint16_t w) {
  
#ifndef USA_SPI_HW
	DoSPIMO(HIBYTE(w));
	DoSPIMO(LOBYTE(w));
#else
  SPI2CONbits.MODE16=1;     // 
  SPI2STATbits.SPIROV = 0;  // Reset overflow bit
  SPI2BUF = w;     // Write character to SPI buffer
  while(SPI2STATbits.SPIBUSY == 1 /*(SPI2STATLbits.SPITBF == 1)*/ /* || (SPI2STATLbits.SPIRBF == 0) */ )
    ClrWdt();
  SPI2BUF;
  SPI2CONbits.MODE16=0;
#endif
}

/*!
    @brief  Issue a single 32-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the longword. Despite the
            name, this function is used even if display connection is
            parallel; name was maintained for backward compatibility. Naming
            is also not consistent with the 8-bit version, spiWrite().
            Sorry about that. Again, staying compatible with outside code.
    @param  l  32-bit value to write.
*/
void SPI_WRITE32(uint32_t l) {
  
#ifndef USA_SPI_HW
	DoSPIMO(HIBYTE(HIWORD(l)));
	DoSPIMO(LOBYTE(HIWORD(l)));
	DoSPIMO(HIBYTE(LOWORD(l)));
	DoSPIMO(LOBYTE(LOWORD(l)));
#else
  SPI2CONbits.MODE32=1;     // USARE!!
  SPI2STATbits.SPIROV = 0;  // Reset overflow bit
  SPI2BUF = l;     // Write character to SPI buffer
  while(SPI2STATbits.SPIBUSY == 1 /*(SPI2STATLbits.SPITBF == 1)*/ /* || (SPI2STATLbits.SPIRBF == 0) */ )
    ClrWdt();
  SPI2BUF;
  SPI2CONbits.MODE32=0; 
#endif

  }


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
  spiWrite(cmd);
  SPI_DC_HIGH();
  }

/*!
    @brief   Read a single 8-bit value from the display. Chip-select and
             transaction must have been previously set -- this ONLY reads
             the byte. This is another of those functions in the library
             with a now-not-accurate name that's being maintained for
             compatibility with outside code. This function is used even if
             display connection is parallel.
    @return  Unsigned 8-bit value read (always zero if USE_FAST_PINIO is
             not supported by the MCU architecture).
*/
uint8_t spiRead(void) {
  uint8_t  b = 0;
  uint16_t w = 0;
    
#ifndef USA_SPI_HW
	b=DoSPIMI();
#else
  SPI2BUF = 0;     // Write character to SPI buffer
  while(!SPI2STATbits.SPIRBF)   // Wait until transmission is started
    ;
  b=SPI2BUF;
	SPI2STATbits.SPIROV = 0;  // Reset overflow bit
#endif
  
  return b;
  }

void begin(void) {

	sleep = 0;
	_initError = 0b00000000;

	SPISDOTris=0;				// SDO è output
	SPISCKTris=0;				// SCK è output
	SPICSTris=0;
	LCDDCTris=0;

  }


extern const char CopyrightString[];
extern const unsigned char logo_msx[];

void drawBG(void) {
	char buffer[22];

	gfx_drawRect(0,0,_TFTWIDTH,_TFTHEIGHT,GREEN);
	gfx_drawRect(1,1,_TFTWIDTH-2,_TFTHEIGHT-2,LIGHTGREEN);

	setTextSize(1);
  
#ifdef MSX  
  drawBitmap4(30,34,logo_msx);
#endif
  
	setTextColor(BRIGHTGREEN);
	LCDXY(0,1);
	gfx_print(CopyrightString);

	setTextColor(BRIGHTCYAN);
	LCDXY(8,14);
	gfx_print("booting...");
  
  DelayMs(800); 
	}

//retrocompatibilità con lcd.c ecc
BYTE LCDInit(void) {		// su SPI

//	TFT_ILI9163C();
  Adafruit_ST7735_1(0,0,0,0,-1);
  ClrWdt();
  Adafruit_ST7735_initR(INITR_BLACKTAB);
  
  ClrWdt();
  
  
#if defined(m_LCDBLBit)
  m_LCDBLBit=1;
#endif
  
//	begin();
	LCDCls();

// init done
	setTextWrap(1);
//	setTextColor2(WHITE, BLACK);

	drawBG();
  
  __delay_ms(250);
  ClrWdt();
  __delay_ms(250);
  ClrWdt();

	return 1;
	}

void LCDXY(BYTE x, BYTE y) {	setCursor(x*textsize*6,y*textsize*8);	}

void LCDCls() {	clearScreen();	LCDXY(0,0); /*display();*/ }	// serve?? 


