#ifndef _ADAFRUIT_GFX_H
#define _ADAFRUIT_GFX_H

//#define USE_CANVAS 1
//#define USE_GFX_UI 1
//#define RAM_BITMAP 1
//#define USE_CUSTOM_FONTS 1
//#define USE_256_CHAR_FONT 1


#include <stdint.h>
#include <stddef.h>
//typedef unsigned int size_t;        // dovrebbe essere nell'include ma MORTE A nickderosa!!

#include "gfxfont.h"
//#include "tft_ili9163c.h"
#include "Adafruit_st77xx.h"

#define swap(a, b) { INT16 t = a; a = b; b = t; }

void Adafruit_GFX(UGRAPH_COORD_T w, UGRAPH_COORD_T h); // Constructor

	typedef void *ADAFRUIT_GFX;
 	extern ADAFRUIT_GFX Adafruit_GFX_this;

	typedef char *__FlashStringHelper;

  // This MUST be defined by the subclass:
    void drawPixel(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT16 color);

  // These MAY be overridden by the subclass to provide device-specific
  // optimized code.  Otherwise 'generic' versions are used.
  void gfx_drawLine(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T x1, UGRAPH_COORD_T y1, UINT16 color);
  void gfx_drawFastVLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T h, UINT16 color);
  void gfx_drawFastHLine(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UINT16 color);
  void gfx_drawRect(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color);
  void gfx_fillRect(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color);
  void gfx_fillScreen(UINT16 color);

// le faccio #define per risparmiare memoria!
#define gfx_drawLine(x0,y0,x1,y1,color) drawLine(x0,y0,x1,y1,color)
#define gfx_drawRect(x,y,w,h,color) drawRect(x,y,w,h,color)
#define gfx_drawFastVLine(x,y,h,color) drawFastVLine(x, y, y+h-1, color)
#define gfx_drawFastHLine(x,y,w,color) drawFastHLine(x, y, x+w-1, color)
#define gfx_fillRect(x,y,w,h,color) fillRect(x,y,w,h,color)
#define gfx_fillScreen(color) fillScreen(color)
#define gfx_invertDisplay(i) invertDisplay(i)


  // These exist only with Adafruit_GFX (no subclass overrides)
  void drawCircle(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T r, UINT16 color);
  void drawCircleHelper(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T r, UINT8 cornername, UINT16 color);
  void fillCircle(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T r, UINT16 color);
  void fillCircleHelper(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T r, UINT8 cornername, GRAPH_COORD_T delta, UINT16 color);
  void drawTriangle(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T x1, UGRAPH_COORD_T y1, UGRAPH_COORD_T x2, UGRAPH_COORD_T y2, UINT16 color);
  void fillTriangle(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T x1, UGRAPH_COORD_T y1, UGRAPH_COORD_T x2, UGRAPH_COORD_T y2, UINT16 color);
  void drawRoundRect(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UGRAPH_COORD_T radius, UINT16 color);
  void fillRoundRect(UGRAPH_COORD_T x0, UGRAPH_COORD_T y0, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UGRAPH_COORD_T radius, UINT16 color);
  void drawBitmap(UGRAPH_COORD_T x, UGRAPH_COORD_T y, const UINT8 *bitmap, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color);
  void drawBitmapBG(UGRAPH_COORD_T x, UGRAPH_COORD_T y, const UINT8 *bitmap, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color, UINT16 bg);
  void drawBitmap4(UGRAPH_COORD_T x, UGRAPH_COORD_T y, const UINT8 *bitmap);
#ifdef RAM_BITMAP
  void drawBitmap3(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT8 *bitmap, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color);
  void drawBitmap4(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT8 *bitmap, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color, UINT16 bg);
  void drawXBitmap(UGRAPH_COORD_T x, UGRAPH_COORD_T y, const UINT8 *bitmap, UGRAPH_COORD_T w, UGRAPH_COORD_T h, UINT16 color);
#endif
  void drawChar(UGRAPH_COORD_T x, UGRAPH_COORD_T y, unsigned char c, UINT16 color, UINT16 bg, UINT8 size);
  void setTextColor(UINT16 c);
  void setTextColorBG(UINT16 c, UINT16 bg);
  void setTextSize(UINT8 s);
  void setTextWrap(BOOL w);
  void setRotation(UINT8 r);
  void cp437(BOOL x);
  void setFont(const GFXfont *f);
  void getTextBounds(char *string, UGRAPH_COORD_T x, UGRAPH_COORD_T y, UGRAPH_COORD_T *x1, UGRAPH_COORD_T *y1, UGRAPH_COORD_T *w, UGRAPH_COORD_T *h);

  size_t cwrite(UINT8);

  UGRAPH_COORD_T height(void);
  UGRAPH_COORD_T width(void);

  UINT8 getRotation(void);
	#define gfx_setRotation(x) setRotation(x)

	#define gfx_setCursor(x,y) setCursor(x,y)


	void gfx_print(char *);

  // get current cursor position (get rotation safe maximum values, using: width() for x, height() for y)

  extern const UGRAPH_COORD_T  WIDTH, HEIGHT;   // This is the 'raw' display w/h - never changes
  extern UGRAPH_COORD_T  _width, _height, // Display w/h as modified by current rotation
    cursor_x, cursor_y;
  extern UINT16  textcolor, textbgcolor;
  extern UINT8  textsize;
  extern BOOL   wrap,   // If set, 'wrap' text at right edge of display
    _cp437; // If set, use correct CP437 charset (default is off)
  extern GFXfont  *gfxFont;


#ifdef USE_GFX_UI
  void Adafruit_GFX_Button(void);
  void initButton(ADAFRUIT_GFX gfx, UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT8 w, UINT8 h, UINT16 outline, UINT16 fill,
    UINT16 textcolor, char *label, UINT8 textsize);
  void drawButton(BOOL inverted);
  BOOL contains(UGRAPH_COORD_T x, UGRAPH_COORD_T y);


  void press(BOOL p);
  BOOL Adafruit_GFX_Button_isPressed();
  BOOL Adafruit_GFX_Button_justPressed();
  BOOL Adafruit_GFX_Button_justReleased();

  extern ADAFRUIT_GFX _gfx;
  extern UGRAPH_COORD_T _x, _y;
  extern UGRAPH_COORD_T _w, _h;
  extern UINT8 _textsize;
  extern UINT16 _outlinecolor, _fillcolor, _textcolor;
  extern char _label[10];

  extern BOOL currstate, laststate;
#endif


#ifdef USE_CANVAS
// questo pure era dinamico ma lo rendo ovviamente statico
  void GFXcanvas1(UGRAPH_COORD_T w, UGRAPH_COORD_T h);
  void drawPixel1(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT16 color);
  void fillScreen1(UINT16 color);


  void GFXcanvas16(UGRAPH_COORD_T w, UGRAPH_COORD_T h);
  void delete_GFXcanvas16(void);
  void drawPixel16(UGRAPH_COORD_T x, UGRAPH_COORD_T y, UINT16 color);
  void fillScreen16(UINT16 color);
  UINT16 *getBuffer16(void);

  UINT16 *gfx_buffer16;

#endif

typedef WORD GFX_COLOR;   
#define Color565(red, green, blue)    (GFX_COLOR) ((((GFX_COLOR)(red) & 0xF8) << 8) | (((GFX_COLOR)(green) & 0xFC) << 3) | (((GFX_COLOR)(blue) & 0xF8) >> 3))
#define ColorRGB(color)    (DWORD) ( ((color & 0xf800) >> 11) | ((color & 0x07e0) << 3) | ((((DWORD)color) & 0x001f) << 16) )
// in adaf/ili era una funzione ma io faccio macro!
#define Color24To565(color_)    ((((color_ >> 16) & 0xFF) / 8) << 11) | ((((color_ >> 8) & 0xFF) / 4) << 5) | (((color_) &  0xFF) / 8)
  //convert 24bit color into packet 16 bit one (credits for this are all mine) GD made a macro!

#ifdef ST7735
#ifndef BLACK
			#define BLACK           Color565(0, 0, 0)
#endif	
			#define BRIGHTBLUE      Color565(0, 0, 255)
			#define BRIGHTGREEN     Color565(0, 255, 0)
			#define BRIGHTCYAN      Color565(0, 255, 255)
			#define BRIGHTRED       Color565(255, 0, 0)
			#define BRIGHTMAGENTA   Color565(255, 0, 255)
			#define YELLOW          Color565(128, 128, 0)
			#define BRIGHTYELLOW    Color565(255, 255, 0)
			#define LIGHTYELLOW     Color565(255, 255, 150)
			#define GOLD            Color565(255, 215, 0)
			#define BLUE            Color565(0, 0, 128)
			#define GREEN           Color565(0, 128, 0)
			#define CYAN            Color565(0, 128, 128)
			#define RED             Color565(128, 0, 0)
			#define MAGENTA         Color565(128, 0, 128)
			#define BROWN           Color565(255, 128, 0)
			#define LIGHTGRAY       Color565(128, 128, 128)
			#define DARKGRAY        Color565(64, 64, 64)
			#define LIGHTBLUE       Color565(128, 128, 255)
			#define LIGHTGREEN      Color565(128, 255, 128)
			#define LIGHTCYAN       Color565(128, 255, 255)
			#define LIGHTRED        Color565(255, 128, 128)
			#define LIGHTMAGENTA    Color565(255, 128, 255)
#ifndef WHITE
			#define WHITE           Color565(255, 255, 255)
#endif
			#define SADDLEBROWN 	Color565(139, 69, 19)
			#define SIENNA      	Color565(160, 82, 45)
			#define PERU        	Color565(205, 133, 63)
			#define BURLYWOOD  	 	Color565(222, 184, 135)
			#define WHEAT       	Color565(245, 245, 220)
// v. TAN in basic.. !!			#define TAN         	Color565(210, 180, 140)
			#define ORANGE         	Color565(255, 187, 76)
			#define DARKORANGE      Color565(255, 140, 0)
			#define LIGHTORANGE     Color565(255, 200, 0)
			#define GRAY242      	Color565(242, 242, 242)    
			#define GRAY229      	Color565(229, 229, 229)    
			#define GRAY224         Color565(224, 224, 224)
			#define GRAY204      	Color565(204, 204, 204)    
			#define GRAY192         Color565(192, 192, 192)
			#define GRAY160         Color565(160, 160, 160)
			#define GRAY128         Color565(128, 128, 128)
			#define GRAY096          Color565(96, 96, 96)
			#define GRAY032          Color565(32, 32, 32)
			#define GRAY010          Color565(10, 10, 10)
		
			#define GRAY95      	Color565(242, 242, 242)
			#define GRAY90      	Color565(229, 229, 229)
			#define GRAY0           Color565(224, 224, 224)
			#define GRAY80      	Color565(204, 204, 204)
			#define GRAY1           Color565(192, 192, 192)
			#define GRAY2           Color565(160, 160, 160)
			#define GRAY3           Color565(128, 128, 128)
			#define GRAY4           Color565(96, 96, 96)
			#define GRAY5           Color565(64, 64, 64)
			#define GRAY6           Color565(32, 32, 32)
#else

#define BLACK 0
#define WHITE 1
#define INVERSE 2

#endif

#endif // _ADAFRUIT_GFX_H
