// Font structures for newer Adafruit_GFX (1.1 and later).
// Example fonts are included in 'Fonts' directory.
// To use a font in your Arduino sketch, #include the corresponding .h
// file and pass address of GFXfont struct to setFont().  Pass NULL to
// revert to 'classic' fixed-space bitmap font.

#ifndef _GFXFONT_H_
#define _GFXFONT_H_


typedef struct { // Data stored PER GLYPH
	UINT16 bitmapOffset;     // Pointer into GFXfont->bitmap
	UINT8  width, height;    // Bitmap dimensions in pixels
	UINT8  xAdvance;         // Distance to advance cursor (x axis)
	INT8   xOffset, yOffset; // Dist from cursor pos to UL corner
	} GFXglyph;

typedef struct { // Data stored for FONT AS A WHOLE:
	UINT8  *bitmap;      // Glyph bitmaps, concatenated
	GFXglyph *glyph;       // Glyph array
	UINT8   first, last; // ASCII extents
	UINT8   yAdvance;    // Newline distance (y axis)
	} GFXfont;

#endif // _GFXFONT_H_

