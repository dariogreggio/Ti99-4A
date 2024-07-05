
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>

#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#include "adafruit_gfx.h"

#include "tms9900_PIC.h"



BYTE fExit=0;
BYTE debug=0;
#define UNIMPLEMENTED_MEMORY_VALUE 0xFF
#define RAM_START 0x8000
#define RAM_SIZE 32768
//#define RAM_START 0xc000
//#define RAM_SIZE 16384 ma DEVE partire dal Top!!
//#define RAM_START 0xf000  // per VELOCIZZAER debug!
//#define RAM_SIZE 4096 // ma DEVE partire dal Top!!
#define RAM_SIZE2 65536
#define RAM_START2 0x0000   // in slot 2, v.sotto
#define ROM_START 0x0000
#define ROM_SIZE 32768
BYTE ram_seg[RAM_SIZE];
#ifdef RAM_SIZE2 
BYTE ram_seg2[RAM_SIZE2];
#endif
BYTE rom_seg[ROM_SIZE];			
#ifdef ROM_SIZE2 
BYTE rom_seg2[ROM_SIZE2];
#endif
volatile BYTE TIMIRQ,VIDIRQ;
volatile WORD TIMEr;
BYTE TMS9918Reg[8],TMS9918RegS,TMS9918Sel,TMS9918WriteStage,TMS9918Buffer;
WORD TMS9918RAMPtr;
BYTE AY38910RegR[16],AY38910RegW[16],AY38910RegSel;
BYTE VideoRAM[VIDEORAM_SIZE];
BYTE Keyboard[8]={255,255,255,255,255,255,255,255,255,255,255};

extern volatile BYTE keysFeedPtr;

extern BYTE DoReset,DoIRQ,DoNMI,DoHalt,DoWait;
#define MAX_WATCHDOG 100      // x30mS v. sotto
extern WORD WDCnt;
extern BYTE ColdReset;
uint16_t Pipe1;
union __attribute__((__packed__)) {
	uint16_t x;
	uint16_t bb[4];
	struct __attribute__((__packed__)) {
		uint8_t l;
		uint8_t h;
//		BYTE u;		 bah no, sposto la pipe quando ci sono le istruzioni lunghe 4...
		} b;
	} Pipe2;



BYTE GetValue(SWORD t) {
	register BYTE i;

	if(t < ROM_SIZE) {			//
		i=rom_seg[t];
		}
	else if(t >= RAM_START && t < (RAM_START+RAM_SIZE)) {
		i=ram_seg[t-RAM_START];
		}

	return i;
	}

BYTE InValue(SWORD t) {    // OCCHIO pare che siano 16bit anche I/O!
	register BYTE i,j;


	return i;
	}

SWORD GetIntValue(SWORD t) {
	register SWORD i;

	if(t < ROM_SIZE) {			//
		i=MAKEWORD(rom_seg[t],rom_seg[t+1]);
		}
	else if(t >= RAM_START && t < (RAM_START+RAM_SIZE)) {
		t-=RAM_START;
		i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
		}

	return i;
	}

uint8_t GetPipe(uint16_t t) {

	if(t < ROM_SIZE) {			//
	  Pipe1=MAKEWORD(rom_seg[t],rom_seg[t+1)];
		Pipe2.b.l=rom_seg[t++];
//		Pipe2.b.h=rom_seg[t++];
//		Pipe2.b.u=rom_seg[t];
		Pipe2.b.h=rom_seg[t];
		}
	else if(t >= RAM_START && t < (RAM_START+RAM_SIZE)) {
		t-=RAM_START;
	  Pipe1=MAKEWORD(ram_seg[t],ram_seg[t+1)];
		Pipe2.b.l=ram_seg[t++];
//		Pipe2.b.h=ram_seg[t++];
//		Pipe2.b.u=ram_seg[t];
		Pipe2.b.h=ram_seg[t];
		}

	return Pipe1;
	}

void PutValue(SWORD t,BYTE t1) {
	register SWORD i;

// printf("rom_seg: %04x, p: %04x\n",rom_seg,p);

	if(t >= RAM_START && t < (RAM_START+RAM_SIZE)) {		// ZX80,81,Galaksija
	  ram_seg[t-RAM_START]=t1;
		}

	}

void PutIntValue(SWORD t,SWORD t1) {
	register SWORD i;

// printf("rom_seg: %04x, p: %04x\n",rom_seg,p);

	if(t >= RAM_START && t < (RAM_START+RAM_SIZE)) {		// ZX80,81,Galaksija
		t-=RAM_START;
	  ram_seg[t++]=LOBYTE(t1);
	  ram_seg[t]=HIBYTE(t1);
		}

}

void OutValue(SWORD t,BYTE t1) {   // 
	register SWORD i;


}


void initHW(void) {
  int i;
  
	AY38910RegR[14]=AY38910RegR[15]=0b11111111;
  AY38910RegSel=0;
  memset(VideoRAM,0,0x4000);    // mah...
  TMS9918Reg[0]=TMS_R0_EXT_VDP_DISABLE | TMS_R0_MODE_GRAPHICS_I;
  TMS9918Reg[1]=TMS_R1_RAM_16K | TMS_R1_MODE_GRAPHICS_I /* bah   | TMS_R1_DISP_ACTIVE | TMS_R1_INT_ENABLE*/;
  TMS9918Reg[2]=TMS_DEFAULT_VRAM_NAME_ADDRESS >> 10;
  TMS9918Reg[3]=TMS_DEFAULT_VRAM_COLOR_ADDRESS >> 6;
  TMS9918Reg[4]=TMS_DEFAULT_VRAM_PATT_ADDRESS >> 11;
  TMS9918Reg[5]=TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS >> 7;
  TMS9918Reg[6]=TMS_DEFAULT_VRAM_SPRITE_PATT_ADDRESS >> 11;
  TMS9918Reg[7]=(1 /*black*/ << 4) | 7 /*cyan*/;
  TMS9918RegS=0;
  TMS9918Sel=TMS9918WriteStage=0;
extern const unsigned char charset_international[2048],tmsFont[(128-32)*8];
  memcpy(VideoRAM+TMS_DEFAULT_VRAM_PATT_ADDRESS,charset_international,2048);// mah...
//  memcpy(VideoRAM+TMS_DEFAULT_VRAM_PATT_ADDRESS,tmsFont,(128-32)*8);
  struct SPRITE_ATTR *sa;
  sa=(struct SPRITE_ATTR *)&VideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS];
  for(i=0; i<32; i++) {
//    VideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS+i*4]=LAST_SPRITE_YPOS;
    sa->ypos=LAST_SPRITE_YPOS;
    sa->xpos=sa->tag=sa->name=0;
//    VideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS+i*4+1]=0;
//    VideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS+i*4+2]=0;
//    VideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS+i*4+3]=0;
    sa++;
    }

  for(i=0; i<768; i++)
    VideoRAM[TMS_DEFAULT_VRAM_NAME_ADDRESS+i]=i & 0xff;
  
  keysFeedPtr=255; //
  
  OC1CONbits.ON = 0;   // spengo buzzer/audio
  PR2 = 65535;		 // 
  OC1RS = 65535;		 // 

  }

