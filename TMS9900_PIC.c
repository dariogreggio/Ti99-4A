// c'è un curioso difettino, dopo alcuni minuti lo schermo si cimisce e escono tipo solo la prima riga ogni 8...
//  ma NON se il programma TI-BASIC gira! boh


// Local Header Files
#include <stdlib.h>
#include <string.h>
#include <xc.h>
#include "TMS9900_PIC.h"

#include <sys/attribs.h>
#include <sys/kmem.h>

#ifdef ST7735
#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#include "adafruit_gfx.h"
#endif
#ifdef ILI9341
#include "Adafruit_ILI9341.h"
#include "adafruit_gfx.h"
#endif



// PIC32MZ1024EFE064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
/* Default SYSCLK = 200 MHz (8MHz FRC / FPLLIDIV * FPLLMUL / FPLLODIV) */
//#pragma config FPLLIDIV = DIV_1, FPLLMULT = MUL_50, FPLLODIV = DIV_2
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (1x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ// System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_51       // System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLODIV = DIV_2        // System PLL Output Clock Divider (2x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = FRCDIV           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS16384          // Watchdog Timer Postscaler (1:16384)
  // circa 6-7 secondi, 24.7.19
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = ON             // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#ifdef ILI9341      // pcb arduino forgetIvrea32
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#else   // pcb radio 2019
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
// 1 su schedina PIC32 primissima, 2 su succ.
#endif
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ3

// DEVADC0

// DEVADC1

// DEVADC2

// DEVADC3

// DEVADC4

// DEVADC7



const char CopyrightString[]= {'T','M','S','9','9','0','0',' ','E','m','u','l','a','t','o','r',' ','v',
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0',' ','-',' ', '0','7','/','0','5','/','2','6', 0 };

const char Copyr1[]="(C) Dario's Automation 2022-2026 - G.Dar\xd\xa\x0";



// Global Variables:
extern BOOL fExit,debug;
extern BYTE ram_seg[];
extern BYTE rom_seg[],rom_seg2[],grom_seg[];
extern SWORD VICRaster;
extern BYTE TMS9918Reg[8],TMS9918RegS,TMS9918Sel,TMS9918WriteStage;
extern BYTE TMS9919[1]; 
extern WORD TMS9901Timer,TMS9901Cnt;
extern BYTE TMS9901[32];
extern BYTE TMSVideoRAM[TMSVIDEORAM_SIZE];
extern volatile BYTE TIMIRQ,VIDIRQ;
extern BOOL ColdReset;
extern BYTE CPUPins;
extern BYTE Keyboard[8];
volatile PIC32_RTCC_DATE currentDate={1,1,0};
volatile PIC32_RTCC_TIME currentTime={0,0,0};
const BYTE dayOfMonth[12]={31,28,31,30,31,30,31,31,30,31,30,31};



//https://www.angelfire.com/art2/unicorndreams/msx/RR-VDP.html#VDP-StatusReg
//#define DO_STRETCH 1
const WORD graphColors[16]={BLACK/*transparent*/,BLACK,LIGHTGREEN,BRIGHTGREEN, BLUE,BRIGHTBLUE,RED,BRIGHTCYAN,
  LIGHTRED,BRIGHTRED,YELLOW,LIGHTYELLOW, GREEN,MAGENTA,LIGHTGRAY,WHITE
	};
int UpdateScreen(SWORD rowIni, SWORD rowFin) {
	register int i;
	UINT16 px,py;
	int row1;
	int x;
	register BYTE *p1,*p2;
  BYTE ch1,ch2,color,color1,color0;

  // ci mette circa 1mS ogni passata... 4/1/23  @256*256 (erano 1.5 @512x256)
  
	// per SPI DMA https://www.microchip.com/forums/m1110777.aspx#1110777

  BYTE videoMode=((TMS9918Reg[1] & 0x18) >> 2) | ((TMS9918Reg[0] & 2) >> 1);
  WORD charAddress=((WORD)(TMS9918Reg[4] & 7)) << 11;
  WORD videoAddress=((WORD)(TMS9918Reg[2] & 0xf)) << 10;
  WORD colorAddress=((WORD)(TMS9918Reg[3])) << 6;
  WORD spriteAttrAddress=((WORD)(TMS9918Reg[5] & 0x7f)) << 7;
  WORD spritePatternAddress=((WORD)(TMS9918Reg[6] & 0x7)) << 11;
#ifdef DO_STRETCH
#define HORIZ_OFFSCREEN (320-((HORIZ_SIZE*9)/8))/2
#define VERT_OFFSCREEN (240-((VERT_SIZE*9)/8))/2
  BOOL lines9;
#else
#define HORIZ_OFFSCREEN (320-HORIZ_SIZE)/2
#define VERT_OFFSCREEN (240-VERT_SIZE)/2
#endif

  START_WRITE();
  
  if(!rowIni) {   // 
    setAddrWindow(0,0,_width,VERT_OFFSCREEN);
    for(py=0; py<VERT_OFFSCREEN; py++) {
#ifdef DO_STRETCH
      for(px=0; px<((HORIZ_SIZE*9)/8+HORIZ_OFFSCREEN*2); px++)
#else
      for(px=0; px<(HORIZ_SIZE+HORIZ_OFFSCREEN*2); px++)
#endif
        writedata16(graphColors[TMS9918Reg[7] & 0xf]);
      }
    }
  
  if(rowIni>=VERT_OFFSCREEN && rowIni<=MAX_RASTER-VERT_OFFSCREEN) {
    
//  LED3 = 1;
  
#ifdef DO_STRETCH
    setAddrWindow(0,(rowIni/8)*9,_width,9);
#else
    setAddrWindow(0,rowIni,_width,8 /*_height-VERT_OFFSCREEN*2*/);
#endif

//    rowIni-=VERT_OFFSCREEN;no!!

    if(!(TMS9918Reg[1] & 0b01000000)) {    // blanked
      for(py=0; py<8; py++)    // bordo/sfondo
        for(px=0; px<HORIZ_SIZE; px++)
          writedata16(graphColors[TMS9918Reg[7] & 0xf]);
      }
    else switch(videoMode) {    
      case 0:     // graphics 1
        for(py=rowIni/8; py<rowFin/8; py++) {    // 192 
					p2=((BYTE*)&TMSVideoRAM[videoAddress]) + ((py-VERT_OFFSCREEN/8)*HORIZ_SIZE/8);
#ifdef DO_STRETCH
          lines9=0;
#endif
          for(row1=0; row1<8; row1++) {    // 192 linee(32 righe char) 
						for(px=0; px<HORIZ_OFFSCREEN; px++)
              writedata16(graphColors[TMS9918Reg[7] & 0xf]);
            p1=p2;
						for(px=0; px<HORIZ_SIZE/8; px++) {    // 256 pixel 
              ch1=*p1++;
              ch2=TMSVideoRAM[charAddress + (ch1*8) + (row1)];
              color=TMSVideoRAM[colorAddress+ch1/8];
              color1=color >> 4; color0=color & 0xf;
							if(!color0)
								color0=TMS9918Reg[7] & 0xf;
							if(!color1)
								color1=TMS9918Reg[7] & 0xf;

              writedata16x2(graphColors[ch2 & 0b10000000 ? color1 : color0],
                      graphColors[ch2 & 0b01000000 ? color1 : color0]);
              writedata16x2(graphColors[ch2 & 0b00100000 ? color1 : color0],
                      graphColors[ch2 & 0b00010000 ? color1 : color0]);
              writedata16x2(graphColors[ch2 & 0b00001000 ? color1 : color0],
                      graphColors[ch2 & 0b00000100 ? color1 : color0]);
              writedata16x2(graphColors[ch2 & 0b00000010 ? color1 : color0],
                      graphColors[ch2 & 0b00000001 ? color1 : color0]);
#ifdef DO_STRETCH
              writedata16(graphColors[ch2 & 0b00000001 ? color1 : color0]);   // da 256 a 288
#endif
              }
						for(px=0; px<HORIZ_OFFSCREEN; px++)
              writedata16(graphColors[TMS9918Reg[7] & 0xf]);
#ifdef DO_STRETCH
            if(!lines9) {
              row1--;   // da 192 a 216
              lines9=1;
              }
#endif
            }
          ClrWdt();
          }
          
  handle_sprites:
        { // (OVVIAMENTE sarebbe meglio gestirli riga per riga...!
				struct SPRITE_ATTR *sa;
				BYTE ssize=TMS9918Reg[1] & 2 ? 32 : 8,smag=TMS9918Reg[1] & 1 ? 16 : 8;
				BYTE j2;
				int8_t j,smax;
				uint16_t sofs;

				for(smax=0; smax<32; smax++) {			// screen order inverso per cui mi serve sapere da dove iniziare
					sa=((struct SPRITE_ATTR *)&TMSVideoRAM[spriteAttrAddress+smax*sizeof(struct SPRITE_ATTR)]);
					if(sa->ypos>=LAST_SPRITE_YPOS)
						break;
					}
				smax--;

				for(i=smax; i>=0; i--) {			// screen order inverso!
					struct SPRITE_ATTR *sa2;
        
					sa=((struct SPRITE_ATTR *)&TMSVideoRAM[spriteAttrAddress+i*sizeof(struct SPRITE_ATTR)]);
					j2=smag*(ssize==32 ? 2 : 1);
					for(j=smax-1; j>=0; j--) {
						sa2=((struct SPRITE_ATTR *)&TMSVideoRAM[spriteAttrAddress+j*sizeof(struct SPRITE_ATTR)]);
						if((sa2->ypos>=sa->ypos && sa2->ypos<=sa->ypos+j2) &&
							(sa2->xpos>=sa->xpos && sa2->xpos<=sa->xpos+j2)) {
							// controllare solo i pixel accesi, a 1!
							TMS9918RegS |= 0b00100000;
							// poi ci sarebbe il flag 5 sprite per riga!
							}
						sa2++;
						}
        
					if(ssize==8)
						p1=((BYTE*)&TMSVideoRAM[spritePatternAddress]) + (((WORD)sa->name)*8); // (((WORD)sa->name)*ssize);
					else
						p1=((BYTE*)&TMSVideoRAM[spritePatternAddress]) + ((((WORD)sa->name) & 0xfc)*8);
					j=ssize;
					if(sa->ypos > 0xe1)     // Y diventa negativo..
						;
					color1=sa->color; color0=TMS9918Reg[7] & 0xf;

					
						if(sa->name==0x88 && sa->ypos>rowIni && sa->ypos<rowFin)			// debug
							x=1;

					if(sa->color==0)
						goto skippa_sprite;

					x=sa->eclock ? -32 : 0;     // X diventa negativo..
					x+=sa->xpos;
					sofs=0;
					for(py=0; py<ssize; py++,j--) {
						BYTE oldpixel;
						int y;

						if(ssize>8)
							y=(py & 15)+sa->ypos;
						else
							y=py+sa->ypos;
//						if(y+VERT_OFFSCREEN<rowIni || y+VERT_OFFSCREEN>rowFin)		non va così.. sistemare per velocizzare!
//							continue;
  	        setAddrWindow(x,y,smag,smag);   // ANCHE 32!! fare
						ch1=*p1++;

						if(x & 1)// FORSE bisognerebbe gestire posizione dispari...
							;
						if(smag==16) {
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b10000000)
								oldpixel = (color1) | (color1 << 4);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b01000000)
								oldpixel = (color1) | (color1 << 4);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b00100000)
								oldpixel = (color1) | (color1 << 4);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b00010000)
								oldpixel = (color1) | (color1 << 4);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b00001000)
								oldpixel = (color1) | (color1 << 4);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b00000100)
								oldpixel = (color1) | (color1 << 4);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b00000010)
								oldpixel = (color1) | (color1 << 4);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b00000001)
								oldpixel = (color1) | (color1 << 4);
	            writedata16(oldpixel); 
							}
						else {
	            writedata16(graphColors[ch2 & 0b10000000 ? color1 : color0]); 
							writedata16(graphColors[ch2 & 0b1000000 ? color1 : color0]); 
							writedata16(graphColors[ch2 & 0b100000 ? color1 : color0]); 
							writedata16(graphColors[ch2 & 0b10000 ? color1 : color0]); 
							writedata16(graphColors[ch2 & 0b1000 ? color1 : color0]); 
							writedata16(graphColors[ch2 & 0b100 ? color1 : color0]); 
							writedata16(graphColors[ch2 & 0b10 ? color1 : color0]); 
							writedata16(graphColors[ch2 & 0b1 ? color1 : color0]); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b10000000)
								oldpixel = (oldpixel & 0x0f) | (color1 << 4);
							if(ch1 & 0b01000000)
								oldpixel = (oldpixel & 0xf0) | (color1);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b00100000)
								oldpixel = (oldpixel & 0x0f) | (color1 << 4);
							if(ch1 & 0b00010000)
								oldpixel = (oldpixel & 0xf0) | (color1);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b00001000)
								oldpixel = (oldpixel & 0x0f) | (color1 << 4);
							if(ch1 & 0b00000100)
								oldpixel = (oldpixel & 0xf0) | (color1);
	            writedata16(oldpixel); 
							oldpixel=0 /**pTMSVideoRAM*/;    // LEGGERE esistente...
							if(ch1 & 0b00000010)
								oldpixel = (oldpixel & 0x0f) | (color1 << 4);
							if(ch1 & 0b00000001)
								oldpixel = (oldpixel & 0xf0) | (color1);
	            writedata16(oldpixel); 
							}

						switch(j) {   // gestisco i "quadranti" sprite messi a cazzo...
							case 23:
	//              setAddrWindow(sa->xpos/2,sa->ypos/2+8/2,8/2,8/2);
								sofs=0;
								break;
							case 15:
//								p1=((BYTE*)&TMSVideoRAM[spritePatternAddress]) + ((WORD)sa->name*ssize) + (16*ssize)/2;
	//              setAddrWindow(sa->xpos/2+8/2,sa->ypos/2,8/2,8/2);
								sofs=0+smag/2;
								if(x>239)		// migliorare bordo dx
									goto skippa_sprite;
								break;
							case 7:
	//              setAddrWindow(sa->xpos/2+8/2,sa->ypos/2+8/2,8/2,8/2);
//								pTMSVideoRAM=(BYTE*)&TMSVideoRAM[0]+(py+sa->ypos+8)*(((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2)))+sa->xpos+(8*ssize)/2;
								sofs=0+smag/2;
								break;
							default:
								break;
							}
						}

skippa_sprite:
						;
          }
        }
        break;
        
      case 1:     // graphics 2
#ifdef DO_STRETCH
        lines9=0;
#endif
        for(py=rowIni; py<rowFin/3; py++) {    // 192 linee 
					p1=((BYTE*)&TMSVideoRAM[videoAddress]) + ((py-VERT_OFFSCREEN/8)*HORIZ_SIZE/8);
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
					for(px=0; px<HORIZ_SIZE/8; px++) {    // 256 pixel 
            ch1=*p1++;
            ch2=TMSVideoRAM[charAddress + (ch1*8) + (py & 7)];
            color=TMSVideoRAM[colorAddress+py];
            color1=color >> 4; color0=color & 0xf;
						if(!color0)
							color0=TMS9918Reg[7] & 0xf;
						if(!color1)
							color1=TMS9918Reg[7] & 0xf;

            writedata16x2(graphColors[ch2 & 0b10000000 ? color1 : color0],
                    graphColors[ch2 & 0b01000000 ? color1 : color0]);
            writedata16x2(graphColors[ch2 & 0b00100000 ? color1 : color0],
                    graphColors[ch2 & 0b00010000 ? color1 : color0]);
            writedata16x2(graphColors[ch2 & 0b00001000 ? color1 : color0],
                    graphColors[ch2 & 0b00000100 ? color1 : color0]);
            writedata16x2(graphColors[ch2 & 0b00000010 ? color1 : color0],
                    graphColors[ch2 & 0b00000001 ? color1 : color0]);
#ifdef DO_STRETCH
            writedata16(graphColors[ch2 & 0b00000001 ? color1 : color0]);   // da 256 a 288
#endif
            }
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
#ifdef DO_STRETCH
          if(!lines9) {
            py--;   // da 192 a 216
            lines9=1;
            }
#endif
          ClrWdt();
          }
#ifdef DO_STRETCH
        lines9=0;
#endif
        for(py=rowFin/3; py<(rowFin*2)/3; py++) {    //
					p1=((BYTE*)&TMSVideoRAM[videoAddress]) + ((py-VERT_OFFSCREEN/8)*HORIZ_SIZE/8);
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
					for(px=0; px<HORIZ_SIZE/8; px++) {    // 256 pixel 
            ch1=*p1++;
            ch2=TMSVideoRAM[charAddress +2048 + (ch1*8) + (py & 7)];
            color=TMSVideoRAM[colorAddress+2048+py];
            color1=color >> 4; color0=color & 0xf;
						if(!color0)
							color0=TMS9918Reg[7] & 0xf;
						if(!color1)
							color1=TMS9918Reg[7] & 0xf;

            writedata16x2(graphColors[ch2 & 0b10000000 ? color1 : color0],
                    graphColors[ch2 & 0b01000000 ? color1 : color0]);
            writedata16x2(graphColors[ch2 & 0b00100000 ? color1 : color0],
                    graphColors[ch2 & 0b00010000 ? color1 : color0]);
            writedata16x2(graphColors[ch2 & 0b00001000 ? color1 : color0],
                    graphColors[ch2 & 0b00000100 ? color1 : color0]);
            writedata16x2(graphColors[ch2 & 0b00000010 ? color1 : color0],
                    graphColors[ch2 & 0b00000001 ? color1 : color0]);
#ifdef DO_STRETCH
            writedata16(graphColors[ch2 & 0b00000001 ? color1 : color0]);   // da 256 a 288
#endif
            }
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
#ifdef DO_STRETCH
          if(!lines9) {
            py--;   // da 192 a 216
            lines9=1;
            }
#endif
          ClrWdt();
          }
#ifdef DO_STRETCH
        lines9=0;
#endif
        for(py=(rowFin*2)/3; py<rowFin; py++) {    // 
					p1=((BYTE*)&TMSVideoRAM[videoAddress]) + ((py-VERT_OFFSCREEN/8)*HORIZ_SIZE/8);
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
					for(px=0; px<HORIZ_SIZE/8; px++) {    // 256 pixel 
            ch1=*p1++;
            ch2=TMSVideoRAM[charAddress +4096 + (ch1*8) + (py & 7)];
            color=TMSVideoRAM[colorAddress+4096+py];
            color1=color >> 4; color0=color & 0xf;
						if(!color0)
							color0=TMS9918Reg[7] & 0xf;
						if(!color1)
							color1=TMS9918Reg[7] & 0xf;

            writedata16x2(graphColors[ch2 & 0b10000000 ? color1 : color0],
                    graphColors[ch2 & 0b01000000 ? color1 : color0]);
            writedata16x2(graphColors[ch2 & 0b00100000 ? color1 : color0],
                    graphColors[ch2 & 0b00010000 ? color1 : color0]);
            writedata16x2(graphColors[ch2 & 0b00001000 ? color1 : color0],
                    graphColors[ch2 & 0b00000100 ? color1 : color0]);
            writedata16x2(graphColors[ch2 & 0b00000010 ? color1 : color0],
                    graphColors[ch2 & 0b00000001 ? color1 : color0]);
#ifdef DO_STRETCH
            writedata16(graphColors[ch2 & 0b00000001 ? color1 : color0]);   // da 256 a 288
#endif
            }
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
#ifdef DO_STRETCH
          if(!lines9) {
            py--;   // da 192 a 216
            lines9=1;
            }
#endif
          ClrWdt();
          }
        goto handle_sprites;
        break;
        
      case 2:     // multicolor
#ifdef DO_STRETCH
        lines9=0;
#endif
        for(py=rowIni; py<rowFin; py+=4) {    // 48 linee diventano 96
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
          p1=((BYTE*)&TMSVideoRAM[videoAddress]) + (py*16);
          ch1=*p1++;
          p2=((BYTE*)&TMSVideoRAM[charAddress+ch1]);
          for(px=0; px<HORIZ_SIZE; px+=2) {    // 64 pixel diventano 128
            ch2=*p2++;
            color=TMSVideoRAM[colorAddress+ch2];
            color1=color >> 4; color0=color & 0xf;

            // finire!!

            writedata16x2(graphColors[color1],graphColors[color0]);
#ifdef DO_STRETCH
// faer              writedata16(graphColors[ch2 & 0b00000001 ? color1 : color0]);   // da 256 a 288
#endif
            }
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
#ifdef DO_STRETCH
          if(!lines9) {
            py--;   // da 192 a 216
            lines9=1;
            }
#endif
          ClrWdt();
          }
        goto handle_sprites;
        break;
        
      case 4:     // text 32x24, ~120mS, O1 opp O2, 2/7/24
        color1=TMS9918Reg[7] >> 4; color0=TMS9918Reg[7] & 0xf;
        for(py=rowIni/8; py<rowFin/8; py++) {    // 192 linee(32 righe char) 
          p2=((BYTE*)&TMSVideoRAM[videoAddress]) + (py*40);
#ifdef DO_STRETCH
          lines9=0;
#endif
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
          for(row1=0; row1<8; row1++) {    // 192 linee(32 righe char) 
            p1=p2;
            for(px=0; px<40; px++) {    // 240 pixel 
              ch1=*p1++;
              ch2=TMSVideoRAM[charAddress + (ch1*8) + (row1)];

              writedata16x2(graphColors[ch2 & 0b10000000 ? color1 : color0],
                      graphColors[ch2 & 0b01000000 ? color1 : color0]);
              writedata16x2(graphColors[ch2 & 0b00100000 ? color1 : color0],
                      graphColors[ch2 & 0b00010000 ? color1 : color0]);
              writedata16x2(graphColors[ch2 & 0b00001000 ? color1 : color0],
                      graphColors[ch2 & 0b00000100 ? color1 : color0]);
#ifdef DO_STRETCH
              writedata16(graphColors[ch2 & 0b00000001 ? color1 : color0]);   // da 240 a 320
#endif
              }
  //          for(px=0; px<8; px++)     // 240 -> 256
  //            writedata16x2(0,0);
#ifdef DO_STRETCH
            if(!lines9) {
              row1--;   // da 192 a 216
              lines9=1;
              }
#endif
            }
  				for(px=0; px<HORIZ_OFFSCREEN; px++)
            writedata16(graphColors[TMS9918Reg[7] & 0xf]);
          ClrWdt();
          }
        break;
      }
    }

  if(rowFin>=_height-VERT_OFFSCREEN) {
    setAddrWindow(0,_height-VERT_OFFSCREEN,_width,VERT_OFFSCREEN);
    for(py=0; py<VERT_OFFSCREEN; py++) {
#ifdef DO_STRETCH
      for(px=0; px<((HORIZ_SIZE*9)/8+HORIZ_OFFSCREEN*2); px++)
#else
      for(px=0; px<(HORIZ_SIZE+HORIZ_OFFSCREEN*2); px++)
#endif
        writedata16(graphColors[TMS9918Reg[7] & 0xf]);
      }
    }
  

  END_WRITE();    // 
  

  //	writecommand(CMD_NOP);
    
//    LED3 = 0;

  }  
  


extern const unsigned char TI994A_BIN_GROM0[],TI994A_BIN_GROM1[],TI994A_BIN_GROM2[],
    TI994A_BIN2[],TI994A_BIN_U610[],TI994A_BIN_U611[];

int main(void) {
  int i;

  // disable JTAG port
//  DDPCONbits.JTAGEN = 0;
  
  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  CFGCONbits.IOLOCK = 0;      // PPS Unlock
  SYSKEY = 0x00000000;
#ifdef ST7735
  RPB15Rbits.RPB15R = 4;        // Assign RPB15 as U6TX, pin 30
  U6RXRbits.U6RXR = 2;      // Assign RPB14 as U6RX, pin 29 
#ifdef USA_SPI_HW
  RPG8Rbits.RPG8R = 6;        // Assign RPG8 as SDO2, pin 6
//  SDI2Rbits.SDI2R = 1;        // Assign RPG7 as SDI2, pin 5
#endif
  RPD5Rbits.RPD5R = 12;        // Assign RPD5 as OC1, pin 53; vaga uscita audio :)
  RPD1Rbits.RPD1R = 12;        // Assign RPD1 as OC1, pin 49; buzzer
#endif
#ifdef ILI9341
  RPB9Rbits.RPB9R = 1;        // Assign RPB9 as U3TX, pin 22 (ok arduino)
// NON SI PUO'... sistemare se serve  U3RXRbits.U3RXR = 2;      // Assign RPB8 as U3RX, pin 21 (ok arduino)
//  RPB9Rbits.RPB9R = 1;        // Assign RPB9 as U3TX, pin 22 IO1
//  U4RXRbits.U4RXR = 2;      // Assign RPB8 as U4RX, pin 21 IO0 NON VA come coppia sulla stessa UART... :( se serve...

  RPB1Rbits.RPB1R = 12;       // Assign RPB1 as OC7, pin 15
//  PPSOutput(4,RPB1,OC7);   //buzzer 4KHz , qua rimappabile 
#endif

  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  CFGCONbits.IOLOCK = 1;      // PPS Lock
  SYSKEY = 0x00000000;


//#define DEBUG_TESTREFCLK  
#ifdef DEBUG_TESTREFCLK
// test REFCLK
#ifdef ST7735
  PPSOutput(4,RPC4,REFCLKO2);   // RefClk su pin 1 (RG15, buzzer)
	REFOCONbits.ROSSLP=1;
	REFOCONbits.ROSEL=1;
	REFOCONbits.RODIV=0;
	REFOCONbits.ROON=1;
	TRISFbits.TRISF3=1;
#endif
#ifdef ILI9341
//  RPD9Rbits.RPD9R = 15;        // Assign RPD9 (SDA) as RefClk3, pin 43
  RPB8Rbits.RPB8R = 15;        // Assign RPB8 as RefClk3, pin 21 + comodo
	REFO3CONbits.SIDL=0;
	REFO3CONbits.RSLP=1;
	REFO3CONbits.ROSEL=1;   // PBclk
	REFO3CONbits.RODIV=1;   // :2
	REFO3CONbits.OE=1;
	REFO3CONbits.ON=1;
//	TRISDbits.TRISD9=1;
	TRISBbits.TRISB8=1;
#endif
#endif

//	PPSLock;

   // Disable all Interrupts
  __builtin_disable_interrupts();
  
//  SPLLCONbits.PLLMULT=10;
  
  OSCTUN=0;
  OSCCONbits.FRCDIV=0;
  
  // Switch to FRCDIV, SYSCLK=8MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x00; // FRC
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
    // At this point, SYSCLK is ~8MHz derived directly from FRC
 //http://www.microchip.com/forums/m840347.aspx
  // Switch back to FRCPLL, SYSCLK=200MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x01; // SPLL
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
  // At this point, SYSCLK is ~200MHz derived from FRC+PLL
//***
  mySYSTEMConfigPerformance();
  //myINTEnableSystemMultiVectoredInt(();

    
#ifdef ST7735
	TRISB=0b0000000000110000;			// AN4,5 (rb4..5)
	TRISC=0b0000000000000000;
	TRISD=0b0000000000001100;			// 2 pulsanti
	TRISE=0b0000000000000000;			// 3 led
	TRISF=0b0000000000000000;			// 
	TRISG=0b0000000000000000;			// SPI2 (rg6..8)

  ANSELB=0;
  ANSELE=0;
  ANSELG=0;

  CNPUDbits.CNPUD2=1;   // switch/pulsanti
  CNPUDbits.CNPUD3=1;
  CNPUGbits.CNPUG6=1;   // I2C tanto per
  CNPUGbits.CNPUG8=1;  
#endif
#ifdef ILI9341

	TRISB=0b0000000000000001;			// pulsante; [ AN ?? ] ; buzzer
	TRISC=0b0000000000000000;
	TRISD=0b0000000000000000;			// 2led
	TRISE=0b0000000000000000;			// led
	TRISF=0b0000000000000001;			// pulsante
	TRISG=0b0000000000000000;			// SPI2 (rg6..8)

  ANSELB=0;
  ANSELE=0;
  ANSELG=0;

  CNPUFbits.CNPUF0=1;   // switch/pulsanti
  CNPUBbits.CNPUB0=1;
  CNPUDbits.CNPUD9=1;   // I2C tanto per
  CNPUDbits.CNPUD10=1;  
#endif
      
//  ShortDelay(50000); 
//  __delay_ms(50);
  
  Timer_Init();
  PWM_Init();
//  UART_Init(/*230400L*/ 115200L);     // si blocca boh qua 5/5/26 #cancrojuventini #leucemiagiulianoleone

  myINTEnableSystemMultiVectoredInt();

  
//    	ColdReset=0;    Emulate(0);

#ifndef USING_SIMULATOR
//#ifndef __DEBUG
#ifdef ST7735
  Adafruit_ST7735_1(0,0,0,0,-1);
  Adafruit_ST7735_initR(INITR_BLACKTAB);
#endif
#ifdef ILI9341
  Adafruit_ILI9341_8(8, 9, 10, 11, 12, 13, 14);
	begin(0);
  __delay_ms(200);
#endif
  
//  displayInit(NULL);
  
#ifdef m_LCDBLBit
  m_LCDBLBit=1;
#endif
  
//	begin();
	clearScreen();

// init done
	setTextWrap(1);
//	setTextColor2(WHITE, BLACK);

	drawBG();
  
  __delay_ms(500);
	clearScreen();

/*  __delay_ms(200);
	gfx_fillRect(3,_TFTHEIGHT-20,_TFTWIDTH-6,16,BLACK);
 	setTextColor(BLUE);
	LCDXY(1,13);
	gfx_print("(emulating Ti99/4A)");
  __delay_ms(1000);*/


//#endif
#endif
  

//  memcpy(rom_seg,TI994A_BIN,0x1800);
  memcpy(grom_seg,TI994A_BIN_GROM0,0x1800);
  memcpy(grom_seg+0x2000,TI994A_BIN_GROM1,0x1800);
  memcpy(grom_seg+0x2000*2,TI994A_BIN_GROM2,0x1800);
  for(i=0; i<8192; i+=2) {
    rom_seg[i+1]=TI994A_BIN_U611[i/2];
    rom_seg[i]=TI994A_BIN_U610[i/2];
    }

        
	ColdReset=0;

  initHW();
  Emulate(0);

  }


enum CACHE_MODE {
  UNCACHED=0x02,
  WB_WA=0x03,
  WT_WA=0x01,
  WT_NWA=0x00,
/* Cache Coherency Attributes */
//#define _CACHE_WRITEBACK_WRITEALLOCATE      3
//#define _CACHE_WRITETHROUGH_WRITEALLOCATE   1
//#define _CACHE_WRITETHROUGH_NOWRITEALLOCATE 0
//#define _CACHE_DISABLE                      2
  };
void mySYSTEMConfigPerformance(void) {
  unsigned PLLIDIV;
  unsigned PLLMUL;
  unsigned PLLODIV;
  float CLK2USEC;
  unsigned SYSCLK;
  unsigned char PLLODIVVAL[]={
    2,2,4,8,16,32,32,32
    };
	unsigned int cp0;

  PLLIDIV=SPLLCONbits.PLLIDIV+1;
  PLLMUL=SPLLCONbits.PLLMULT+1;
  PLLODIV=PLLODIVVAL[SPLLCONbits.PLLODIV];

  SYSCLK=(FOSC*PLLMUL)/(PLLIDIV*PLLODIV);
  CLK2USEC=SYSCLK/1000000.0f;

  SYSKEY = 0x0;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;

  if(SYSCLK<=60000000)
    PRECONbits.PFMWS=0;
  else if(SYSCLK<=120000000)
    PRECONbits.PFMWS=1;
  else if(SYSCLK<=200000000)
    PRECONbits.PFMWS=2;
  else if(SYSCLK<=252000000)
    PRECONbits.PFMWS=4;
  else
    PRECONbits.PFMWS=7;

  PRECONbits.PFMSECEN=0;    // non c'è nella versione "2019" ...
  PRECONbits.PREFEN=0b10;

  SYSKEY = 0x0;

  // Set up caching
  cp0 = _mfc0(16, 0);
  cp0 &= ~0x07;
  cp0 |= WB_WA /*0b011*/; // K0 = Cacheable, non-coherent, write-back, write allocate
  _mtc0(16, 0, cp0);  
  }

void myINTEnableSystemMultiVectoredInt(void) {

  PRISS = 0x76543210;
  INTCONSET = _INTCON_MVEC_MASK /*0x1000*/;    //MVEC
  asm volatile ("ei");
  //__builtin_enable_interrupts();
  }

/* CP0.Count counts at half the CPU rate */
#define TICK_HZ (CPU_HZ / 2)

/* wait at least usec microseconds */
#if 0
void delay_usec(unsigned long usec) {
unsigned long start, stop;

  /* get start ticks */
  start = readCP0Count();

  /* calculate number of ticks for the given number of microseconds */
  stop = (usec * 1000000) / TICK_HZ;

  /* add start value */
  stop += start;

  /* wait till Count reaches the stop value */
  while (readCP0Count() < stop)
    ;
  }
#endif

void xdelay_us(uint32_t us) {
  
  if(us == 0) {
    return;
    }
  unsigned long start_count = ReadCoreTimer /*_CP0_GET_COUNT*/();
  unsigned long now_count;
  long cycles = ((GetSystemClock() + 1000000U) / 2000000U) * us;
  do {
    now_count = ReadCoreTimer /*_CP0_GET_COUNT*/();
    } while ((unsigned long)(now_count-start_count) < cycles);
  }

void __attribute__((used)) DelayUs(unsigned int usec) {
  unsigned int tWait, tStart;

  tWait=(GetSystemClock()/2000000)*usec;
  tStart=_mfc0(9,0);
  while((_mfc0(9,0)-tStart)<tWait)
    ClrWdt();        // wait for the time to pass
  }

void __attribute__((used)) DelayMs(unsigned int ms) {
  
  for(;ms;ms--)
    DelayUs(1000);
  }

// ===========================================================================
// ShortDelay - Delays (blocking) for a very short period (in CoreTimer Ticks)
// ---------------------------------------------------------------------------
// The DelayCount is specified in Core-Timer Ticks.
// This function uses the CoreTimer to determine the length of the delay.
// The CoreTimer runs at half the system clock. 100MHz
// If CPU_CLOCK_HZ is defined as 80000000UL, 80MHz/2 = 40MHz or 1LSB = 25nS).
// Use US_TO_CT_TICKS to convert from uS to CoreTimer Ticks.
// ---------------------------------------------------------------------------

void ShortDelay(                       // Short Delay
  DWORD DelayCount)                   // Delay Time (CoreTimer Ticks)
{
  DWORD StartTime;                    // Start Time
  StartTime = ReadCoreTimer();         // Get CoreTimer value for StartTime
  while( (DWORD)(ReadCoreTimer() - StartTime) < DelayCount)
    ClrWdt();
  }
 

void Timer_Init(void) {

  T2CON=0;
  T2CONbits.TCS = 0;                  // clock from peripheral clock
  T2CONbits.TCKPS = 0b111;            // 1:256 prescaler (pwm clock=1.4KHz circa se buzzer viene impostato a 0x50)
  T2CONbits.T32 = 0;                  // 16bit
//  PR2 = 2000;                         // rollover every n clocks; 2000 = 50KHz
  PR2 = 65535;                         // per ora faccio solo onda quadra
  T2CONbits.TON = 1;                  // start timer per PWM
  
  // TIMER 3 INITIALIZATION (TIMER IS USED AS A TRIGGER SOURCE FOR ALL CHANNELS).
  T3CON=0;
  T3CONbits.TCS = 0;                  // clock from peripheral clock
  T3CONbits.TCKPS = 0b100;            // 1:16 prescaler
  PR3 = (GetPeripheralClock()/16)/1600;         // 1600Hz
  T3CONbits.TON = 1;                  // start timer 

  IPC3bits.T3IP=4;            // set IPL 4, sub-priority 2??
  IPC3bits.T3IS=0;
  IEC0bits.T3IE=1;             // enable Timer 3 interrupt se si vuole

	}

void PWM_Init(void) {

  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  CFGCONbits.OCACLK=0;      // sceglie timer per PWM
  SYSKEY = 0x00000000;
  
#ifdef ST7735
  OC1CON = 0x0006;      // TimerX ossia Timer2; PWM mode no fault; Timer 16bit, TimerX
//  OC1R    = 500;		 // su PIC32 è read-only!
//  OC1RS   = 1000;   // 50%, relativo a PR2 del Timer2
  OC1R    = 32768;		 // su PIC32 è read-only!
  OC1RS   = 0;        // per ora faccio solo onda quadra, v. SID reg. 0-1
  OC1CONbits.ON = 1;   // on
#endif
#ifdef ILI9341
  OC7CON = 0x0006;      // TimerX ossia Timer2; PWM mode no fault; Timer 16bit, TimerX
//  OC7R    = 500;		 // su PIC32 è read-only!
//  OC7RS   = 1000;   // 50%, relativo a PR2 del Timer2
  OC7R    = 32768;		 // su PIC32 è read-only!
  OC7RS   = 0;        // per ora faccio solo onda quadra, v. SID reg. 0-1
  OC7CONbits.ON = 1;   // on
#endif

  }

void UART_Init(DWORD baudRate) {
  
  U3MODE=0b0000000000001000;    // BRGH=1
  U3STA= 0b0000010000000000;    // TXEN
  DWORD baudRateDivider = ((GetPeripheralClock()/(4*baudRate))-1);
  U3BRG=baudRateDivider;
  U3MODEbits.ON=1;
 
  }

char BusyUART1(void) {
  
  return(!U3STAbits.TRMT);
  }

char DataRdyUART1(void) {
  
  return(U3STAbits.URXDA);
  }

void putsUART1(unsigned int *buffer) {
  char *temp_ptr = (char *)buffer;

    // transmit till NULL character is encountered 
  if(U3MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer) {
            while(U3STAbits.UTXBF); /* wait if the buffer is full */
            U3TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
  else {
        while(*temp_ptr) {
            while(U3STAbits.UTXBF);  /* wait if the buffer is full */
            U3TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
  }

unsigned int ReadUART1(void) {
  
  if(U3MODEbits.PDSEL == 3)
    return (U3RXREG);
  else
    return (U3RXREG & 0xFF);
  }

void WriteUART1(unsigned int data) {
  
  if(U3MODEbits.PDSEL == 3)
    U3TXREG = data;
  else
    U3TXREG = data & 0xFF;
  }


void __ISR(_UART3_RX_VECTOR) UART3_ISR(void) {

#warning VERIFICARE UART!!!!
  LED3 ^= 1;    // LED to indicate the ISR.
  char curChar = U3RXREG;
  IFS4bits.U3RXIF = 0;  // Clear the interrupt flag!
  }


int emulateKBD(BYTE ch) {
  int i;
  
	switch(toupper(ch)) {   // solo maiuscole qua..
    case 0:
      for(i=0; i<8; i++)
        Keyboard[i]=0xff;
      // FORSE non dovremmo rilasciare i modifier, qua???
      break;
    case ' ':		//VK_SPACE
      Keyboard[1]&=~0b10000000;
      break;
    case 'A':
      Keyboard[5]&=~0b00000100;
      break;
    case 'B':
      Keyboard[7]&=~0b00001000;
      break;
    case 'C':
      Keyboard[7]&=~0b00100000;
      break;
    case 'D':
      Keyboard[5]&=~0b00100000;
      break;
    case 'E':
      Keyboard[6]&=~0b00100000;
      break;
    case 'F':
      Keyboard[5]&=~0b00010000;
      break;
    case 'G':
      Keyboard[5]&=~0b00001000;
      break;
    case 'H':
      Keyboard[1]&=~0b00001000;
      break;
    case 'I':
      Keyboard[2]&=~0b00100000;
      break;
    case 'J':
      Keyboard[1]&=~0b00010000;
      break;
    case 'K':
      Keyboard[1]&=~0b00100000;
      break;
    case 'L':
      Keyboard[1]&=~0b01000000;
      break;
    case 'M':
      Keyboard[0]&=~0b00010000;
      break;
    case 'N':
      Keyboard[0]&=~0b00001000;
      break;
    case 'O':
      Keyboard[2]&=~0b01000000;
      break;
    case 'P':
      Keyboard[2]&=~0b00000100;
      break;
    case 'Q':
      Keyboard[6]&=~0b00000100;
      break;
    case 'R':
      Keyboard[6]&=~0b00010000;
      break;
    case 'S':
      Keyboard[5]&=~0b01000000;
      break;
    case 'T':
      Keyboard[6]&=~0b00001000;
      break;
    case 'U':
      Keyboard[2]&=~0b00010000;
      break;
    case 'V':
      Keyboard[7]&=~0b00010000;
      break;
    case 'W':
      Keyboard[6]&=~0b01000000;
      break;
    case 'X':
      Keyboard[7]&=~0b01000000;
      break;
    case 'Y':
      Keyboard[2]&=~0b00001000;
      break;
    case 'Z':
      Keyboard[7]&=~0b00000100;
      break;
    case '0':
      Keyboard[3]&=~0b00000100;
      break;
    case '1':
      Keyboard[4]&=~0b00000100;
      break;
    case '2':
      Keyboard[4]&=~0b01000000;
      break;
    case '3':
      Keyboard[4]&=~0b00100000;
      break;
    case '4':
      Keyboard[4]&=~0b00010000;
      break;
    case '5':
      Keyboard[4]&=~0b00001000;
      break;
    case '6':
      Keyboard[3]&=~0b00001000;
      break;
    case '7':
      Keyboard[3]&=~0b00010000;
      break;
    case '8':
      Keyboard[3]&=~0b00100000;
      break;
    case '9':
      Keyboard[3]&=~0b01000000;
      break;
/*			case VK_HOME:
      break;
    case VK_DOWN:
      Keyboard[4]&=~0b10000000;
      Keyboard[7]&=~0b01000000;
      break;
    case VK_RIGHT:
      Keyboard[4]&=~0b10000000;
      Keyboard[5]&=~0b00100000;
      break;
    case VK_UP:
      Keyboard[4]&=~0b10000000;
      Keyboard[6]&=~0b00100000;
      break;
    case VK_LEFT:
      Keyboard[4]&=~0b10000000;
      Keyboard[5]&=~0b01000000;
      break;
    case VK_LSHIFT:
    case VK_SHIFT:
    case VK_RSHIFT:
      Keyboard[5]&=~0b10000000;
      break;
    case VK_CONTROL:
      Keyboard[6]&=~0b10000000;
      break;
    case VK_MENU:     // alt ossia FCTN
      Keyboard[4]&=~0b10000000;
      break;
    case VK_CAPITAL:
      break;
    case VK_MULTIPLY:
      Keyboard[5]&=~0b10000000;
      Keyboard[3]&=~0b00100000;
      break;
    case VK_SUBTRACT:
      Keyboard[5]&=~0b10000000;
      Keyboard[0]&=~0b00000100;
      break;
    case 0xdb:		// '?
      break;
    case 0xdd:		// ì^
      Keyboard[0]&=~0b10000000);
      break;*/
    case '\"':		// à#  faccio "
      Keyboard[4]&=~0b10000000;
      Keyboard[2]&=~0b00000100;
      break;
    case 0xba:		// è
      Keyboard[4]&=~0b10000000;
      Keyboard[6]&=~0b00010000;			// sarebbe R o F a seconda
      break;
    case 0xbb:
      Keyboard[4]&=~0b10000000;
      Keyboard[6]&=~0b00001000;			// sarebbe T o G a seconda
      break;
    case '+':
      Keyboard[5]&=~0b10000000;
      Keyboard[0]&=~0b10000000;
      break;
    case 0xc0:		// ò@
      break;
    case '~':		// ù		~
      Keyboard[4]&=~0b10000000;
      Keyboard[2]&=~0b01000000;
      break;
    case ',':		// ,
      Keyboard[0]&=~0b00100000;
      break;
//			case VK_DECIMAL:
//		case VK_OEM_PERIOD          /*'.'*/:
    case '.':		// .
      Keyboard[0]&=~0b01000000;
      break;
    case '/':		// -_
//			case VK_DIVIDE:
      Keyboard[0]&=~0b00000100;
      break;
    case '\r':
//			case VK_RETURN:
      Keyboard[2]&=~0b10000000;
      break;
    case '\x8':			// FCNT 9
      Keyboard[4]&=~0b10000000;
//				Keyboard[3]&=~0b01000000);		// non va... forse extended basic?
      Keyboard[5]&=~0b01000000;		// faccio left quindi
      break;
//			case VK_DELETE:		// FCNT 1
//				Keyboard[4]&=~0b10000000;
//				Keyboard[4]&=~0b00000100;
//				break;
    case 0xa1 /*VK_F1*/:
      Keyboard[4]&=~0b10000000;
      Keyboard[4]&=~0b00000100;
      break;
    case 0xa2 /*VK_F2*/:
      break;
    case 0xa3 /*VK_F3*/:				// FCNT 3
      Keyboard[4]&=~0b10000000;
      Keyboard[4]&=~0b00100000;
      break;
    case 0xa4 /*VK_F4*/:				// FCNT 4
      Keyboard[4]&=~0b10000000;
      Keyboard[4]&=~0b00010000;
      break;
    case 0xa5 /*VK_F5*/:
      break;
    case 0xa6 /*VK_F6*/:
      break;
    case 0xa7 /*VK_F7*/:
      break;
    case 0xa8 /*VK_F8*/:				// FCNT 8
      Keyboard[4]&=~0b10000000;
      Keyboard[3]&=~0b00100000;
      break;
    case '|':		// |\ :)
      Keyboard[4]&=~0b10000000;
      Keyboard[5]&=~0b00000100;		// sarebbe A opp. Z a seconda del simbolo...
      break;
    case '<':		// <  :)
      Keyboard[4]&=~0b10000000;
      Keyboard[7]&=~0b00000100;
      break;
    case '\x1b' /*VK_ESCAPE*/:		// faccio come CLEAR ossia FCTN 4
      Keyboard[4]&=~0b10000000;
      Keyboard[4]&=~0b00010000;
      break;
//			case VK_PAUSE:
//				break;
    case 0xac /*VK_F12*/:			// simulo reset! FCNT =
      Keyboard[4]&=~0b10000000;
      Keyboard[0]&=~0b10000000;
      break;
    }
  
 
  
no_irq:
    ;
  }

BYTE whichKeysFeed=0;
char keysFeed[32]={0};
volatile BYTE keysFeedPtr=255;
const char *keysFeed1="\r";   // 
const char *keysFeed2="1\r";   // 1 
const char *keysFeed3="10 PRINT 3.14\r";   //
const char *keysFeed4="20 GOTO 10\r";   // 
const char *keysFeed5="LIST\r";   //
const char *keysFeed6="RUN\r";   // 

void __ISR(_TIMER_3_VECTOR,ipl4SRS) TMR_ISR(void) {
// https://www.microchip.com/forums/m842396.aspx per IRQ priority ecc
  static BYTE divider,divider2;
  static WORD dividerTim;
  static WORD dividerEmulKbd;
  static BYTE keysFeedPhase=0;
  int i;

#define TIMIRQ_DIVIDER 32   // 50Hz
  
  LED3 ^= 1;      // check timing: 1600Hz, 7/5/26 (fuck mankind
  
  divider++;


  dividerTim++;
  if(dividerTim>=1600) {   // 1Hz RTC
		} 
  

  if(keysFeedPtr==255)      // EOL
    goto fine;
  if(keysFeedPtr==254) {    // NEW string
    keysFeedPtr=0;
    keysFeedPhase=0;
		switch(whichKeysFeed) {
			case 0:
				strcpy(keysFeed,keysFeed1);
				break;
			case 1:
				strcpy(keysFeed,keysFeed2);
				break;
			case 2:
				strcpy(keysFeed,keysFeed3);
				break;
			case 3:
				strcpy(keysFeed,keysFeed4);
				break;
			case 4:
				strcpy(keysFeed,keysFeed5);
				break;
			case 5:
				strcpy(keysFeed,keysFeed6);
				break;
			}
		whichKeysFeed++;
		if(whichKeysFeed>5)
			whichKeysFeed=0;
//    goto fine;
		}
  if(keysFeed[keysFeedPtr]) {
    dividerEmulKbd++;
    if(dividerEmulKbd>=500 /*300*/) {   // ~.2Hz per emulazione tastiera! (più veloce di tot non va...))
      dividerEmulKbd=0;
      if(!keysFeedPhase) {
        keysFeedPhase=1;
        emulateKBD(keysFeed[keysFeedPtr]);
        }
      else {
        keysFeedPhase=0;
        emulateKBD(NULL);
        keysFeedPtr++;
wait_kbd: ;
        }
      }
    }
  else
    keysFeedPtr=255;
    
fine:
  IFS0CLR = _IFS0_T3IF_MASK;
  }

// ---------------------------------------------------------------------------------------
// declared static in case exception condition would prevent
// auto variable being created
static enum {
	EXCEP_IRQ = 0,			// interrupt
	EXCEP_AdEL = 4,			// address error exception (load or ifetch)
	EXCEP_AdES,				// address error exception (store)
	EXCEP_IBE,				// bus error (ifetch)
	EXCEP_DBE,				// bus error (load/store)
	EXCEP_Sys,				// syscall
	EXCEP_Bp,				// breakpoint
	EXCEP_RI,				// reserved instruction
	EXCEP_CpU,				// coprocessor unusable
	EXCEP_Overflow,			// arithmetic overflow
	EXCEP_Trap,				// trap (possible divide by zero)
	EXCEP_IS1 = 16,			// implementation specfic 1
	EXCEP_CEU,				// CorExtend Unuseable
	EXCEP_C2E				// coprocessor 2
  } _excep_code;

static unsigned int _epc_code;
static unsigned int _excep_addr;

void __attribute__((weak)) _general_exception_handler(uint32_t __attribute__((unused)) code, uint32_t __attribute__((unused)) address) {
  }

void __attribute__((nomips16,used)) _general_exception_handler_entry(void) {
  
	asm volatile("mfc0 %0,$13" : "=r" (_epc_code));
	asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

	_excep_code = (_epc_code & 0x0000007C) >> 2;

  _general_exception_handler(_excep_code, _excep_addr);

	while (1)	{
		// Examine _excep_code to identify the type of exception
		// Examine _excep_addr to find the address that caused the exception
    }
  }


