//---------------------------------------------------------------------------
//
#ifndef _TMS9900_PIC_INCLUDED
#define _TMS9900_PIC_INCLUDED

//---------------------------------------------------------------------------


/* check if build is for a real debug tool */
#if defined(__DEBUG) && !defined(__MPLAB_ICD2_) && !defined(__MPLAB_ICD3_) && \
   !defined(__MPLAB_PICKIT2__) && !defined(__MPLAB_PICKIT3__) && \
   !defined(__MPLAB_REALICE__) && \
   !defined(__MPLAB_DEBUGGER_REAL_ICE) && \
   !defined(__MPLAB_DEBUGGER_ICD3) && \
   !defined(__MPLAB_DEBUGGER_PK3) && \
   !defined(__MPLAB_DEBUGGER_PICKIT2) && \
   !defined(__MPLAB_DEBUGGER_PIC32MXSK)
    #warning Debug with broken MPLAB simulator
    #define USING_SIMULATOR
#endif


#include <stdint.h>


//#define TMS9940 1
#define CPU_CLOCK_DIVIDER 250000L		// 
#define HW_CLOCK_DIVIDER 32000L		// dice che il timer viaggia a 1:64 Phi... diciamo che la CPU giri a 1/4Phi ergo...

#define FCY 204000000ul    //Oscillator frequency; ricontrollato con baud rate, pare giusto così!

#define CPU_CLOCK_HZ             (FCY)    // CPU Clock Speed in Hz
#define CPU_CT_HZ            (CPU_CLOCK_HZ/2)    // CPU CoreTimer   in Hz
#define PERIPHERAL_CLOCK_HZ      (FCY/2 /*100000000UL*/)    // Peripheral Bus  in Hz
#define GetSystemClock()         (FCY)    // CPU Clock Speed in Hz
#define GetPeripheralClock()     (PERIPHERAL_CLOCK_HZ)    // Peripheral Bus  in Hz
#define FOSC 8000000ul

#define US_TO_CT_TICKS  (CPU_CT_HZ/1000000UL)    // uS to CoreTimer Ticks
    
#define VERNUML 1
#define VERNUMH 1


#define MIN_RASTER 0        // noi visualizziamo da 48 a 248
#define MAX_RASTER 239      //191
#define HORIZ_SIZE 256
#define VERT_SIZE 192


typedef char BOOL;
typedef unsigned char UINT8;
typedef unsigned char BYTE;
typedef signed char INT8;
typedef unsigned short int WORD;
typedef unsigned short int SWORD;       // v. C64: con int/32bit è più veloce!
typedef unsigned long UINT32;
typedef unsigned long DWORD;
typedef signed long INT32;
typedef unsigned short int UINT16;
typedef signed int INT16;

typedef DWORD COLORREF;

#define RGB(r,g,b)      ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))


#define TRUE 1
#define FALSE 0


union __attribute__((__packed__)) PIPE {
	SWORD x;
//	BYTE bb[4];
	struct {
		BYTE l;
		BYTE h;
		} b;
	};

union __attribute__((__packed__)) T_REG {
  SWORD x;
  struct { 
    BYTE l;
    BYTE h;
    } b;
  };
union __attribute__((__packed__)) T_REGISTERS {
  BYTE b[32];
  union T_REG r[16];
  };
#define ID_LG 0x8000
#define ID_AG 0x4000
#define ID_ZERO 0x2000
#define ID_CARRY 0x1000
#define ID_OVF 0x0800
#define ID_PARITY 0x0400
#define ID_XOP 0x0200
#ifdef TMS9940 
#define ID_DIGITCARRY 0x100  // boh!
  // v. anche 990/10?? b7=PR b8=MF
#endif
#define ID_INTERRUPTMASK 0x000F
union __attribute__((__packed__)) REGISTRO_F {
  SWORD x;
  struct {
    unsigned int InterruptMask: 4;
    unsigned int unused2: 4;
#ifdef TMS9940 
//      unsigned int MapFile: 1;   // 
//      unsigned int Privileged: 1;   // v. anche 990/10?? b7=PR b8=MF
    unsigned int DigitCarry: 1;   // boh
#else
    unsigned int unused: 1;
#endif
    unsigned int XOP: 1;
    unsigned int Parity: 1;   // 0=pari 1=dispari (ODD PARITY)
    unsigned int Overflow: 1;
    unsigned int Carry: 1;
    unsigned int Zero: 1;
    unsigned int ArithmeticGreater: 1;
    unsigned int LogicalGreater: 1;
    };
  };
union __attribute__((__packed__)) OPERAND {
  BYTE *reg8;
  SWORD *reg16;
  SWORD mem;
  };
union __attribute__((__packed__)) RESULT {
  struct {
    BYTE l;
    BYTE h;
    } b;
  SWORD x;
  DWORD d;
  };


#ifdef ST7735
#define _TFTWIDTH  		160     //the REAL W resolution of the TFT
#define _TFTHEIGHT 		128     //the REAL H resolution of the TFT
typedef signed char GRAPH_COORD_T;
typedef unsigned char UGRAPH_COORD_T;
#endif
#ifdef ILI9341
#define _TFTWIDTH  		320     //the REAL W resolution of the TFT
#define _TFTHEIGHT 		240     //the REAL H resolution of the TFT
typedef signed short int GRAPH_COORD_T;
typedef unsigned short int UGRAPH_COORD_T;
#endif
typedef WORD GFX_COLOR;

#define TMS99xx_BASE 0x8800
#define TMSVIDEORAM_SIZE 16384
#define TMS_R0_MODE_GRAPHICS_I    0x00
#define TMS_R0_MODE_GRAPHICS_II   0x02
#define TMS_R0_MODE_MULTICOLOR    0x00
#define TMS_R0_MODE_TEXT          0x00
#define TMS_R0_EXT_VDP_ENABLE     0x01
#define TMS_R0_EXT_VDP_DISABLE    0x00

#define TMS_R1_RAM_16K            0x80
#define TMS_R1_RAM_4K             0x00
#define TMS_R1_DISP_BLANK         0x00
#define TMS_R1_DISP_ACTIVE        0x40
#define TMS_R1_INT_ENABLE         0x20
#define TMS_R1_INT_DISABLE        0x00
#define TMS_R1_MODE_GRAPHICS_I    0x00
#define TMS_R1_MODE_GRAPHICS_II   0x00
#define TMS_R1_MODE_MULTICOLOR    0x08
#define TMS_R1_MODE_TEXT          0x10
#define TMS_R1_SPRITE_8           0x00
#define TMS_R1_SPRITE_16          0x02
#define TMS_R1_SPRITE_MAG1        0x00
#define TMS_R1_SPRITE_MAG2        0x01
#define LAST_SPRITE_YPOS	        0xC0		// dice 0xBE  https://www.unige.ch/medecine/nouspikel/ti99/tms9918a.htm#Sprites

#define TMS_DEFAULT_VRAM_NAME_ADDRESS          0x3800		// qua boh, forse 0
#define TMS_DEFAULT_VRAM_COLOR_ADDRESS         0x0400
#define TMS_DEFAULT_VRAM_PATT_ADDRESS          0x0800
#define TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS   0x0500
#define TMS_DEFAULT_VRAM_SPRITE_PATT_ADDRESS   0x1000
extern BYTE TMSVideoRAM[];
struct __attribute__((__packed__)) SPRITE_ATTR {
  uint8_t ypos,xpos;    // v. sotto, a volte usato come signed
  uint8_t name;
  union __attribute__((__packed__)) {
    struct __attribute__((__packed__)) {
      unsigned int color:4;
      unsigned int unused:3;
      unsigned int eclock:1;
      };
    uint8_t tag;
    };
  };

enum {
  DoReset=1,
  DoIRQ=2,
//  DoWait=8,     // 
  DoIdle=4,
	DoLOAD=8
	};

//https://en.wikipedia.org/wiki/TMS9900
#define WORKING_REG_INDEX (Pipe1 & 0xf)
#define GET_REG(q) (MAKEWORD(regs->r[q].b.h,regs->r[q].b.l))      // 
#define GET_WORKING_REG_S() GET_REG(workingRegIndex)      // 
#define SET_REG(q,n) {regs->r[q].b.l=HIBYTE(n);regs->r[q].b.h=LOBYTE(n);}      // 
#define SET_WORKING_REG_S(n) SET_REG(workingRegIndex,n);
#define WORKING_TS ((Pipe1 >> 4) & 0b11)
#define WORKING_TD ((Pipe1 >> 10) & 0b11)
#define REGISTER_DIRECT 0
#define REGISTER_INDIRECT 1
#define REGISTER_SYMBOLIC_INDEXED 2
#define REGISTER_INDIRECT_AUTOINCREMENT 3
#define WORKING_REG2_INDEX ((Pipe1 >> 6) & 0xf)
#define GET_WORKING_REG_D() GET_REG(workingReg2Index)      // 
#define SET_WORKING_REG_D(n) SET_REG(workingReg2Index,n);

// USARE, MA OCCHIO!!  ci sono le piccole differenze con getPipe e pc+=2, e anche autoincrement
// mancano solo + alcune 8bit
#define COMPUTE_SOURCE_NOPIPE(a) \
  switch(workingTS) {\
    case REGISTER_DIRECT:\
      res##a.x=GET_WORKING_REG_S();\
      break;\
    case REGISTER_INDIRECT:\
      res##a.x=GetIntValue(GET_WORKING_REG_S());\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingRegIndex)\
        res##a.x=GetIntValue((uint16_t)(GET_WORKING_REG_S()+(int16_t)Pipe2.x));\
      else\
        res##a.x=GetIntValue(Pipe2.x);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res##a.x=GetIntValue(GET_WORKING_REG_S());\
      SET_WORKING_REG_S(GET_WORKING_REG_S()+2);\
      break;\
    }
#define COMPUTE_SOURCE_PIPE(a) \
  switch(workingTS) {\
    case REGISTER_DIRECT:\
      res##a.x=GET_WORKING_REG_S();\
      break;\
    case REGISTER_INDIRECT:\
      res##a.x=GetIntValue(GET_WORKING_REG_S());\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingRegIndex)\
        res##a.x=GetIntValue((uint16_t)(GET_WORKING_REG_S()+(int16_t)Pipe2.x));\
      else\
        res##a.x=GetIntValue(Pipe2.x);\
      GetPipe(_pc);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res##a.x=GetIntValue(GET_WORKING_REG_S());\
      SET_WORKING_REG_S(GET_WORKING_REG_S()+2);\
      break;\
    }
#define COMPUTE_SOURCE2 \
  switch(workingTD) {\
    case REGISTER_DIRECT:\
      res2.x=GET_WORKING_REG_D();\
      break;\
    case REGISTER_INDIRECT:\
      res2.x=GetIntValue(GET_WORKING_REG_D());\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingReg2Index)\
        res2.x=GetIntValue((uint16_t)(GET_WORKING_REG_D()+(int16_t)Pipe2.x));\
      else\
        res2.x=GetIntValue(Pipe2.x);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res2.x=GetIntValue(GET_WORKING_REG_D());\
      SET_WORKING_REG_D(GET_WORKING_REG_D()+2);\
      break;\
    }
#define COMPUTE_SOURCE2_NOINC(a) \
  switch(workingTD) {\
    case REGISTER_DIRECT:\
      res##a.x=GET_WORKING_REG_D();\
      break;\
    case REGISTER_INDIRECT:\
      res##a.x=GetIntValue(GET_WORKING_REG_D());\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingReg2Index)\
        res##a.x=GetIntValue((uint16_t)(GET_WORKING_REG_D()+(int16_t)Pipe2.x));\
      else\
        res##a.x=GetIntValue(Pipe2.x);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res##a.x=GetIntValue(GET_WORKING_REG_D());\
      break;\
    }
#define COMPUTE_SOURCE_NOPC_NOINC(a) \
  switch(workingTS) {\
    case REGISTER_DIRECT:\
      res##a.x=GET_WORKING_REG_S();\
      break;\
    case REGISTER_INDIRECT:\
      res##a.x=GetIntValue(GET_WORKING_REG_S());\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingRegIndex)\
        res##a.x=GetIntValue((uint16_t)(GET_WORKING_REG_S()+(int16_t)Pipe2.x));\
      else\
        res##a.x=GetIntValue(Pipe2.x);\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res##a.x=GetIntValue(GET_WORKING_REG_S());\
      break;\
    }
#define COMPUTE_SOURCE2_NOPC_NOINC(a) \
  switch(workingTD) {\
    case REGISTER_DIRECT:\
      res##a.x=GET_WORKING_REG_D();\
      break;\
    case REGISTER_INDIRECT:\
      res##a.x=GetIntValue(GET_WORKING_REG_D());\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingReg2Index)\
        res##a.x=GetIntValue((uint16_t)(GET_WORKING_REG_D()+(int16_t)Pipe2.x));\
      else\
        res##a.x=GetIntValue(Pipe2.x);\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res##a.x=GetIntValue(GET_WORKING_REG_D());\
      break;\
    }
#define COMPUTE_SOURCE8_PIPE(a) \
  switch(workingTS) {\
    case REGISTER_DIRECT:\
      res##a.b.l=HIBYTE(GET_WORKING_REG_S());\
      break;\
    case REGISTER_INDIRECT:\
      res##a.b.l=GetValue(GET_WORKING_REG_S());\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingRegIndex)\
        res##a.b.l=GetValue((uint16_t)(GET_WORKING_REG_S()+(int16_t)Pipe2.x));\
      else\
        res##a.b.l=GetValue(Pipe2.x);\
      GetPipe(_pc);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res##a.b.l=GetValue(GET_WORKING_REG_S());\
      SET_WORKING_REG_S(GET_WORKING_REG_S()+1);\
      break;\
    }
#define COMPUTE_SOURCE82(a) \
  switch(workingTD) {\
    case REGISTER_DIRECT:\
      res##a.b.l=HIBYTE(GET_WORKING_REG_D());\
      break;\
    case REGISTER_INDIRECT:\
      res##a.b.l=GetValue(GET_WORKING_REG_D());\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingReg2Index)\
        res##a.b.l=GetValue(GET_WORKING_REG_D()+(int16_t)Pipe2.x);\
      else\
        res##a.b.l=GetValue(Pipe2.x);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res##a.b.l=GetValue(GET_WORKING_REG_D());\
      SET_WORKING_REG_D(GET_WORKING_REG_D()+1);\
      break;\
    }
#define COMPUTE_SOURCE82_NOPC_NOINC(a) \
  switch(workingTD) {\
    case REGISTER_DIRECT:\
      res##a.b.l=HIBYTE(GET_WORKING_REG_D());\
      break;\
    case REGISTER_INDIRECT:\
      res##a.b.l=GetValue(GET_WORKING_REG_D());\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingReg2Index)\
        res##a.b.l=GetValue(GET_WORKING_REG_D()+(int16_t)Pipe2.x);\
      else\
        res##a.b.l=GetValue(Pipe2.x);\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res##a.b.l=GetValue(GET_WORKING_REG_D());\
      break;\
    }
#define COMPUTE_SOURCE_BRANCH \
  switch(workingTS) {\
    case REGISTER_DIRECT:\
      res3.x=GET_WORKING_REG_S();		/* forse non c'è...*/\
      break;\
    case REGISTER_INDIRECT:\
/* BOH no... tipo B  *R5                  res3.x=GetIntValue(WORKING_REG);*/\
      res3.x=GET_WORKING_REG_S();\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingRegIndex)\
        res3.x=GET_WORKING_REG_S()+(int16_t)Pipe2.x;\
      else\
        res3.x=Pipe2.x;\
      _pc+=2;/*non servirebbe in B ma ok...*/\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      res3.x=GetIntValue(GET_WORKING_REG_S());/* BOH...*/\
      SET_WORKING_REG_S(GET_WORKING_REG_S()+2);\
      break;\
    }

#define STORE_DEST_16 \
  switch(workingTD) {\
    case REGISTER_DIRECT:\
      SET_WORKING_REG_D(res3.x);\
      break;\
    case REGISTER_INDIRECT:\
      PutIntValue(GET_WORKING_REG_D(),res3.x);\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingReg2Index)\
        PutIntValue((uint16_t)(GET_WORKING_REG_D()+(int16_t)Pipe2.x),res3.x);\
      else\
        PutIntValue(Pipe2.x,res3.x);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      PutIntValue(GET_WORKING_REG_D(),res3.x);\
      SET_WORKING_REG_D(GET_WORKING_REG_D()+2);\
      break;\
    }
#define STORE_DEST_8 \
  switch(workingTD) {\
    case REGISTER_DIRECT:\
      SET_WORKING_REG_D(MAKEWORD(LOBYTE(GET_WORKING_REG_D()),res3.b.l));	/* se registro, va in MSB*/ \
      break;\
    case REGISTER_INDIRECT:\
      PutValue(GET_WORKING_REG_D(),res3.b.l);	/* v. di là, big-endian circa*/ \
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingReg2Index)\
        PutValue((uint16_t)(GET_WORKING_REG_D()+(int16_t)Pipe2.x),res3.b.l);\
      else\
        PutValue(Pipe2.x,res3.b.l);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      PutValue(GET_WORKING_REG_D(),res3.b.l);\
      SET_WORKING_REG_D(GET_WORKING_REG_D()+1);\
      break;\
    }

#define STORE_SOURCE_16 \
  switch(workingTS) {\
    case REGISTER_DIRECT:\
      SET_WORKING_REG_S(res3.x);\
      break;\
    case REGISTER_INDIRECT:\
      PutIntValue(GET_WORKING_REG_S(),res3.x);\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingRegIndex)\
        PutIntValue((uint16_t)(GET_WORKING_REG_S()+(int16_t)Pipe2.x),res3.x);\
      else\
        PutIntValue(Pipe2.x,res3.x);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      PutIntValue(GET_WORKING_REG_S(),res3.x);\
      SET_WORKING_REG_S(GET_WORKING_REG_S()+2);\
      break;\
    }
#define STORE_SOURCE_8 \
  switch(workingTS) {\
    case REGISTER_DIRECT:\
      SET_WORKING_REG_S(res3.x);\
      break;\
    case REGISTER_INDIRECT:\
      PutIntValue(GET_WORKING_REG_S(),res3.x);\
      break;\
    case REGISTER_SYMBOLIC_INDEXED:\
      if(workingRegIndex)\
        PutIntValue((uint16_t)(GET_WORKING_REG_S()+(int16_t)Pipe2.x),res3.x);\
      else\
        PutIntValue(Pipe2.x,res3.x);\
      _pc+=2;\
      break;\
    case REGISTER_INDIRECT_AUTOINCREMENT:\
      PutIntValue(GET_WORKING_REG_S(),res3.x);\
      SET_WORKING_REG_S(GET_WORKING_REG_S()+2);\
      break;\
    }



void mySYSTEMConfigPerformance(void);
void myINTEnableSystemMultiVectoredInt(void);

#define ReadCoreTimer()                  _CP0_GET_COUNT()           // Read the MIPS Core Timer

void ShortDelay(DWORD DelayCount);
#define __delay_ms(n) ShortDelay(n*100000UL)
#define __delay_ns(n) ShortDelay(n*100UL)
void DelayUs(unsigned int);
void DelayMs(unsigned int);

#define ClrWdt() { WDTCONbits.WDTCLRKEY=0x5743; }

// PIC32 RTCC Structure
typedef union {
  struct {
    unsigned char   weekday;    // BCD codification for day of the week, 00-06
    unsigned char   mday;       // BCD codification for day of the month, 01-31
    unsigned char   mon;        // BCD codification for month, 01-12
    unsigned char   year;       // BCD codification for years, 00-99
  	};                              // field access	
  unsigned char       b[4];       // byte access
  unsigned short      w[2];       // 16 bits access
  unsigned long       l;          // 32 bits access
	} PIC32_RTCC_DATE;

// PIC32 RTCC Structure
typedef union {
  struct {
    unsigned char   reserved;   // reserved for future use. should be 0
    unsigned char   sec;        // BCD codification for seconds, 00-59
    unsigned char   min;        // BCD codification for minutes, 00-59
    unsigned char   hour;       // BCD codification for hours, 00-24
  	};                              // field access
  unsigned char       b[4];       // byte access
  unsigned short      w[2];       // 16 bits access
  unsigned long       l;          // 32 bits access
	} PIC32_RTCC_TIME;
extern volatile PIC32_RTCC_DATE currentDate;
extern volatile PIC32_RTCC_TIME currentTime;



void Timer_Init(void);
void PWM_Init(void);
void UART_Init(DWORD);
void putsUART1(unsigned int *buffer);

int decodeKBD(int, long, BOOL);
WORD GetPipe(SWORD);
BYTE GetValue(SWORD);
SWORD GetValueCRU(SWORD,BYTE);
SWORD GetIntValue(SWORD);
void PutValue(SWORD,BYTE);
void PutIntValue(SWORD,SWORD);
void PutValueCRU(SWORD,SWORD,BYTE);
int Emulate(int);

int UpdateScreen(SWORD rowIni, SWORD rowFin);


#ifdef ST7735

#define LED1 LATEbits.LATE2
#define LED2 LATEbits.LATE3
#define LED3 LATEbits.LATE4
#define SW1  PORTDbits.RD2
#define SW2  PORTDbits.RD3


// pcb SDRradio 2019
#define	SPISDITris 0		// niente qua
#define	SPISDOTris TRISGbits.TRISG8				// SDO
#define	SPISCKTris TRISGbits.TRISG6				// SCK
#define	SPICSTris  TRISGbits.TRISG7				// CS
#define	LCDDCTris  TRISEbits.TRISE7				// DC che su questo LCD è "A0" per motivi ignoti
//#define	LCDRSTTris TRISBbits.TRISB7
	
#define	m_SPISCKBit LATGbits.LATG6		// pin 
#define	m_SPISDOBit LATGbits.LATG8		// pin 
#define	m_SPISDIBit 0
#define	m_SPICSBit  LATGbits.LATG7		// pin 
#define	m_LCDDCBit  LATEbits.LATE7 		// pin 
//#define	m_LCDRSTBit LATBbits.LATB7 //FARE
//#define	m_LCDBLBit  LATBbits.LATB12
#endif

#ifdef ILI9341

#define LED1 LATEbits.LATE4
#define LED2 LATDbits.LATD0
#define LED3 LATDbits.LATD11
#define SW2  PORTFbits.RF0
#define SW1  PORTBbits.RB0          // bah uso AREF tanto per...

#define	LCDDCTris  TRISBbits.TRISB3				// http://attach01.oss-us-west-1.aliyuncs.com/IC/Datasheet/11009.zip?spm=a2g0o.detail.1000023.9.70352ae94rI9S1&file=11009.zip
#define	LCDRSTTris TRISBbits.TRISB10

#define	LCDRDTris  TRISBbits.TRISB5          // 
#define	LCDWRTris  TRISBbits.TRISB4          // WR per LCD parallelo
#define	LCDSTRTris  TRISBbits.TRISB4         // Strobe per LCD parallelo A3_TRIS (in pratica Write...)

#define	LCDCSTris  TRISBbits.TRISB2

#define	m_LCDDCBit  LATBbits.LATB3 		// 
#define	m_LCDRSTBit LATBbits.LATB10
//#define	m_LCDBLBit  LATBbits.LATB12

#define	m_LCDRDBit  LATBbits.LATB5 		// 
#define	m_LCDWRBit  LATBbits.LATB4 		// per LCD parallelo ILI
#define	m_LCDSTRBit LATBbits.LATB4        // non è chiaro... m_A3_out; in pratica è WRITE

#define	m_LCDCSBit  LATBbits.LATB2

//Buzzer RB1
#endif

#endif

