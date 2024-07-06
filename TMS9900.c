// https://github.com/mamedev/mame/blob/master/src/devices/cpu/tms9900/tms9900.cpp

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>

#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#include "adafruit_gfx.h"

#include "TMS9900_PIC.h"




extern BYTE fExit;
extern BYTE debug;
volatile BYTE TIMIRQ,VIDIRQ;

extern volatile BYTE keysFeedPtr;

BYTE DoReset=0,DoIRQ=0,DoLOAD=0,DoIdle=0;
#define MAX_WATCHDOG 100      // x30mS v. sotto
WORD WDCnt=MAX_WATCHDOG;
BYTE ColdReset=1;

//#define TMS9940 1

uint16_t Pipe1;
union __attribute__((__packed__)) {
	uint16_t x;
	uint8_t bb[4];
	struct __attribute__((__packed__)) {
		uint8_t l;
		uint8_t h;
//		BYTE u;		 bah no, sposto la pipe quando ci sono le istruzioni lunghe 4...
		} b;
	} Pipe2;

union __attribute__((__packed__)) Z_REG {
  uint16_t x;
  struct __attribute__((__packed__)) { 
    uint8_t l;
    uint8_t h;
    } b;
  };
union __attribute__((__packed__)) Z_REGISTERS {
  uint8_t b[32];
  union Z_REG r[16];
  };

#define ID_LG 0x1
#define ID_AG 0x2
#define ID_ZERO 0x4
#define ID_CARRY 0x8
#define ID_OVF 0x10
#define ID_PARITY 0x20
#define ID_XOP 0x40
#define ID_INTERRUPTMASK 0xF000
union __attribute__((__packed__)) REGISTRO_F {
  uint16_t x;
  struct __attribute__((__packed__)) {
    unsigned int LogicalGreater: 1;
    unsigned int ArithmeticGreater: 1;
    unsigned int Zero: 1;
    unsigned int Carry: 1;
    unsigned int Overflow: 1;
    unsigned int Parity: 1;   // 0=pari 1=dispari (ODD PARITY)
    unsigned int XOP: 1;
    unsigned int unused: 1;
    unsigned int unused2: 4;
    unsigned int InterruptMask: 4;
    };
  };
union __attribute__((__packed__)) OPERAND {
  uint8_t *reg8;
  uint16_t *reg16;
  uint16_t mem;
  };
union __attribute__((__packed__)) RESULT {
  struct __attribute__((__packed__)) {
    uint8_t l;
    uint8_t h;
    } b;
  uint16_t x;
  uint32_t d;
  };
    
int Emulate(int mode) {
//https://en.wikipedia.org/wiki/TMS9900
#define WORKING_REG regs1->r[(Pipe1 & 0xf)].x      // 
#define WORKING_TS ((Pipe1 >> 4) & 0b11)
#define WORKING_TD ((Pipe1 >> 12) & 0b11)
#define REGISTER_DIRECT 0
#define REGISTER_INDIRECT 1
#define REGISTER_SYMBOLIC_INDEXED 2
#define REGISTER_INDIRECT_AUTOINCREMENT 3
#define WORKING_REG2 regs1->r[((Pipe1 >> 8) & 0xf)].x      // 
    
	SWORD _pc=0;
	SWORD _wp=0;
	BYTE IPL=0;
  union Z_REGISTERS *regs1;
  union RESULT res1,res2,res3;
//  union OPERAND op1,op2;
	union REGISTRO_F _st;
	/*register*/ uint16_t i;
  int c=0;


  _pc=GetIntValue(0x0002);
  _wp=GetIntValue(0x0000);
  IPL=0b0001;   // Ti99
  
  
//  _pc=0x0935;
//  _sp=0x8700;
  

	do {

		c++;
		if(!(c & 0x3ffff)) {
      ClrWdt();
// yield()
#ifndef USING_SIMULATOR      
			UpdateScreen(0,192);    // fare passate più piccole!
#endif
extern BYTE TMS9918Reg[8],TMS9918RegS;
      TMS9918RegS |= 0b10000000;
      if(TMS9918Reg[1] & 0b00100000) {
        VIDIRQ=1;
        }
      
      LED1^=1;    // 42mS~ con SKYNET 7/6/20; 10~mS con Z80NE 10/7/21; 35mS GALAKSIJA 16/10/22; 30mS ZX80 27/10/22
      // QUADRUPLICO/ecc! 27/10/22
      
      
      }

		if(ColdReset) {
      DoReset=1;
			continue;
      }


    if(TIMIRQ) {
//      DoIRQ=1;
      TIMIRQ=0;
      }
    if(VIDIRQ) {
//      DoIRQ=1;
      VIDIRQ=0;
      }

    
		/*
		if((_pc >= 0xa000) && (_pc <= 0xbfff)) {
			printf("%04x    %02x\n",_pc,GetValue(_pc));
			}
			*/
		if(debug) {
//			printf("%04x    %02x\n",_pc,GetValue(_pc));
			}
		/*if(kbhit()) {
			getch();
			printf("%04x    %02x\n",_pc,GetValue(_pc));
			printf("281-284: %02x %02x %02x %02x\n",*(p1+0x281),*(p1+0x282),*(p1+0x283),*(p1+0x284));
			printf("2b-2c: %02x %02x\n",*(p1+0x2b),*(p1+0x2c));
			printf("33-34: %02x %02x\n",*(p1+0x33),*(p1+0x34));
			printf("37-38: %02x %02x\n",*(p1+0x37),*(p1+0x38));
			}*/
		if(DoReset) {
			_pc=GetIntValue(0x0002);
			_wp=GetIntValue(0x0000);
			IPL=0b0001;   // Ti99
			DoReset=0;DoIdle=0;
      keysFeedPtr=255; //meglio ;)
      initHW();
      continue;
			}
		if(DoLOAD) {
			DoLOAD=0; DoIdle=0;
//?? serve			IPL=0b1111;
			_pc=GetIntValue(0xfffe);
			_wp=GetIntValue(0xfffc);

      }
		if(DoIRQ) {   // https://www.unige.ch/medecine/nouspikel/ti99/ints.htm
      
      // LED2^=1;    // 
      DoIdle=0;     // 
      
			if(IPL <= _st.InterruptMask) {
//??				IPL = _st.InterruptMask;
				DoIRQ=0;
  			_pc=GetIntValue(0x0002+IPL*2);
    		_wp=GetIntValue(0x0000+IPL*2);
        
				}
			}

  
		if(DoIdle) {
      //mettere ritardino per analogia con le istruzioni?
//      __delay_ns(500); non va più nulla... boh...
			continue;		// esegue cmq IRQ e refresh
      }

//printf("Pipe1: %02x, Pipe2w: %04x, Pipe2b1: %02x,%02x\n",Pipe1,Pipe2.word,Pipe2.bytes.byte1,Pipe2.bytes.byte2);
    
    
      if(!SW2) {        // test tastiera, me ne frego del repeat/rientro :)
       // continue;
        __delay_ms(100); ClrWdt();
        DoReset=1;
        }
      if(!SW1) {        // test tastiera
        if(keysFeedPtr==255)      // debounce...
          keysFeedPtr=254;
        }

      LED2^=1;    // ~700nS 7/6/20, ~600 con 32bit 10/7/21 MA NON FUNZIONA/visualizza!! verificare; 5-700nS 27/10/22

    
/*      if(_pc == 0x069d ab5 43c Cd3) {
        ClrWdt();
        }*/
extern BYTE ram_seg[];
    regs1=(union Z_REGISTERS *)&ram_seg[_wp & 0xff /* -RAM_START */];     // così oppure cast diretto...
  
		GetPipe(_pc);
    _pc += 2;
execute:
		switch(Pipe1 & 0b1111000000000000) {
      case 0b0000 << 12:
    		if(Pipe1 & 0b0000100000000000) {    // SLA SRA SRC SRL
          if(!(Pipe1 & 0b0000000011110000)) { // count
            res2.b.l=regs1->r[0].x >> 12;
            if(!res2.b.l)
              res2.b.l=16;
            }
          else {
            res2.b.l=(Pipe1 & 0b0000000011110000) >> 4;
            }
          switch(WORKING_TS) {
            case REGISTER_DIRECT:
              res1.x=WORKING_REG;
              break;
            case REGISTER_INDIRECT:
              res1.x=GetIntValue(WORKING_REG);
              break;
            case REGISTER_SYMBOLIC_INDEXED:
              res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
              break;
            case REGISTER_INDIRECT_AUTOINCREMENT:
              res1.x=GetIntValue(WORKING_REG);
              WORKING_REG+=2;
              break;
            }
        
          switch(Pipe1 & 0b1111111100000000) {
            case 0b00001010 << 8:     // SLA Shift left arithmetic
              while(res2.b.l) {
                _st.Carry= res1.x & 0x8000 ? 1 : 0;
                res1.x <<= 1;
                res3.x=res1.x;
                }
              
aggRotate:
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  WORKING_REG=res3.x;
                  break;
                case REGISTER_INDIRECT:
                  PutIntValue(WORKING_REG,res3.x);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  PutIntValue(WORKING_REG+(int16_t)Pipe2.x,res3.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  PutIntValue(WORKING_REG,res3.x);
                  WORKING_REG+=2;
                  break;
                }
              break;
            case 0b00001000 << 8:     // SRA Shift right arithmetic
              while(res2.b.l) {
                _st.Carry=res1.x & 0x1;
                res1.x >>= 1;
                if(res1.x & 0x4000)
                  res1.x |= 0x8000;
                res3.x=res1.x;
                }
              goto aggRotate;
              break;
            case 0b00001011 << 8:     // SRC Shift right circular
              while(res2.b.l) {
                _st.Carry=res3.x & 1;
                res3.x >>= 1;
                if(_st.Carry)
                  res1.x |= 0x8000;
                res3.x=res1.x;
                }
              goto aggRotate;
              break;
            case 0b00001001 << 8:     // SRL Shift right logical
              while(res2.b.l) {
                _st.Carry=res1.x & 0x1;
                res1.x >>= 1;
                res3.x=res1.x;
                }
              goto aggRotate;
              break;
            }
          }
        else {
          switch(Pipe1 & 0b1111111111000000) {
            case 0b0000001000 << 6:     // AI ANDI CI LI ORI
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000010001 << 5:     // AI Add immediate
                  res1.x=WORKING_REG;
                  res2.x=Pipe2.x;
                  res3.x=(uint32_t)res1.x+(uint32_t)res2.x;
                  
                  WORKING_REG=res3.x;

									_pc+=2;
                  
aggFlag34A:
                  _st.Carry=!!HIWORD(res3.d);
                  _st.Overflow = !!(((res1.x & 0x8000) == (res2.x & 0x8000)) && ((res3.x & 0x8000) != (res2.x & 0x8000)));
									goto aggFlag012;
                  break;
                case 0b00000010010 << 5:     // ANDI AND immediate
                  res1.x=WORKING_REG;
                  res2.x=Pipe2.x;
                  res3.x=res1.x & res2.x;
									_pc+=2;

aggFlag012:
                  _st.LogicalGreater=!!(res3.x != 0);
                  _st.ArithmeticGreater=!!(res3.x != 0 && !(res3.x & 0x8000));
                  _st.Zero=res3.x ? 0 : 1;
                  break;
                case 0b00000010100 << 5:     // CI Compare immediate
                  res1.x=WORKING_REG;
                  res2.x=Pipe2.x;
                  res3.x=(uint32_t)res1.x-(uint32_t)res2.x;
									_pc+=2;
        
compare:        
                  _st.LogicalGreater=!!(((res1.x & 0x8000) && !(res2.x & 0x8000))
                    || (((res1.x & 0x8000) == (res2.x & 0x8000)) && !(res3.x & 0x8000)));
                  _st.ArithmeticGreater=!!((!(res1.x & 0x8000) && (res2.x & 0x8000))
                    || (((res1.x & 0x8000) == (res2.x & 0x8000)) && !(res3.x & 0x8000)));
                  _st.Zero=res3.x ? 0 : 1;
                  break;
                case 0b00000010000 << 5:     // LI Load immediate
                  WORKING_REG=Pipe2.x;
                  res3.x=WORKING_REG;
									_pc+=2;
                  goto aggFlag012;
                  break;
                case 0b00000010011 << 5:     // ORI OR immediate
                  res1.x=WORKING_REG;
                  res2.x=Pipe2.x;
                  res3.x=res1.x | res2.x;
									_pc+=2;
                  goto aggFlag012;
                  break;
                }
              break;
              
            case 0b0000010001 << 6:     // B Branch
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res3.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res3.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res3.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res3.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              _pc=res3.x;
              break;
            case 0b0000011010 << 6:     // BL Branch and Link
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res3.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res3.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res3.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res3.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              regs1->r[14].x=_pc;
              _pc=res3.x;
              break;
            case 0b0000010000 << 6:     // BLWP Branch and Load Workspace Pointer
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res3.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res3.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res3.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res3.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
            	_pc=GetIntValue(0x0002+res3.x);
          		_wp=GetIntValue(0x0000+res3.x);
              regs1->r[13].x=_wp;
              regs1->r[14].x=_pc;
              regs1->r[15].x=_st.x;
              _wp=res3.x;
              
              _pc=WORKING_REG +2;
              break;
            case 0b0000010011 << 6:     // CLR Clear Operand
              res3.x=0;
              
store16_2:              
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  WORKING_REG=res3.x;
                  break;
                case REGISTER_INDIRECT:
                  PutIntValue(WORKING_REG,res3.x);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  PutIntValue(WORKING_REG+(int16_t)Pipe2.x,res3.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  PutIntValue(WORKING_REG,res3.x);
                  WORKING_REG2+=2;
                  break;
                }
            	goto aggFlag012;
              break;
            case 0b0000011100 << 6:     // SETO Set To Ones
              res3.x=0xffff;
              goto store16_2;
              break;
            case 0b0000010101 << 6:     // INV Invert
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res1.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res1.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res1.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              res3.x=~res1.x;
              goto store16_2;
              
              
              break;
            case 0b0000010100 << 6:     // NEG Negate
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res2.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res2.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res2.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res2.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              res1.x=0;
              res3.x=res1.x-res2.x;
              goto store16_2;
              
              break;
            case 0b0000011101 << 6:     // ABS Absolute Value
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res1.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res1.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res1.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              res3.x=abs(res3.x);
              goto store16_2;
              break;
            case 0b0000011011 << 6:     // SWPB Swap Bytes
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res1.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res1.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res1.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              res3.x=MAKEWORD(HIBYTE(res1.x),LOBYTE(res1.x));
              goto store16_2;
              break;
            case 0b0000010110 << 6:     // INC Increment
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res1.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res1.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res1.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              res3.x=res1.x;
              res3.x++;
              
aggInc:
              _st.Overflow= !!(!(res1.x & 0x8000) && (res3.x & 0x8000));
              _st.Carry=!!HIWORD(res1.x);
              goto store16_2;
              break;
            case 0b0000010111 << 6:     // INCT Increment by Two
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res1.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res1.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res1.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              res3.x=res1.x;
              res3.x+=2;
              goto aggInc;
              break;
            case 0b0000011000 << 6:     // DEC Decrement
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res1.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res1.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res1.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              res3.x=res1.x;
              res3.x--;
aggDec:
              _st.Overflow= !!((res1.x & 0x8000) && !(res3.x & 0x8000));
              _st.Carry=!!HIWORD(res1.x);
              goto store16_2;
              break;
            case 0b0000011001 << 6:     // DECT Decrement by Two
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res1.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res1.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res1.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              res3.x=res1.x;
              res3.x-=2;
              goto aggDec;
              break;
            case 0b0000010010 << 6:     // X Execute
              switch(WORKING_TS) {
                case REGISTER_DIRECT:
                  res3.x=WORKING_REG;
                  break;
                case REGISTER_INDIRECT:
                  res3.x=GetIntValue(WORKING_REG);
                  break;
                case REGISTER_SYMBOLIC_INDEXED:
                  res3.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
                  _pc+=2;
                  break;
                case REGISTER_INDIRECT_AUTOINCREMENT:
                  res3.x=GetIntValue(WORKING_REG);
                  WORKING_REG+=2;
                  break;
                }
              Pipe1=res3.x;
              goto execute;
              break;
              
            case 0b0000001011 << 6:     // LWPI LIMI
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000010111 << 5:     // LWPI Load workspace pointer immediate
                  _wp=Pipe2.x;
                  _pc+=2;
                  break;
                  
                case 0b00000010110 << 5:     // STST Store status register
                  WORKING_REG=_st.x;
                  break;
                }
              break;
            case 0b0000001100 << 6:     // LWPI LIMI
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000011000 << 5:     // LIMI Load interrupt mask
                  _st.x=(_st.x & 0b0000111111111111) | (Pipe2.x & 0b1111000000000000);
                  _pc+=2;
                  break;
                }
              break;
            case 0b0000001010 << 6:     // STWP
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000010101 << 5:     // STWP Store workspace pointer
                  WORKING_REG=_wp;
                  break;
                }
              break;
              
            case 0b0000001110 << 6:     // RTWP
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000011100 << 5:     // RTWP Return workspace pointer
                  _st.x=regs1->r[15].x;
                  _pc=regs1->r[14].x;
                  _wp=regs1->r[13].x;
                  break;
                }
              break;
              
            case 0b0000001101 << 6:     // IDLE
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000011010 << 5:     // IDLE
          			  DoIdle=1;
                  break;
                case 0b00000011011 << 5:     // RSET
                  _st.x &= 0b0000111111111111;
                  break;
                }
              break;

            case 0b0000001111 << 6:     // CKOF CKON LREX
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000011110 << 5:     // CKOF
                  break;
                case 0b00000011101 << 5:     // CKON
                  break;
                case 0b00000011111 << 5:     // LREX
                  break;
                }
              break;
            }
          }
        break;
      
      case 0b0001 << 12:
    		switch(Pipe1 & 0b1111111100000000) {
          case 0b00010011 << 8:     // JEQ Jump equal
            if(_st.Zero)
              goto Jump;
            break;
          case 0b00010101 << 8:     // JGT Jump greater than
            if(_st.ArithmeticGreater)
              goto Jump;
            break;
          case 0b00011011 << 8:     // JH Jump high
            if(_st.LogicalGreater && !_st.Zero)
              goto Jump;
            break;
          case 0b00010100 << 8:     // JHE Jump high or equal
            if(_st.LogicalGreater || _st.Zero)
              goto Jump;
            break;
          case 0b00011010 << 8:     // JL Jump low
            if(!_st.LogicalGreater && !_st.Zero)
              goto Jump;
            break;
          case 0b00010010 << 8:     // JLE Jump low or equal
            if(!_st.LogicalGreater || _st.Zero)
              goto Jump;
            break;
          case 0b00010001 << 8:     // JLT Jump less than
            if(!_st.ArithmeticGreater && !_st.Zero)
              goto Jump;
            break;
          case 0b00010000 << 8:     // JMP Jump unconditional
Jump:
    				_pc += (int8_t)LOBYTE(Pipe1) *2;
            break;
          case 0b00010111 << 8:     // JNC Jump no carry
            if(!_st.Carry)
              goto Jump;
            break;
          case 0b00010110 << 8:     // JNE Jump not equal
            if(!_st.Zero)
              goto Jump;
            break;
          case 0b00011001 << 8:     // JNO Jump no overflow
            if(!_st.Overflow)
              goto Jump;
            break;
          case 0b00011000 << 8:     // JOC Jump carry
            if(_st.Carry)
              goto Jump;
            break;
          case 0b00011100 << 8:     // JOP Jump odd parity
            if(_st.Parity)
              goto Jump;
            break;
          }
        break;
        
      case 0b1010 << 12:    // A Add
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.x=WORKING_REG;
            break;
          case REGISTER_INDIRECT:
            res1.x=GetIntValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.x=GetIntValue(WORKING_REG);
            WORKING_REG+=2;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.x=WORKING_REG2;
            break;
          case REGISTER_INDIRECT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.x=GetIntValue(WORKING_REG2+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          }
        res3.d=(uint32_t)res1.x+(uint32_t)res2.x;
        
store16:
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            WORKING_REG2=res3.x;
            break;
          case REGISTER_INDIRECT:
            PutIntValue(WORKING_REG2,res3.x);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            PutIntValue(WORKING_REG2+(int16_t)Pipe2.x,res3.x);
            _pc+=2;
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            PutIntValue(WORKING_REG2,res3.x);
            WORKING_REG2+=2;
            break;
          }
        goto aggFlag34A;
        break;
      case 0b1011 << 12:    // AB Add bytes
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res1.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.b.l=GetValue(WORKING_REG+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.b.l=GetValue(WORKING_REG);
            WORKING_REG++;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res2.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.b.l=GetValue(WORKING_REG2+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res2.b.l=GetValue(WORKING_REG2);
            WORKING_REG2++;
            break;
          }
        res3.x=(uint16_t)res1.b.l+(uint16_t)res2.b.l;
        
//        _st.Overflow = !!(((res1.b.l & 0x40) + (res2.b.l & 0x40)) & 0x80) != !!(((res1.x & 0x80) + (res2.x & 0x80)) & 0x100);
        _st.Overflow = !!(((res1.b.h & 0x80) == (res2.b.h & 0x80)) && ((res3.b.h & 0x80) != (res2.b.h & 0x80)));
          
store8:
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            WORKING_REG2=res3.x;
            break;
          case REGISTER_INDIRECT:
            PutValue(WORKING_REG2,res3.b.l);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            PutValue(WORKING_REG2+(int16_t)Pipe2.x,res3.b.l);
            _pc+=2;
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            PutValue(WORKING_REG2,res3.b.l);
            WORKING_REG2+=2;
            break;
          }
        
        _st.Carry=!!res3.b.h;
        _st.LogicalGreater=!!(res3.b.l != 0);
        _st.ArithmeticGreater=!!(res3.b.l != 0 && !(res3.b.l & 0x8000));
        _st.Zero=res3.b.l ? 0 : 1;

calcParity:
        {
        BYTE par;
        par= res3.b.l >> 1;			// Microchip AN774
        par ^= res3.b.l;
        res3.b.l= par >> 2;
        par ^= res3.b.l;
        res3.b.l= par >> 4;
        par ^= res3.b.l;
        _st.Parity=par & 1 ? 0 : 1;   // ODD
        }
        break;

      case 0b1000 << 12:    // C Compare
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.x=WORKING_REG;
            break;
          case REGISTER_INDIRECT:
            res1.x=GetIntValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.x=GetIntValue(WORKING_REG);
            WORKING_REG+=2;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.x=WORKING_REG2;
            break;
          case REGISTER_INDIRECT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.x=GetIntValue(WORKING_REG2+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          }
        res3.d=(uint32_t)res1.x-(uint32_t)res2.x;
        
        goto compare;
        break;
      case 0b1001 << 12:    // CB Compare bytes
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res1.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.b.l=GetValue(WORKING_REG+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.b.l=GetValue(WORKING_REG);
            WORKING_REG++;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res2.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.b.l=GetValue(WORKING_REG2+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res2.b.l=GetValue(WORKING_REG2);
            WORKING_REG2++;
            break;
          }
        res3.x=(uint16_t)res1.b.l-(uint16_t)res2.b.l;
        
        _st.LogicalGreater=!!(((res1.b.l & 0x80) && !(res2.b.l & 0x80))
          || (((res1.b.l & 0x80) == (res2.b.l & 0x80)) && !(res3.b.l & 0x80)));
        _st.ArithmeticGreater=!!((!(res1.b.l & 0x80) && (res2.b.l & 0x80))
          || (((res1.b.l & 0x80) == (res2.b.l & 0x80)) && !(res3.b.l & 0x80)));
        _st.Zero=res3.b.l ? 0 : 1;
        break;

      case 0b0110 << 12:    // S Subtract
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.x=WORKING_REG;
            break;
          case REGISTER_INDIRECT:
            res1.x=GetIntValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.x=GetIntValue(WORKING_REG);
            WORKING_REG+=2;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.x=WORKING_REG2;
            break;
          case REGISTER_INDIRECT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.x=GetIntValue(WORKING_REG2+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          }
        res3.d=(uint32_t)res1.x-(uint32_t)res2.x;
        
aggFlag34S:
        _st.Carry=!!HIWORD(res3.d);
//        _st.Overflow = !!(((res1.x & 0x4000) + (res2.x & 0x4000)) & 0x8000) != !!(((res1.d & 0x8000) + (res2.d & 0x8000)) & 0x10000);
        _st.Overflow = !!(((res1.x & 0x8000) != (res2.x & 0x8000)) && ((res3.x & 0x8000) != (res2.x & 0x8000)));
        goto store16;
        break;
      case 0b0111 << 12:    // SB Subtract bytes
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res1.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.b.l=GetValue(WORKING_REG+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.b.l=GetValue(WORKING_REG);
            WORKING_REG++;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res2.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.b.l=GetValue(WORKING_REG2+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.b.l=GetValue(WORKING_REG2);
            WORKING_REG2++;
            break;
          }
        res3.x=(uint16_t)res1.b.l-(uint16_t)res2.b.l;
        _st.Overflow = !!(((res1.b.h & 0x80) != (res2.b.h & 0x80)) && ((res3.b.h & 0x80) != (res2.b.h & 0x80)));
        goto store8;
        break;
      
      case 0b1110 << 12:    // SOC Set ones corresponding
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.x=WORKING_REG;
            break;
          case REGISTER_INDIRECT:
            res1.x=GetIntValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.x=GetIntValue(WORKING_REG);
            WORKING_REG+=2;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.x=WORKING_REG2;
            break;
          case REGISTER_INDIRECT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.x=GetIntValue(WORKING_REG2+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          }
        res3.x=res2.x | res1.x;
        
store16_012:
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            WORKING_REG2=res3.x;
            break;
          case REGISTER_INDIRECT:
            PutIntValue(WORKING_REG2,res3.x);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            PutIntValue(WORKING_REG2+(int16_t)Pipe2.x,res3.x);
            _pc+=2;
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            PutIntValue(WORKING_REG2,res3.x);
            WORKING_REG2+=2;
            break;
          }
        break;
      case 0b1111 << 12:    // SOCB Set ones corresponding bytes  
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res1.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.b.l=GetValue(WORKING_REG+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.b.l=GetValue(WORKING_REG);
            WORKING_REG++;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res2.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.b.l=GetValue(WORKING_REG2+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res2.b.l=GetValue(WORKING_REG2);
            WORKING_REG2++;
            break;
          }
        res3.b.l=res2.b.l | res1.b.l;
        
        goto store8;
        break;
      
      case 0b0100 << 12:    // SZC Set zeros corresponding
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.x=WORKING_REG;
            break;
          case REGISTER_INDIRECT:
            res1.x=GetIntValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.x=GetIntValue(WORKING_REG);
            WORKING_REG+=2;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.x=WORKING_REG2;
            break;
          case REGISTER_INDIRECT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.x=GetIntValue(WORKING_REG2+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res2.x=GetIntValue(WORKING_REG2);
            break;
          }
        res3.x=res2.x & ~res1.x;
        
        goto store16_012;
        break;
      case 0b0101 << 12:    // SZCB Set zeros corresponding bytes  
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res1.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.b.l=GetValue(WORKING_REG+(int16_t)Pipe2.x);
            _pc+=2;
            GetPipe(_pc);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.b.l=GetValue(WORKING_REG);
            WORKING_REG++;
            break;
          }
        switch(WORKING_TD) {
          case REGISTER_DIRECT:
            res2.b.l=LOBYTE(WORKING_REG);
            break;
          case REGISTER_INDIRECT:
            res2.b.l=GetValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res2.b.l=GetValue(WORKING_REG2+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res2.b.l=GetValue(WORKING_REG2);
            WORKING_REG2++;
            break;
          }
        res3.b.l=res2.b.l & ~res1.b.l;
        
        goto store8;
        break;
      
      case 0b1100 << 12:    // MOV Move
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.x=WORKING_REG;
            break;
          case REGISTER_INDIRECT:
            res1.x=GetIntValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.x=GetIntValue(WORKING_REG);
            WORKING_REG+=2;
            break;
          }
        res3.x=res1.x;
        
        goto store16_012;
        break;
      case 0b1101 << 12:    // MOVB Move bytes  
        res3.b.l=res1.b.l;
        
        goto store8;
        break;
      
      
      case 0b0010 << 12:    // Compare Ones, Compare Zeros, Exclusive OR
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.x=WORKING_REG;
            break;
          case REGISTER_INDIRECT:
            res1.x=GetIntValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
            _pc+=2;
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.x=GetIntValue(WORKING_REG);
            WORKING_REG+=2;
            break;
          }
    		switch(Pipe1 & 0b1111110000000000) {
          case 0b001000 << 10:     // COC Compare Ones corresponding
            if((res1.x & WORKING_REG) == res1.x)
              _st.Zero=1;
            break;
          case 0b001001 << 10:     // CZC Compare Zeros corresponding
            if((~res1.x & ~WORKING_REG) == ~res1.x)
              _st.Zero=1;
            break;
          case 0b001010 << 10:     // XOR Exclusive OR
            res3.x = res1.x ^ res2.x;
            WORKING_REG2=res3.x;
            goto aggFlag012;
            break;
          case 0b001011 << 10:     // XOP Extended Operation
#ifdef TMS9940 
            
#else
            _st.XOP=1;
            regs1->r[(Pipe1 & 0b1111000000) >> 4].x;      // D
           	_pc=GetIntValue(0x0042+res1.x*4);
         		_wp=GetIntValue(0x0040+res1.x*4);
            break;
#endif
          }
        break;
      
      case 0b0011 << 12:    // Multiply, Divide
        switch(WORKING_TS) {
          case REGISTER_DIRECT:
            res1.x=WORKING_REG;
            break;
          case REGISTER_INDIRECT:
            res1.x=GetIntValue(WORKING_REG);
            break;
          case REGISTER_SYMBOLIC_INDEXED:
            res1.x=GetIntValue(WORKING_REG+(int16_t)Pipe2.x);
            _pc+=2;
            break;
          case REGISTER_INDIRECT_AUTOINCREMENT:
            res1.x=GetIntValue(WORKING_REG);
            WORKING_REG+=2;
            break;
          }
    		switch(Pipe1 & 0b1111110000000000) {
          case 0b001110 << 10:     // MPY Multiply
            res3.d = res1.x * res2.x;
            WORKING_REG2=HIWORD(res3.d);
            regs1->r[(((Pipe1 >> 8) +1) & 0xf)].x=res3.x;   // OKKIO, porcata, & se 15...
//no!            goto aggFlag;
            break;
          case 0b001111 << 10:     // DIV Divide
            res2.d = MAKELONG(WORKING_REG2,regs1->r[(((Pipe1 >> 8) +1) & 0xf)].x);    // OKKIO...
            if(!res1.x) {
              //DIVIDE ZERO??
              }
    /*    da0 = (divident >> 16);     // https://hackaday.io/project/20826-tms9900-compatible-cpu-core-in-vhdl/log/67326-success-fpga-based-ti-994a-working
    da1 = divident & 0xFFFF;
    sa = divisor;
    
    int st4;
    if( (((sa & 0x8000) == 0 && (da0 & 0x8000) == 0x8000))
      || ((sa & 0x8000) == (da0 & 0x8000) && (((da0 - sa) & 0x8000) == 0)) ) {
      st4 = 1;
      } 
    else {
      st4 = 0;
      // actual division loop, here sa is known to be larger than da0.
      for(int i=0; i<16; i++) {
          da0 = (da0 << 1) | ((da1 >> 15) & 1);
          da1 <<= 1;
          if(da0 >= sa) {
              da0 -= sa;
              da1 |= 1;   // successful substraction
          }
      }
      }*/
            if( (((res1.x & 0x8000) == 0 && (res2.x & 0x8000) == 0x8000))
              || ((res1.x & 0x8000) == (res2.x & 0x8000) && (((res2.x - res1.x) & 0x8000) == 0)) )
              _st.Overflow = 1;
            else {
              res3.d = res2.d / (uint32_t)res1.x;
              WORKING_REG2=LOWORD(res3.d);
              regs1->r[(((Pipe1 >> 8) +1) & 0xf)].x=res2.d % (uint32_t)res1.x;   // OKKIO, porcata, & se 15...
              _st.Overflow = 0;
              }
            break;
            
          case 0b001100 << 10:     // LDCR Load communication register
            break;
          case 0b001101 << 10:     // STCR Store communication register
            break;
            
          case 0b000111 << 10:     // SBO SBZ TB (CRU operations)
        		switch(Pipe1 & 0b1111111100000000) {    // https://www.unige.ch/medecine/nouspikel/ti99/cru.htm
              case 0b00111101 << 8:     // SBO Set bit to one
                break;
              case 0b00111110 << 8:     // SBZ Set bit to zero
                break;
              case 0b00111111 << 8:     // TB Test bit 
                break;
              }
            break;
          }
        break;

        
			
			}
		} while(!fExit);
	}


