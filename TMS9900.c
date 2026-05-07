// https://github.com/mamedev/mame/blob/master/src/devices/cpu/tms9900/tms9900.cpp

//NB  in byte access, the CPU outputs the byte on the lower as well as the upper eight data lines.
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>

#ifdef ST7735
#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#include "adafruit_gfx.h"
#endif
#ifdef ILI9341
#include "Adafruit_ILI9341.h"
#include "adafruit_gfx.h"
#endif

#include "TMS9900_PIC.h"



BYTE fExit;
uint16_t Pipe1;
union __attribute__((__packed__)) PIPE Pipe2;
BYTE debug;
extern SWORD VICRaster;
extern BYTE ram_seg[];
extern volatile BYTE *keysFeedPtr;
extern const char keysFeed[];
volatile BYTE TIMIRQ,VIDIRQ;


#define MAX_WATCHDOG 100      // x30mS v. sotto
WORD WDCnt=MAX_WATCHDOG;
BYTE ColdReset=1;
BYTE CPUPins=DoReset;

WORD CPUClock=2000000L/CPU_CLOCK_DIVIDER,HWClock=1000000L/HW_CLOCK_DIVIDER;



// da 68000 Makushi o come cazzo si chiama :D
// res2 è Source e res1 è Dest ossia quindi res3=Result
//#define CARRY_ADD_8() (!!(((res2.b.l & res1.b.l) | (~res3.b.l & (res2.b.l | res1.b.l))) & 0x80))		// ((S & D) | (~R & (S | D)))
// V. DEC/INC! 
#define CARRY_ADD_8() (!!(res3.b.l < res2.b.l))
#define OVF_ADD_8()  (!!(((res2.b.l ^ res3.b.l) & (res1.b.l ^ res3.b.l)) & 0x80))			// ((S^R) & (D^R))
//#define CARRY_ADD_16() (!!(((res2.x & res1.x) | (~res3.x &	(res2.x | res1.x))) & 0x8000))
#define CARRY_ADD_16() (!!(res3.x < res2.x))
#define OVF_ADD_16() (!!(((res2.x ^ res3.x) & (res1.x ^ res3.x)) & 0x8000))
#define CARRY_SUB_8() (!!(((res2.b.l & res3.b.l) | (~res1.b.l & (res2.b.l | res3.b.l))) & 0x80))		// ((S & R) | (~D & (S | R)))
//#define CARRY_SUB_8() (!!((res3.b.l<res2.b.l) || (res2.b.l==0)))
#define OVF_SUB_8()  (!!(((res2.b.l ^ res1.b.l) & (res3.b.l ^ res1.b.l)) & 0x80))			// ((S^D) & (R^D))
#define CARRY_SUB_16() (!!(((res2.x & res3.x) | (~res1.x &	(res2.x | res3.x))) & 0x8000))
//#define CARRY_SUB_16() (!!((res3.x<res2.x) || (res2.x==0)))
#define OVF_SUB_16() (!!(((res2.x ^ res1.x) & (res3.x ^ res1.x)) & 0x8000))

extern BYTE TMS9918Reg[],TMS9918RegS;
extern BYTE TMS9901[];
extern WORD TMS9901Timer,TMS9901Cnt;
 

int Emulate(int mode) {
	SWORD _pc=0;
	SWORD _wp=0;
	BYTE IPL=0;
  union T_REGISTERS *regs=NULL;
  union RESULT res1,res2,res3;
//  union OPERAND op1,op2;
	union REGISTRO_F _st;
	/*register*/ uint16_t i;
  uint8_t workingTS,workingTD,workingRegIndex,workingReg2Index;
  int c=0;

	DWORD cyclesPerSec,cyclesSoFar,cyclesCPU,cyclesHW;
	BYTE screenDivider;

	cyclesPerSec=10000000L;		// AT
	cyclesCPU=0; cyclesHW=0;
	cyclesSoFar=0;

  _pc=GetIntValue(0x0002);
  _wp=GetIntValue(0x0000);
  _st.x=0;
  IPL=0b0001;   // Ti99
  
  

	do {

		cyclesSoFar++;

		c++;
		if(!(c & 0x3fff)) {
      ClrWdt();
// yield()
        VICRaster+=8;					 	 // raster pos count, 200 al sec...
        if(VICRaster >= MAX_RASTER) {		 // 
          VICRaster=MIN_RASTER;
//          LED2 ^= 1;      // 50Hz 8/11/19; 70mS su ILI 320x240, 7/8/20; 25mS PIC32MM 17/6/21
          }

        
#ifdef ILI9341
//        static BYTE divider;
//        divider++;
//        if(!(divider & 1))
#endif
#ifndef USING_SIMULATOR      
//			UpdateScreen(0,192);    // fare passate più piccole!
          UpdateScreen(VICRaster,VICRaster+8);
#endif
  //      LED3 ^= 1;
        
extern BYTE TMS9918Reg[8],TMS9918RegS;
      TMS9918RegS |= 0b10000000;
      if(TMS9918Reg[1] & 0b00100000) {
        VIDIRQ=1;
        }
      
      LED1^=1;    // 11mS~ 7/5/26
     
      
      }

		if(ColdReset) {
			ColdReset=0;
//			initHW();
      CPUPins |= DoReset;
			continue;
      }


    if(TIMIRQ) {		// v. TMS9901, gestire...
      CPUPins |= DoIRQ;
			IPL=0b0001;   // 
      TIMIRQ=0;
      }
    if(VIDIRQ) {
        static BYTE oldSW2;
  
        
//#ifndef USING_SIMULATOR
        if(!SW1) {        // test tastiera, me ne frego del repeat/rientro :)
          if(keysFeedPtr==255)      // debounce...
            keysFeedPtr=254;
          }
        
        if(!SW2) {
          if(oldSW2) {
//            CPUPins |= DoNMI;    // solo sul fronte! o si blocca/sovraccarica
            oldSW2=0;
            }
          }
        else
          oldSW2=1;
//#endif
      
      CPUPins |= DoIRQ;
			IPL=0b0001;   // 
      VIDIRQ=0;
      }

    
		if(CPUPins & DoReset) {
			_pc=GetIntValue(0x0002);
			_wp=GetIntValue(0x0000);
      _st.x=0;
			IPL=0b0001;   // Ti99
			CPUPins &= ~(DoReset | DoIdle);
      initHW();
      continue;
			}
		if(CPUPins & DoLOAD) {
			CPUPins &= ~(DoLOAD | DoIdle);
//?? serve			IPL=0b1111);
			_pc=GetIntValue(0xfffe);
			_wp=GetIntValue(0xfffc);

      }
		if(CPUPins & DoIRQ) {   // https://www.unige.ch/medecine/nouspikel/ti99/ints.htm
      
      // LED2^=1;    // 
			CPUPins &= ~(DoIdle);
      
			if(IPL <= _st.InterruptMask) {		// TMS9900 will perform BLWP @>0000 through BLWP @>003C depending on the interrupt level. 
//??				IPL = _st.InterruptMask;
				CPUPins &= ~DoIRQ;
        i=_wp;
    		_wp=GetIntValue((uint16_t)(0x0000+IPL*4));
		    regs=(union T_REGISTERS *)&ram_seg[_wp & 0xff /* -RAM_START */];     // così oppure cast diretto...
        SET_REG(14,_pc);		// VERIFICARE!
  			_pc=GetIntValue((uint16_t)(0x0002+IPL*4));
        SET_REG(13,i);
        SET_REG(15,_st.x);

				}
			}

  
		if(CPUPins & DoIdle) {
      //mettere ritardino per analogia con le istruzioni?
//      __delay_ns(500); non va più nulla... boh...
			continue;		// esegue cmq IRQ 
      }

//printf("Pipe1: %02x, Pipe2w: %04x, Pipe2b1: %02x,%02x\n",Pipe1,Pipe2.word,Pipe2.bytes.byte1,Pipe2.bytes.byte2);
    
    
      if(!SW2) {        // test tastiera, me ne frego del repeat/rientro :)
       // continue;
        __delay_ms(100); ClrWdt();
        CPUPins |= DoReset;
        }

      LED2^=1;    // ~400nS (alcune 1000)  7/5/26
		if(cyclesSoFar<cyclesCPU)		//
			goto rallenta;
		cyclesCPU += CPUClock;
    
/*      if(_pc == 0x069d ab5 43c Cd3) {
        ClrWdt();
        }*/
extern BYTE ram_seg[];
    regs=(union Z_REGISTERS*)&ram_seg[_wp & 0xff /* -RAM_START */];     // così oppure cast diretto...
  
		GetPipe(_pc);
    _pc += 2;
execute:
    workingTS=WORKING_TS; workingTD=WORKING_TD;   // mettere solo dove serve??
    workingRegIndex=WORKING_REG_INDEX; workingReg2Index=WORKING_REG2_INDEX;
		switch(Pipe1 & 0b1111000000000000) {
      case 0b0000 << 12:
    		if(Pipe1 & 0b0000100000000000) {    // SLA SRA SRC SRL
          if(!(Pipe1 & 0b0000000011110000))  // count
            res2.b.l=GET_REG(0) & 0xf;
          else
            res2.b.l=(uint8_t)(Pipe1 & 0b0000000011110000) >> 4;
          if(!res2.b.l)
            res2.b.l=16;
          res1.x=GET_WORKING_REG_S();
        
          switch(Pipe1 & 0b1111111100000000) {
            case 0b00001010 << 8:     // SLA Shift left arithmetic
              res3.x=res1.x;
              while(res2.b.l--) {
                _st.Carry= res1.x & 0x8000 ? 1 : 0;
                res1.x <<= 1;
                if((res1.x & 0x8000) != (res3.x & 0x8000))
									_st.Overflow=1;
                res3.x=res1.x;
                }
              
aggRotate:
              SET_WORKING_REG_S(res3.x);
              goto aggFlag16Z;
              break;
            case 0b00001000 << 8:     // SRA Shift right arithmetic
              while(res2.b.l--) {
                _st.Carry=res1.x & 0x1;
                res1.x >>= 1;
                if(res1.x & 0x4000)
                  res1.x |= 0x8000;
                res3.x=res1.x;
                }
              goto aggRotate;
              break;
            case 0b00001011 << 8:     // SRC Shift right circular
              res3.x=res1.x;
              while(res2.b.l--) {
                _st.Carry=res3.x & 1;
                res1.x >>= 1;
                if(_st.Carry)
                  res1.x |= 0x8000;
                res3.x=res1.x;
                }
              goto aggRotate;
              break;
            case 0b00001001 << 8:     // SRL Shift right logical
              while(res2.b.l--) {
                _st.Carry=res1.x & 0x1;
                res1.x >>= 1;
                res3.x=res1.x;
                }
              goto aggRotate;
              break;
            }
          }		// SLA ecc
        else {
          switch(Pipe1 & 0b1111111111000000) {
            case 0b0000001000 << 6:     // AI LI 
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000010001 << 5:     // AI Add immediate
                  res2.x=GET_WORKING_REG_S();
                  res1.x=Pipe2.x;
                  res3.x=res1.x+res2.x;
                  SET_WORKING_REG_S(res3.x);
									_pc+=2;
                  
aggFlag16A:
                  _st.Carry=CARRY_ADD_16();
                  _st.Overflow = OVF_ADD_16();
									goto aggFlag16Z;
                  break;
                case 0b00000010000 << 5:     // LI Load immediate
                  res3.x=Pipe2.x;
									SET_WORKING_REG_S(res3.x);
									_pc+=2;
                  goto aggFlag16Z;
                  break;
                }
              break;
              
            case 0b0000001001 << 6:     // ANDI ORI
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000010010 << 5:     // ANDI AND immediate
                  res1.x=GET_WORKING_REG_S();
                  res2.x=Pipe2.x;
                  res3.x=res1.x & res2.x;
                  SET_WORKING_REG_S(res3.x);
									_pc+=2;

aggFlag16Z:
                  _st.LogicalGreater=res3.x>0 ? 1 : 0;
                  _st.ArithmeticGreater=((int16_t)res3.x)>((int16_t)0) ? 1 : 0;
                  _st.Zero=res3.x ? 0 : 1;
                  break;
                case 0b00000010011 << 5:     // ORI OR immediate
                  res1.x=GET_WORKING_REG_S();
                  res2.x=Pipe2.x;
                  res3.x=res1.x | res2.x;
                  SET_WORKING_REG_S(res3.x);
									_pc+=2;
                  goto aggFlag16Z;
                  break;
                }
              break;
              
            case 0b0000010001 << 6:     // B Branch
							COMPUTE_SOURCE_BRANCH
              _pc=res3.x;
              break;
            case 0b0000011010 << 6:     // BL Branch and Link
							COMPUTE_SOURCE_BRANCH
              SET_REG(11,_pc);
              _pc=res3.x;
              break;
            case 0b0000010000 << 6:     // BLWP Branch and Load Workspace Pointer
							COMPUTE_SOURCE_BRANCH
              i=_wp;
          		_wp=GetIntValue((uint16_t)(0x0000+res3.x));
						  regs=(union T_REGISTERS *)&ram_seg[_wp & 0xff /* -RAM_START */];     // così oppure cast diretto...
              SET_REG(14,_pc);
            	_pc=GetIntValue((uint16_t)(0x0002+res3.x));
              SET_REG(13,i);
              SET_REG(15,_st.x);
              // saltare interrupt dopo di questa, dice...
              break;
            case 0b0000010011 << 6:     // CLR Clear Operand
              res3.x=0;
              
store16_S_noF:
							STORE_SOURCE_16
              break;
            case 0b0000011100 << 6:     // SETO Set To Ones
              res3.x=0xffff;
              goto store16_S_noF;
              break;
            case 0b0000010101 << 6:     // INV Invert
							COMPUTE_SOURCE_NOPIPE(1);
              res3.x=~res1.x;

store16_S:
							STORE_SOURCE_16
            	goto aggFlag16Z;
              break;
            case 0b0000010100 << 6:     // NEG Negate
							COMPUTE_SOURCE_NOPC_NOINC(2);
              res1.x=0;
              res3.x=res1.x-res2.x;
							_st.Carry=CARRY_SUB_16();
			        _st.Overflow = OVF_SUB_16();
              goto store16_S;
              break;
            case 0b0000011101 << 6:     // ABS Absolute Value
							COMPUTE_SOURCE_NOPC_NOINC(2);
              res3.x=abs(res2.x);
							STORE_SOURCE_16
              res3.x=res2.x;							//flag van controllati PRIMA!!! 
            	goto aggFlag16Z;
              break;
            case 0b0000011011 << 6:     // SWPB Swap Bytes
							COMPUTE_SOURCE_NOPC_NOINC(2);
              res3.x=MAKEWORD(HIBYTE(res2.x),LOBYTE(res2.x));
              goto store16_S_noF;
              break;
            case 0b0000010110 << 6:     // INC Increment
							COMPUTE_SOURCE_NOPC_NOINC(1);
              res3.x=res1.x+1;
//              _st.Carry= res3.x & 0x10 ? 1 : 0;		// v. ti99sim
              _st.Carry= res3.x < 1 ? 1 : 0;		// v. classic99
//					  _st.Overflow= (x3==0x8000) ? 1 : 0;
              _st.Overflow= !!(!(res1.x & 0x8000) && (res3.x & 0x8000));
              goto store16_S;
              break;
            case 0b0000010111 << 6:     // INCT Increment by Two
							COMPUTE_SOURCE_NOPC_NOINC(1);
              res3.x=res1.x+2;
              _st.Carry= res3.x < 2 ? 1 : 0;		// v. classic99
//					  _st.Overflow= ((x3==0x8000)||(x3==0x8001)) ? 1 : 0;
              _st.Overflow= !!(!(res1.x & 0x8000) && (res3.x & 0x8000));
              goto store16_S;
              break;
            case 0b0000011000 << 6:     // DEC Decrement
							COMPUTE_SOURCE_NOPC_NOINC(1);
              res3.x=res1.x-1;
//              _st.Carry= res3.x & 0x10 ? 0 : 1;		// v. ti99sim
              _st.Carry= res3.x != 0xffff ? 1 : 0;		// v. classic99
              _st.Overflow= !!((res1.x & 0x8000) && !(res3.x & 0x8000));
              goto store16_S;
              break;
            case 0b0000011001 << 6:     // DECT Decrement by Two
							COMPUTE_SOURCE_NOPC_NOINC(1);
              res3.x=res1.x-2;
//              _st.Carry= res3.x & 0x10 ? 0 : 1;		// v. ti99sim
              _st.Carry= res3.x < 0xfffe ? 1 : 0;		// v. classic99
              _st.Overflow= !!((res1.x & 0x8000) && !(res3.x & 0x8000));
              goto store16_S;
              break;
            case 0b0000010010 << 6:     // X Execute
							COMPUTE_SOURCE_NOPIPE(1);
              Pipe1=res1.x;
              goto execute;
              break;
              
            case 0b0000001011 << 6:     // LWPI LIMI
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000010111 << 5:     // LWPI Load workspace pointer immediate
                  _wp=Pipe2.x;
                  _pc+=2;
                  break;
                  
                case 0b00000010110 << 5:     // STST Store status register
                  SET_WORKING_REG_S(_st.x);
                  break;
                }
              break;
            case 0b0000001100 << 6:     // LWPI LIMI
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000011000 << 5:     // LIMI Load interrupt mask
                  _st.InterruptMask=Pipe2.x & 0b00001111;
                  _pc+=2;
                  break;
                }
              break;
            case 0b0000001010 << 6:     // STWP
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000010101 << 5:     // STWP Store workspace pointer
                  SET_WORKING_REG_S(_wp);
                  break;
                case 0b00000010100 << 5:     // CI Compare immediate
                  res1.x=GET_WORKING_REG_S();
                  res2.x=Pipe2.x;
									_pc+=2;
        
compare16:
                  _st.LogicalGreater=res1.x>res2.x ? 1 : 0;
                  _st.ArithmeticGreater=((int16_t)res1.x)>((int16_t)res2.x) ? 1 : 0;
                  _st.Zero=res1.x==res2.x ? 1 : 0;
                  break;
                }
              break;
              
            case 0b0000001110 << 6:     // RTWP
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000011100 << 5:     // RTWP Return workspace pointer
                  _st.x=GET_REG(15);
                  _pc=GET_REG(14);
                  _wp=GET_REG(13);
                  break;
                }
              break;
              
            case 0b0000001101 << 6:     // IDLE
              switch(Pipe1 & 0b1111111111100000) {
                case 0b00000011010 << 5:     // IDLE
          			  CPUPins |= DoIdle;
                  break;
                case 0b00000011011 << 5:     // RSET
                  _st.InterruptMask = 0;
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
    		switch(Pipe1 & 0b111100000000) {
          case 0b1011 << 8:     // JH Jump high
            if(_st.LogicalGreater && !_st.Zero)
              goto Jump;
            break;
          case 0b1010 << 8:     // JL Jump low
            if(!_st.LogicalGreater && !_st.Zero)
              goto Jump;
            break;
          case 0b0100 << 8:     // JHE Jump high or equal
            if(_st.LogicalGreater || _st.Zero)
              goto Jump;
            break;
          case 0b0010 << 8:     // JLE Jump low or equal
            if(!_st.LogicalGreater || _st.Zero)
              goto Jump;
            break;
          case 0b0101 << 8:     // JGT Jump greater than
            if(_st.ArithmeticGreater)
              goto Jump;
            break;
          case 0b0001 << 8:     // JLT Jump less than
            if(!_st.ArithmeticGreater && !_st.Zero)
              goto Jump;
            break;
          case 0b0011 << 8:     // JEQ Jump equal
            if(_st.Zero)
              goto Jump;
            break;
          case 0b0110 << 8:     // JNE Jump not equal
            if(!_st.Zero)
              goto Jump;
            break;
          case 0b1000 << 8:     // JOC Jump carry
            if(_st.Carry)
              goto Jump;
            break;
          case 0b0111 << 8:     // JNC Jump no carry
            if(!_st.Carry)
              goto Jump;
            break;
          case 0b1001 << 8:     // JNO Jump no overflow
            if(!_st.Overflow)
              goto Jump;
            break;
          case 0b1100 << 8:     // JOP Jump odd parity
            if(_st.Parity)
              goto Jump;
            break;
          case 0b0000 << 8:     // JMP Jump unconditional  (se 0x1000 vale come NOP !
Jump:
    				_pc += (int8_t)LOBYTE(Pipe1) *2;
            break;

               // SBO SBZ TB (CRU operations)
          case 0b1101 << 8:     // SBO Set bit to one
		        res3.x=GetValueCRU((uint16_t)(GET_REG(12)+LOBYTE(Pipe1)/8),1);
		        PutValueCRU((uint16_t)(GET_REG(12)+LOBYTE(Pipe1)/8),res3.x,1);
            break;
          case 0b1110 << 8:     // SBZ Set bit to zero
		        res3.x=GetValueCRU((uint16_t)(GET_REG(12)+LOBYTE(Pipe1)/8),1);
		        PutValueCRU((uint16_t)(GET_REG(12)+LOBYTE(Pipe1)/8),res3.x,1);
            break;
          case 0b1111 << 8:     // TB Test bit 
		        res3.x=GetValueCRU((uint16_t)(GET_REG(12)+LOBYTE(Pipe1)/8),1);
						if(res3.x)
							_st.Zero=1;			// occhio invertito
						else
							_st.Zero=0;
            break;
          }
        break;
        
      case 0b1010 << 12:    // A Add
				COMPUTE_SOURCE_PIPE(2); 
				COMPUTE_SOURCE2_NOPC_NOINC(1);
        res3.x=res1.x+res2.x;
				STORE_DEST_16
        goto aggFlag16A;
        break;
      case 0b1011 << 12:    // AB Add bytes
				COMPUTE_SOURCE8_PIPE(2);
				COMPUTE_SOURCE82_NOPC_NOINC(1)
        res3.b.l=res1.b.l+res2.b.l;
        
//        _st.Overflow = !!(((res1.b.l & 0x40) + (res2.b.l & 0x40)) & 0x80) != !!(((res1.x & 0x80) + (res2.x & 0x80)) & 0x100);
//        _st.Overflow = !!(((res1.b.h & 0x80) == (res2.b.h & 0x80)) && ((res3.b.h & 0x80) != (res2.b.h & 0x80)));
        _st.Overflow = OVF_ADD_8();
        _st.Carry=CARRY_ADD_8();

store8_D:
				STORE_DEST_8
      
aggFlag8Z:
        _st.LogicalGreater=res3.b.l>0 ? 1 : 0;
        _st.ArithmeticGreater=((int8_t)res3.b.l)>((int8_t)0) ? 1 : 0;
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
        _st.Parity=par & 1 ? 1 : 0;   // ODD
        }
        break;

      case 0b1000 << 12:    // C Compare
				COMPUTE_SOURCE_PIPE(1);
				COMPUTE_SOURCE2
        goto compare16;
        break;
      case 0b1001 << 12:    // CB Compare bytes
				COMPUTE_SOURCE8_PIPE(1);
				COMPUTE_SOURCE82(2)
				res3.b.l=res1.b.l;
        _st.LogicalGreater=res1.b.l>res2.b.l ? 1 : 0;
        _st.ArithmeticGreater=((int8_t)res1.b.l)>((int8_t)res2.b.l) ? 1 : 0;
        _st.Zero=res1.b.l == res2.b.l ? 1 : 0;
				goto calcParity;
        break;

      case 0b0110 << 12:    // S Subtract
				COMPUTE_SOURCE_PIPE(2); 
				COMPUTE_SOURCE2_NOPC_NOINC(1);
        res3.x=res1.x-res2.x;
        
        _st.Carry=CARRY_SUB_16();
//        _st.Overflow = !!(((res1.x & 0x4000) + (res2.x & 0x4000)) & 0x8000) != !!(((res1.d & 0x8000) + (res2.d & 0x8000)) & 0x10000);
//        _st.Overflow = !!(((res1.x & 0x8000) != (res2.x & 0x8000)) && ((res3.x & 0x8000) != (res2.x & 0x8000)));
        _st.Overflow = OVF_SUB_16();
				STORE_DEST_16
        goto aggFlag16Z;
        break;
      case 0b0111 << 12:    // SB Subtract bytes  OPERANDI INVERTITI ;) anche in Add, mentre Compare è dritta!
				COMPUTE_SOURCE8_PIPE(2);
				COMPUTE_SOURCE82_NOPC_NOINC(1)
        res3.b.l=res1.b.l-res2.b.l;
        _st.Carry=CARRY_SUB_8();
//        _st.Overflow = !!(((res1.b.h & 0x80) != (res2.b.h & 0x80)) && ((res3.b.h & 0x80) != (res2.b.h & 0x80)));
        _st.Overflow = OVF_SUB_8();
        goto store8_D;
        break;
      
      case 0b1110 << 12:    // SOC Set ones corresponding
				COMPUTE_SOURCE_PIPE(1); 
				COMPUTE_SOURCE2_NOINC(2);
        res3.x=res2.x | res1.x;
        
store16_D:
				STORE_DEST_16
        goto aggFlag16Z;
        break;
      case 0b1111 << 12:    // SOCB Set ones corresponding bytes  
				COMPUTE_SOURCE8_PIPE(1);
				COMPUTE_SOURCE82_NOPC_NOINC(2)
        res3.b.l=res2.b.l | res1.b.l;
        goto store8_D;
        break;
      
      case 0b0100 << 12:    // SZC Set zeros corresponding
				COMPUTE_SOURCE_PIPE(1);
				COMPUTE_SOURCE2_NOINC(2);
        res3.x=res2.x & ~res1.x;
        goto store16_D;
        break;
      case 0b0101 << 12:    // SZCB Set zeros corresponding byte
				COMPUTE_SOURCE8_PIPE(1);
				COMPUTE_SOURCE82_NOPC_NOINC(2)
        res3.b.l=res2.b.l & ~res1.b.l;
        goto store8_D;
        break;
      
      case 0b1100 << 12:    // MOV Move
				COMPUTE_SOURCE_PIPE(1);
        res3.x=res1.x;
        goto store16_D;
        break;
      case 0b1101 << 12:    // MOVB Move bytes  
				COMPUTE_SOURCE8_PIPE(1);
        res3.b.l=res1.b.l;
        goto store8_D;
        break;
      
      case 0b0010 << 12:    // Compare Ones, Compare Zeros, Exclusive OR
				COMPUTE_SOURCE_NOPIPE(1);
    		switch(Pipe1 & 0b1111110000000000) {
          case 0b001000 << 10:     // COC Compare Ones corresponding
            if((res1.x & GET_WORKING_REG_D()) == res1.x)
              _st.Zero=1;
						else
              _st.Zero=0;
            break;
          case 0b001001 << 10:     // CZC Compare Zeros corresponding
            if((res1.x & GET_WORKING_REG_D()) == 0 /*res1.x*/)
              _st.Zero=1;
						else
              _st.Zero=0;
            break;
          case 0b001010 << 10:     // XOR Exclusive OR
            res2.x=GET_WORKING_REG_D();
            res3.x = res1.x ^ res2.x;
            SET_WORKING_REG_D(res3.x);
            goto aggFlag16Z;
            break;
          case 0b001011 << 10:     // XOP Extended Operation
#ifdef TMS9940 
        		switch(Pipe1 & 0b0000001111000000) {
              case 0b0000) << 6:   // DCA      verificare!!
                res3.b.l=res1.b.l;
                i=_st.Carry;
                _st.Carry=0;
                if((res1.b.l & 0xf) > 9 || _st.DigitCarry) {
                  res3.x+=6;
                  res1.b.l=res3.b.l;
                  _st.Carry= i || res3.b.h;
                  _st.DigitCarry=1;
                  }
                else
                  _st.DigitCarry=0;
                if((res1.b.l>0x99) || i) {
                  res3.b.l+=0x60;  
                  _st.Carry=1;
                  }
                else
                  _st.Carry=0;
store_dca:
								STORE_DEST_8
								_st.LogicalGreater=res3.b.l>0 ? 1 : 0;
								_st.ArithmeticGreater=((int8_t)res3.b.l)>((int8_t)0) ? 1 : 0;
                _st.Zero=res3.b.l ? 0 : 1;
                goto calcParity;
                break;
              case 0b0001 << 6:   // DCS      verificare!! finire
                res3.b.l=res1.b.l;
                i=_st.Carry;
                _st.Carry=0;
                if((res1.b.l & 0xf) > 9 || _st.DigitCarry) {
                  res3.x+=6;
                  res1.b.l=res3.b.l;
                  _st.Carry= i || res3.b.h;
                  _st.DigitCarry=1;
                  }
                else
                  _st.DigitCarry=0;
                if((res1.b.l>0x99) || i) {
                  res3.b.l+=0x60;  
                  _st.Carry=1;
                  }
                else
                  _st.Carry=0;
                goto store_dca;
                break;
              case 0b0010) << 6:   // LIIM
                _st.InterruptMask=(_st.InterruptMask & 0b11111100)) | (Pipe2.x & 0b00000011));
                break;
              default:   // XOP
                i=_wp;
                _wp=GetIntValue((uint16_t)(0x0040+GET_REG((Pipe1 & 0b1111000000) >> 6)*4));
						    regs=(union T_REGISTERS *)&ram_seg[_wp & 0xff /* -RAM_START */];     // così oppure cast diretto...
                SET_REG(11,res3.x);
                SET_REG(13,i);
                SET_REG(14,_pc);
                SET_REG(15,_st.x);
                _st.XOP=1;
                _pc=GetIntValue(0x0042+GET_REG((Pipe1 & 0b1111000000)) >> 6)*4);
                break;
              }
            
#else
            i=_wp;
         		_wp=GetIntValue((uint16_t)(0x0040+GET_REG((Pipe1 & 0b1111000000) >> 6)*4));
				    regs=(union T_REGISTERS *)&ram_seg[_wp & 0xff /* -RAM_START */];     // così oppure cast diretto...
            SET_REG(11,res3.x);
            SET_REG(13,i);
            SET_REG(14,_pc);
            SET_REG(15,_st.x);
            _st.XOP=1;
           	_pc=GetIntValue((uint16_t)(0x0042+GET_REG((Pipe1 & 0b1111000000) >> 6)*4));
            break;
#endif
          }
        break;
      
      case 0b0011 << 12:    // Multiply, Divide, CRU
				COMPUTE_SOURCE_NOPIPE(1);
    		switch(Pipe1 & 0b1111110000000000) {
          case 0b001110 << 10:     // MPY Multiply
						res2.x=GET_WORKING_REG_D();
            res3.d = res1.x * res2.x;
            SET_WORKING_REG_D(HIWORD(res3.d));
            SET_REG(((WORKING_REG2_INDEX+1) /*& 0xf*/),res3.x);   // OKKIO, porcata, & se 15... dice che deve andare in memoria subito dopo! tipo R16
//no!            goto aggFlag;
            break;
          case 0b001111 << 10:     // DIV Divide
            res2.d = MAKELONG(GET_REG((WORKING_REG2_INDEX+1) /*& 0xf*/),GET_WORKING_REG_D());    // OKKIO...
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
            if(res2.d < res1.x)
              _st.Overflow = 1;
            else {
	            res3.d = res2.d / (uint32_t)res1.x;		// signed o unsigned??
              SET_WORKING_REG_D(LOWORD(res3.d));
              SET_REG((WORKING_REG2_INDEX+1) /*& 0xf*/,res2.d % (uint32_t)res1.x);   // OKKIO, porcata, & se 15... dice che deve andare in memoria subito dopo! tipo R16
              _st.Overflow = 0;
              }
            break;
            
          case 0b001100 << 10:     // LDCR Load communication register
						{uint8_t cnt=WORKING_REG2_INDEX;
            PutValueCRU(GET_REG(12),res1.x,cnt);
						res3.x=res1.x;
						if(cnt>8)
							goto aggFlag16Z;
						else
							goto aggFlag8Z;
						}
            break;
          case 0b001101 << 10:     // STCR Store communication register
						{uint8_t cnt=WORKING_REG2_INDEX;
            res3.x=GetValueCRU(GET_REG(12),cnt);
						if(cnt>8) {
							goto store16_S;
							}
						else {
							res3.b.l=res3.b.h;
							STORE_SOURCE_8 
							goto aggFlag8Z;
							}
						}
            break;
          }
        break;

			}

rallenta:
		if(cyclesSoFar>cyclesHW) {			// 0.8uS => 1.19MHz

			TMS9901Cnt--;		// finire...
			if(!TMS9901Cnt) {
				TMS9901Cnt=TMS9901Timer;
	//			TIMIRQ=1;
				}
			cyclesHW += HWClock;		// Timer
			}

		} while(!fExit);

	return 1;
	}


