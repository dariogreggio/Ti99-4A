
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

#include "tms9900_PIC.h"


#define UNIMPLEMENTED_MEMORY_VALUE 0xFF
#define RAM_START 0x8000
#define RAM_SIZE 256
//#define RAM_START 0xc000
//#define RAM_SIZE 16384 ma DEVE partire dal Top!!
//#define RAM_START 0xf000  // per VELOCIZZAER debug!
//#define RAM_SIZE 4096 // ma DEVE partire dal Top!!
//#define RAM_SIZE2 65536
//#define RAM_START2 0x0000   // in slot 2, v.sotto
#define ROM_START 0x0000
#define ROM_SIZE 8192
#define GROM_START 0x0000
#define GROM_SIZE (3*0x2000)    // in effetti 3 da 0x1800.. v. sotto
BYTE ram_seg[RAM_SIZE];
#ifdef RAM_SIZE2 
BYTE ram_seg2[RAM_SIZE2];
#endif
BYTE rom_seg[ROM_SIZE];			
#ifdef ROM_SIZE2 
BYTE rom_seg2[ROM_SIZE2];
#endif
BYTE grom_seg[GROM_SIZE];		// 3x	
#ifdef GROM_SIZE2
BYTE grom_seg2[GROM_SIZE2];
BYTE rom_seg2[ROM_SIZE2];
#endif
WORD GROMPtr;
BYTE GROMWriteStage,GROMBuffer;
volatile BYTE TIMIRQ,VIDIRQ;
volatile WORD TIMEr;
BYTE TMS9918Reg[8],TMS9918RegS,TMS9918Sel,TMS9918WriteStage,TMS9918Buffer;
WORD TMS9918RAMPtr;
BYTE TMS9919[1],TMSvolume[4];
WORD TMSfreq[4];    // https://www.unige.ch/medecine/nouspikel/ti99/tms9919.htm
BYTE TMS9901[32];   // https://www.unige.ch/medecine/nouspikel/ti99/tms9901.htm
WORD TMS9901Timer,TMS9901Cnt;
BYTE TMS5220[1];		// https://www.unige.ch/medecine/nouspikel/ti99/speech.htm
BYTE TMSVideoRAM[TMSVIDEORAM_SIZE];		// 
BYTE Keyboard[8],KeyboardCol=0;
SWORD VICRaster=MIN_RASTER;

extern volatile BYTE keysFeedPtr;


extern BYTE CPUPins;
extern BYTE ColdReset;
extern uint16_t Pipe1;
extern union __attribute__((__packed__)) PIPE Pipe2;



uint8_t GetValue(uint16_t t) {
	register uint8_t i;

#ifdef ROM_SIZE2
	if(t >= ROM_START2 && t < (ROM_START2+ROM_SIZE2)) 
		i=rom_seg2[t-ROM_START2];
	else if(t < ROM_SIZE) {			//
		i=rom_seg[t];
		}
#else
	if(t < ROM_SIZE) {			//
		i=rom_seg[t];
		}
#endif
	else if(t >= RAM_START && t < (RAM_START+RAM_SIZE*4)) {   // 256, mirrored (?)
		uint16_t t2;
		t2 = (t-RAM_START) & 0xff /* 0xfe*/;


/*      if(t == 0x072 || t == 0x073 || t == 0x074 ) {
				int T;
        T=0;
        }*/


		i=ram_seg[t2];
		}
	else 
		switch(t >> 8) {
			case 0x84:		// sound
				i=TMS9919[0];
				break;
			case 0x88:
				switch(t & 0x3e) {
					case 0x00:		// VDP read data
						TMS9918WriteStage=0;
						i=TMS9918Buffer;
						TMS9918Buffer=TMSVideoRAM[(TMS9918RAMPtr++) & (TMSVIDEORAM_SIZE-1)];
						break;
					case 0x02:		// VDP read status register
						i=TMS9918RegS;
						TMS9918RegS &= ~0b10100000;			// pulire anche flag sprite??
						TMS9918WriteStage=0;
						break;
					}
				break;
			case 0x8c:
				switch(t & 0x3e) {
					case 0x00:		// VDP write data (non dovrebbe esistere
						TMS9918WriteStage = 0;
						break;
					case 0x02:				// VDP write register (non dovrebbe esistere
						TMS9918WriteStage = 0;
						break;
					}
				break;
			case 0x94:		// speech
				i=TMS5220[0];
				break;
			case 0x98:
				{
				uint8_t sel=(t & 0x3e);
				switch(sel) {
					case 0x00:		// GROM read page 0
						{   // 
							WORD n;
							i=GROMBuffer;
#ifdef GROM_SIZE2
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else if(GROMPtr<0x6000+GROM_SIZE2)
								GROMBuffer = grom_seg2[GROMPtr-0x6000];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#else
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#endif
							n=GROMPtr & 0xe000;
							GROMPtr &= ~0xe000;
							GROMPtr=(GROMPtr+1-GROM_START) & (GROM_SIZE-1);
							GROMPtr |= n;
							}
/*						else {
							i=UNIMPLEMENTED_MEMORY_VALUE;
							}*/
						GROMWriteStage=0;
						break;
					case 0x02:		// GROM read address
					// da Classic99: address read is destructive;  Is the address incremented anyway? ie: if you keep reading, what do you get?
						i=HIBYTE(GROMPtr);		//uint8_t z=(GRMADD&0xff00)>>8;
						GROMPtr=MAKEWORD(LOBYTE(GROMPtr),LOBYTE(GROMPtr));		// 		GRMADD=(((GRMADD&0xff)<<8)|(GRMADD&0xff));
#if 0
						if(!GROMWriteStage) {   // least significant byte goes first
							i=HIBYTE(GROMPtr);		// big endian
							GROMWriteStage = 1;
							}
						else {    // https://forums.atariage.com/topic/360111-grom-addressing-for-dummies-please/ https://www.unige.ch/medecine/nouspikel/ti99/titechpages.htm
							i=LOBYTE(GROMPtr);		// big endian
							GROMWriteStage = 0;
							}
#endif
						break;
					case 0x04:		// GROM read page #1
						{   // 
							WORD n;
							i=GROMBuffer;
#ifdef GROM_SIZE2
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else if(GROMPtr<0x6000+GROM_SIZE2)
								GROMBuffer = grom_seg2[GROMPtr-0x6000];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#else
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#endif
							n=GROMPtr & 0xe000;
							GROMPtr &= ~0xe000;
							GROMPtr=(GROMPtr+1-GROM_START) & (GROM_SIZE-1);
							GROMPtr |= n;
							}
						GROMWriteStage=0;
						break;
					case 0x06:		// GROM read address #1
					// da Classic99: address read is destructive;  Is the address incremented anyway? ie: if you keep reading, what do you get?
						i=HIBYTE(GROMPtr);		//uint8_t z=(GRMADD&0xff00)>>8;
						GROMPtr=MAKEWORD(LOBYTE(GROMPtr),LOBYTE(GROMPtr));		// 		GRMADD=(((GRMADD&0xff)<<8)|(GRMADD&0xff));
						break;
					case 0x08:		// GROM read address #3
						{   // 
							WORD n;
							i=GROMBuffer;
#ifdef GROM_SIZE2
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else if(GROMPtr<0x6000+GROM_SIZE2)
								GROMBuffer = grom_seg2[GROMPtr-0x6000];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#else
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#endif
							n=GROMPtr & 0xe000;
							GROMPtr &= ~0xe000;
							GROMPtr=(GROMPtr+1-GROM_START) & (GROM_SIZE-1);
							GROMPtr |= n;
							}
						GROMWriteStage=0;
						break;
					case 0x20:		// GROM read page 1			VERIFICARE! questa arriva ma pare errore, 9b9b (è solo per GramKarte, dice
	//				case 0x40:		// GROM read page 2			VERIFICARE!
	//    case 0x60:
	//    case 0x80:
	//    case 0xa0:
					default:		// beh li controlla tutti da 04 a 3c in step da 4, come dice il doc
						i=UNIMPLEMENTED_MEMORY_VALUE;
						break;
					}
				}
				break;
			case 0x9c:		// GROM write data (non dovrebbe esistere
				switch(t & 0xfe) {
					case 0x00:						//  (non dovrebbe esistere
						{   // 
						WORD n;
#ifdef GROM_SIZE2
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else if(GROMPtr<0x6000+GROM_SIZE2)
								GROMBuffer = grom_seg2[GROMPtr-0x6000];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#else
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#endif
						n=GROMPtr & 0xe000;
						GROMPtr &= ~0xe000;
						GROMPtr=(GROMPtr+1-GROM_START) & (GROM_SIZE-1);
						GROMPtr |= n;
						}
						GROMWriteStage=0;
						break;
					case 0x02:		// GROM set address (non dovrebbe esistere
						GROMWriteStage=0;
						break;
					default:		// 
						GROMWriteStage=0;
						break;
					}
				break;
			}

	return i;
	}

uint16_t GetIntValue(uint16_t t) {
	register uint16_t i;

#ifdef ROM_SIZE2
	if(t >= ROM_START2 && t < (ROM_START2+ROM_SIZE2)) {
    t &= 0xfffe;
		t -= ROM_START2;
		i=MAKEWORD(rom_seg2[t+1],rom_seg2[t]);		// big-endian
		}
	else 	if(t < ROM_SIZE) {			//
    t &= 0xfffe;
		i=MAKEWORD(rom_seg[t+1],rom_seg[t]);		// big-endian
		}
#else
	if(t < ROM_SIZE) {			//
    t &= 0xfffe;
		i=MAKEWORD(rom_seg[t+1],rom_seg[t]);		// big-endian
		}
#endif
	else if(t >= RAM_START && t < (RAM_START+RAM_SIZE*4)) {   // 256 bytes, mirrored (?)
		t-=RAM_START;


/*      if(t == 0x072 || t == 0x073 || t == 0x074 ) {
				int T;
        T=0;
        }*/


    t &= 0xfe /*0xfe*/;
		i=MAKEWORD(ram_seg[t+1],ram_seg[t]);		// big-endian
		}

	return i;
	}

uint16_t GetValueCRU(uint16_t r12,uint8_t cnt) {
	uint8_t i,t;

#ifdef _DEBUG
/*					{char myBuf[128];
extern HFILE spoolFile;
					wsprintf(myBuf,"CRU  get: r12=%04x, %02x   sel=%u\n",r12,cnt,KeyboardCol);
				_lwrite(spoolFile,myBuf,strlen(myBuf));
				}*/
#endif


	// kbd:
//CRU put: r12=0024, 0500: 00 03			// col
//CRU get: r12=0006, 0005: ff					// rows

	if(!cnt)
		cnt=16;
	if(r12>=6 && r12<=0x14) {
//			if(( m_CapsLock == false ) && ( address == 7 ))		{
//https://www.ninerpedia.org/wiki/TI-99/4A_CRU_definitions
//			return 1;
//		}
/*		if(KeyboardCol==21) {// v.sotto

//			t= (GetKeyState(VK_CAPITAL) & 0x0001) ? 0xff : 0x00;
			return MAKEWORD(0xff,t);
			}*/

#if 0
		if(Joystick) {		// usare touch screen!!
			JOYINFO ji;
			//ecc
#define JOY_THRESHOLD 10000
			if(joyGetPos(JOYSTICKID1,&ji) == MMSYSERR_NOERROR) {
				if(ji.wButtons & JOY_BUTTON1)
					Keyboard[0]&=~B8(00000010);
				else
					Keyboard[0]|=B8(00000010);
/*				if(ji.wButtons & JOY_BUTTON2)
					Keyboard[0]&=~B8(00000001);
				else
					Keyboard[0]|=B8(00000001);*/
				if(ji.wXpos<0x8000-JOY_THRESHOLD)
					Keyboard[1]&=~B8(00000010);
				else
					Keyboard[1]|=B8(00000010);
				if(ji.wXpos>0x8000+JOY_THRESHOLD)
					Keyboard[2]&=~B8(00000010);
				else
					Keyboard[2]|=B8(00000010);
				if(ji.wYpos<0x8000-JOY_THRESHOLD)
					Keyboard[4]&=~B8(00000010);
				else
					Keyboard[4]|=B8(00000010);
				if(ji.wYpos>0x8000+JOY_THRESHOLD)
					Keyboard[3]&=~B8(00000010);
				else
					Keyboard[3]|=B8(00000010);
				}
			}
#endif
    
		t=0x0;
		for(i=1; i<=cnt; i++) {			// cnt = 1..8 qua (direi
			t >>= 1;
//			t |= 0x80;

			if(!(Keyboard[r12/2-3 /*6..14hex*/  +i-1] & (1 << (7-KeyboardCol))))
				t &= ~0x80;			// togliere, dunque...
			else
				t |= 0x80;

			}
		for(i; i<=8; i++) {
			t >>= 1;
//			t |= 0x80;
			}
		return MAKEWORD(0xff,t);
		}
	else if(r12==0x24) {
		return MAKEWORD(0xff,KeyboardCol);
		}
	else if(r12==0x04) {		// o è questo?? vertical sync dice, in IRQ, e cnt=2
		return 0x00;
		}
	else if(r12==0x02) {		// ?? DOVREBBE esse peripheral IRQ
		return 0x00;
		}
	else if(r12==0x00) {		// ?? DOVREBBE essere Timer

		if(KeyboardCol==21) {
/*If you want to detect the Alpha key:
CLR R12 Set CRU base to zero for the TMS9901
SBZ 21 Set the P5 line on the TMS9901 to zero (low)
TB 7 Test ROW INT7 for the Alpha key. EQ bit 0 if down, 1 if up
SBO 21 Be sure to turn off the P5 line on the way out! (SBO does not affect EQ bit)*/
//			t= (GetKeyState(VK_CAPITAL) & 0x0001) ? 0xff : 0x00;
			return MAKEWORD(0xff,t);
			}
		return 0x00;
		}

	return 0xffFF;


/*	else if(t >= 0x1300 && t < 0x1400) {		// RS232/Timer (CRU?? https://www.unige.ch/medecine/nouspikel/ti99/cru.htm
		}
	else if(t >= 0x0000 && t < 0x0400) {		// Keyboard (CRU?? https://www.unige.ch/medecine/nouspikel/ti99/cru.htm
		}*/
	}

uint16_t GetPipe(uint16_t t) {

#ifdef ROM_SIZE2
	if(t >= ROM_START2 && t < (ROM_START2+ROM_SIZE2)) {
    t &= 0xfffe;
		t -= ROM_START2;
	  Pipe1=MAKEWORD(rom_seg2[t+1],rom_seg2[t]);
		Pipe2.x=MAKEWORD(rom_seg2[t+3],rom_seg2[t+2]);
		}
	else if(t < ROM_SIZE) {			//
		t &= 0xfffe;
	  Pipe1=MAKEWORD(rom_seg[t+1],rom_seg[t]);
		Pipe2.x=MAKEWORD(rom_seg[t+3],rom_seg[t+2]);
		}
#else
	if(t < ROM_SIZE) {			//
		t &= 0xfffe;
	  Pipe1=MAKEWORD(rom_seg[t+1],rom_seg[t]);
		Pipe2.x=MAKEWORD(rom_seg[t+3],rom_seg[t+2]);
		}
#endif
	else if(t >= RAM_START && t < (RAM_START+RAM_SIZE*4)) {   // 256, mirrored (?) NON SONO SICURO QUA ABBIA SENSO!
		t-=RAM_START;
		t &= 0xfe /*0xfe*/;
	  Pipe1=MAKEWORD(ram_seg[t+1],ram_seg[t]);
		Pipe2.x=MAKEWORD(ram_seg[t+3],ram_seg[t+2]);
		}

	return Pipe1;
	}

void PutValue(uint16_t t,uint8_t t1) {
  int j;

// printf("rom_seg: %04x, p: %04x\n",rom_seg,p);

	if(t >= RAM_START && t < (RAM_START+RAM_SIZE*4)) {   // 256, mirrored  (?)
		uint16_t t2;
		t2 = (t-RAM_START) & 0xff /*0xfe*/;
		ram_seg[t2]=t1;


		/*
      if(t == 0x072 || t == 0x073 || t == 0x074 ) {
				int T;
        T=0;
        }*/


		}
	else 
    switch(t >> 8) {
      case 0x84:
			{
			uint8_t sel=(t & 0x3e),chan=(t1 >> 5) & 3;
			switch(sel) {
				case 0x0:		// sound				https://unige.ch/medecine/nouspikel/ti99/tms9919.htm
					TMS9919[0]=t1;
					if(t1 & 0x80) {		// comando
						if(t1 & 0x10) {		// volume (9f bf df ff alla partenza
							TMSvolume[chan]=t1 & 0xf;   // (15=muto)
              goto set_wave;
							}
						else {			// parte bassa freq
							TMSfreq[chan]=t1 & 0xf;
							}
						}
					else {				// 2° parte freq
/*						if(t1==0x20)		// patch brutale! sarebbe il secondo parametro: bf df ff  80 05 92
							PlayResource(MAKEINTRESOURCE(IDR_WAVE_TONE1),FALSE);
						else if(t1==0x05)
							PlayResource(MAKEINTRESOURCE(IDR_WAVE_TONE2),FALSE);*/
            
						TMSfreq[chan] |= (t1 & 0x3f) << 4;
            
set_wave:							
						if(TMSfreq[chan] /*safety*/ && TMSvolume[chan]<15) {			// 
              j=(7*TMSfreq[chan])/2;   // la freq reale è 223700L/2/TMSfreq[chan]
              PR2 = j;		 // 80=~20KHz (100MHz/64 (=1562500)  /80)
#ifdef ST7735
              OC1RS = j/2;		 // 
              OC1CONbits.ON = 1;
#endif
#ifdef ILI9341
              OC7RS = j/2;		 // 
              OC7CONbits.ON = 1;
#endif
              
              
//              __delay_ms(1000);
              
              
              }
            else {
#ifdef ST7735
              OC1CONbits.ON = 0;
#endif
#ifdef ILI9341
              OC7CONbits.ON = 0;
#endif
              }
						}
/*
Generator 	Frequency 	Volume
Tone 1 			>8z >xy 		>9v
Tone 2 			>Az >yx 		>Bv
Tone 2 			>Cz >yx 		>Dv
Noise 			>En 				>Fv

Frequency = 111860.8 Hz / xyz
Volume v:  +1 = -2 dB (>F = off)
*/
#ifdef _DEBUG
/*				{char myBuf[128];
extern HFILE spoolFile;
					wsprintf(myBuf,"sound write: %02x\n",t1);
				_lwrite(spoolFile,myBuf,strlen(myBuf));
				}*/
#endif
	        break;
				}
				}
        break;
      case 0x88:		// VDP read data (non dovrebbe esistere
				{
				uint8_t sel=(t & 0x3e);
				switch(sel) {
		      case 0x00:		// VDP read data (non dovrebbe esistere
				    TMS9918WriteStage = 0;
		        break;
					case 0x02:		// VDP read status register (non dovrebbe esistere
						TMS9918WriteStage = 0;
						break;
					}
					break;
				}
				break;
      case 0x8c:	// VDP write data
				{
				uint8_t sel=(t & 0x3e);
				switch(sel) {
					case 0x00:	// VDP write data
						TMS9918WriteStage=0;

#ifdef _DEBUG
/*				{char myBuf[128];
extern HFILE spoolFile;
					wsprintf(myBuf,"videoRAM write: %04X: %02x; GROM ptr=%04X\n",TMS9918RAMPtr,t1,GROMPtr);
				_lwrite(spoolFile,myBuf,strlen(myBuf));
				}*/
#endif
						TMSVideoRAM[(TMS9918RAMPtr++) & (TMSVIDEORAM_SIZE-1)]=t1;
						break;
					case 0x02:		// VDP write register
						if(!TMS9918WriteStage) {   /* first stage byte - either an address LSB or a register value */
							TMS9918Sel = t1;
							TMS9918WriteStage = 1;
							}
						else {    /* second byte - either a register number or an address MSB */
							if(t1 & 0x80) { /* register */
				//          if((t1 & 0x7f) < 8)
								TMS9918Reg[t1 & 0x07] = TMS9918Sel;
								}
							else {  /* address */
								TMS9918RAMPtr = MAKEWORD(TMS9918Sel,t1 & 0x3f);
								if(!(t1 & 0x40)) {
									TMS9918Buffer = TMSVideoRAM[(TMS9918RAMPtr++) & (TMSVIDEORAM_SIZE-1)];
									}
								}
							TMS9918WriteStage = 0;
							} 
		        break;
					} 
				} 
        break;
      case 0x94:		// speech
        TMS5220[0]=t1;
        break;
			case 0x98:
				{
				uint8_t sel=(t & 0x3e);
				switch(sel) {
					case 0x00:		// GROM read page 0 (non dovrebbe esistere
					case 0x20:		// GROM read page 1 (non dovrebbe esistere
						{   // 
						WORD n;
//						i=GROMBuffer;
#ifdef GROM_SIZE2
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else if(GROMPtr<0x6000+GROM_SIZE2)
								GROMBuffer = grom_seg2[GROMPtr-0x6000];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#else
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#endif
						n=GROMPtr & 0xe000;
						GROMPtr &= ~0xe000;
						GROMPtr=(GROMPtr+1-GROM_START) & (GROM_SIZE-1);
						GROMPtr |= n;
						}
						GROMWriteStage=0;
			      break;
		      case 0x02:		// GROM read address (non dovrebbe esistere
					  GROMWriteStage=0;
						break;
					}
				}
        break;
      case 0x9c:		// GROM write data (ev. GRAM, dice
				{
				uint8_t sel=(t & 0x3e);
				switch(sel) {
					case 0x00:		// GROM write data (ev. GRAM, dice
						{   // 
						WORD n;
						grom_seg[GROMPtr-GROM_START]=t1;
						n=GROMPtr & 0xe000;
						GROMPtr &= ~0xe000;
						GROMPtr=(GROMPtr+1-GROM_START) & (GROM_SIZE-1);
						GROMPtr |= n;
						}
						GROMWriteStage=0;
						break;
					case 0x02:		// GROM set address
						if(!GROMWriteStage) {   // least significant byte goes first
							GROMPtr = MAKEWORD(LOBYTE(GROMPtr),t1);
							GROMWriteStage = 1;
							}
						else {    // https://forums.atariage.com/topic/360111-grom-addressing-for-dummies-please/
							WORD n;
							GROMPtr = MAKEWORD(t1,HIBYTE(GROMPtr));
#ifdef GROM_SIZE2
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else if(GROMPtr<0x6000+GROM_SIZE2)
								GROMBuffer = grom_seg2[GROMPtr-0x6000];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#else
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#endif
							n=GROMPtr & 0xe000;
							GROMPtr &= ~0xe000;
							GROMPtr=(GROMPtr+1-GROM_START) & (GROM_SIZE-1);
							GROMPtr |= n;
							GROMWriteStage = 0;
							}
						break;
					case 0x06:		// GROM set address #2
						if(!GROMWriteStage) {   // least significant byte goes first
							GROMPtr = MAKEWORD(LOBYTE(GROMPtr),t1);
							GROMWriteStage = 1;
							}
						else {    // https://forums.atariage.com/topic/360111-grom-addressing-for-dummies-please/
							WORD n;
							GROMPtr = MAKEWORD(t1,HIBYTE(GROMPtr));
#ifdef GROM_SIZE2
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else if(GROMPtr<0x6000+GROM_SIZE2)
								GROMBuffer = grom_seg2[GROMPtr-0x6000];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#else
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#endif
							n=GROMPtr & 0xe000;
							GROMPtr &= ~0xe000;
							GROMPtr=(GROMPtr+1-GROM_START) & (GROM_SIZE-1);
							GROMPtr |= n;
							GROMWriteStage = 0;
							}
						break;
					default:			// tutti gli altri ?!
						if(!GROMWriteStage) {   // least significant byte goes first
							GROMPtr = MAKEWORD(LOBYTE(GROMPtr),t1);
							GROMWriteStage = 1;
							}
						else {    // https://forums.atariage.com/topic/360111-grom-addressing-for-dummies-please/
							WORD n;
							GROMPtr = MAKEWORD(t1,HIBYTE(GROMPtr));
#ifdef GROM_SIZE2
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else if(GROMPtr<0x6000+GROM_SIZE2)
								GROMBuffer = grom_seg2[GROMPtr-0x6000];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#else
							if(GROMPtr<0x6000)
								GROMBuffer = grom_seg[GROMPtr];
							else
								GROMBuffer = UNIMPLEMENTED_MEMORY_VALUE;
#endif
							n=GROMPtr & 0xe000;
							GROMPtr &= ~0xe000;
							GROMPtr=(GROMPtr+1-GROM_START) & (GROM_SIZE-1);
							GROMPtr |= n;
							GROMWriteStage = 0;
							}
						break;
					}
				}
        break;
      break;
		}

	}

void PutValueCRU(uint16_t r12,uint16_t t,uint8_t cnt) {

#ifdef _DEBUG
/*					{char myBuf[128];
extern HFILE spoolFile;
					wsprintf(myBuf,"CRU put: r12=%04x, %04x %02x\n",r12,t,cnt);
				_lwrite(spoolFile,myBuf,strlen(myBuf));
				}*/
#endif
//	al boot:
//CRU put: r12=0006, 0000: 00 08			// kbd
//CRU put: r12=0020, 0000: 00 01			// ?? n.c.
//CRU put: r12=0030, 0000: 00 01			// audio
//CRU put: r12=0004, FF00: 00 01			// VDU irq
//CRU put: r12=0002, FF00: 00 01			// periph irq
//CRU put: r12=002c, FF00: 00 02			// CS1 motor

	// kbd:
//CRU put: r12=0024, 0500: 00 03			// col
//CRU get: r12=0006, 0005: ff					// rows


	if(r12==0x30) {			// audio gate ... no?
		t=1;
		}
	if(r12==0x24) {			// tastiera
		KeyboardCol=HIBYTE(t) & 7;		// cnt=3 per 3 bit, ma ok ignoro
		}
/*	else if(t >= 0x1300 && t < 0x1400) {		// RS232/Timer (CRU?? https://www.unige.ch/medecine/nouspikel/ti99/cru.htm
		}
	else if(t >= 0x0000 && t < 0x0400) {		// Keyboard (CRU?? https://www.unige.ch/medecine/nouspikel/ti99/cru.htm
		}*/
	if(r12==0x00) {			// tastiera?? cos'è? è a inizio SCAN keyboard con 21: potrebbe essere caps-lock (da Classic99
		//https://www.ninerpedia.org/wiki/TI-99/4A_CRU_definitions
		KeyboardCol=21;
//				m_CapsLock = ( data != 0 ) ? true : false;
		 //((GetKeyState(VK_CAPITAL) & 0x0001)!=0)

		}
	if(r12==0x2a) {			// NON SI CAPISCE
		KeyboardCol=21;
		if(cnt==21) {
//			t= (GetKeyState(VK_CAPITAL) & 0x0001) ? 0xff : 0x00;
//			return MAKEWORD(0xff,t);
			}
		}
	}

void PutIntValue(uint16_t t,uint16_t t1) {
	register uint16_t i;

// printf("rom_seg: %04x, p: %04x\n",rom_seg,p);

	if(t >= RAM_START && t < (RAM_START+RAM_SIZE*4)) {   // 256 bytes, mirrored (?)
		t-=RAM_START;
		t &= 0xfe /*0xfe*/;
	  ram_seg[t++]=HIBYTE(t1);			// big-endian 
	  ram_seg[t]  =LOBYTE(t1);
		}

  }


void initHW(void) {
  int i;
extern const unsigned char charset_international[2048],tmsFont[(128-32)*8];
  struct SPRITE_ATTR *sa;

	GROMWriteStage=0;
	GROMPtr=0;

	memset(Keyboard,0xff,sizeof(Keyboard));
	KeyboardCol=0;

  
	TMS9919[0]=0b00000000;
	memset(TMSvolume,0,sizeof(TMSvolume));
	memset(TMSfreq,0,sizeof(TMSfreq));
  TMS9901[0]=0;
	TMS9901Timer=0; TMS9901Cnt=0;
  TMS5220[0]=0;
  
  memset(TMSVideoRAM,0,TMSVIDEORAM_SIZE);    // mah...
// Ti99 dice:	The real 9918A will set all VRs to 0, which basically makes the screen black, blank, and off, 4K VRAM selected, and no interrupts. 
  TMS9918Reg[0]=TMS_R0_EXT_VDP_DISABLE | TMS_R0_MODE_GRAPHICS_I;
  TMS9918Reg[1]=TMS_R1_RAM_16K | TMS_R1_MODE_GRAPHICS_I /* bah   | TMS_R1_DISP_ACTIVE | TMS_R1_INT_ENABLE*/;
  TMS9918Reg[2]=TMS_DEFAULT_VRAM_NAME_ADDRESS >> 10;
  TMS9918Reg[3]=TMS_DEFAULT_VRAM_COLOR_ADDRESS >> 6;
  TMS9918Reg[4]=TMS_DEFAULT_VRAM_PATT_ADDRESS >> 11;
  TMS9918Reg[5]=TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS >> 7;
  TMS9918Reg[6]=TMS_DEFAULT_VRAM_SPRITE_PATT_ADDRESS >> 11;
  TMS9918Reg[7]=(1 /*black*/ << 4) | 15 /*bianco*/;		//(1 /*black*/ << 4) | 7 /*cyan*/;
  TMS9918RegS=0;
  TMS9918Sel=TMS9918WriteStage=0;
//  memcpy(TMSVideoRAM+TMS_DEFAULT_VRAM_PATT_ADDRESS,charset_international,2048);// mah... non serve
//  memcpy(TMSVideoRAM+TMS_DEFAULT_VRAM_PATT_ADDRESS,tmsFont,(128-32)*8);
  sa=(struct SPRITE_ATTR *)&TMSVideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS];
  for(i=0; i<32; i++) {
//    TMSVideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS+i*4]=LAST_SPRITE_YPOS;
    sa->ypos=LAST_SPRITE_YPOS;
    sa->xpos=sa->tag=sa->name=0;
//    TMSVideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS+i*4+1]=0;
//    TMSVideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS+i*4+2]=0;
//    TMSVideoRAM[TMS_DEFAULT_VRAM_SPRITE_ATTR_ADDRESS+i*4+3]=0;
    sa++;
    }

//  for(i=0; i<768; i++)		non serve idem
//    TMSVideoRAM[TMS_DEFAULT_VRAM_NAME_ADDRESS+i]=i & 0xff;
  
 
 
  keysFeedPtr=255; //
  
#ifdef ST7735
  OC1CONbits.ON = 0;   // spengo buzzer/audio
  PR2 = 65535;		 // 
  OC1RS = 65535;		 // 
#endif
#ifdef ILI9341
  OC7CONbits.ON = 0;   // spengo buzzer
  PR2 = 65535;		 // 
  OC7RS = 65535;		 // 
#endif

  }

