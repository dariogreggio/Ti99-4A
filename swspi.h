#include "tms9900_PIC.h"

void DoSPIStart(void);

void DoSPIStop(void);
		
void DoSPIMO(BYTE );

BYTE DoSPIMI(void);

//#define Delay_SPI() Delay_uS(1)  era 500uS circa, 26.2.16
#ifdef SLOW_CLOCK
#define Delay_SPI() {	Nop();	Nop();	Nop();	ClrWdt(); }						// faccio così!
#else
#define Delay_SPI() {	ClrWdt(); }						// faccio così! non perfetto...
#endif


