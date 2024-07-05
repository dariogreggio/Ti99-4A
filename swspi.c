#include <xc.h>
#include "tms9900_PIC.h"
#include "swspi.h"


void DoSPIStart(void) {								// Master, start (CLK negativo)
	//call setDataInput				; SDI=input
// qua no	SPISDITris=1;				// SDI è input
	SPISDOTris=0;				// SDO è output
	SPISCKTris=0;				// SCK è output
	
	m_SPISCKBit=0;						// CLK=0
	m_SPISDOBit=1;						// SDO=1 (CS5460)
	m_SPICSBit=0;
	Delay_SPI();
	}

void DoSPIStop(void) {							// Master, stop

	m_SPICSBit=1;
	Delay_SPI();
	}


#ifndef USA_SPI_HW
		
void DoSPIMO(BYTE temp) {							// Master, output (CLK negativo): entra in W il BYTE
	BYTE tempC;

												// PRIMA fare DoSPIStart!!
	tempC=8;

	do {
//  clrwdt			; già in Delay_uS
		if(temp & 0x80)					// MSB first
			m_SPISDOBit=1;					// DATA=1
		else
			m_SPISDOBit=0;					// DATA=0

		temp <<= 1;
    
//		Delay_1uS();					// e' giusto?? mah...
		m_SPISCKBit=1;      // sampling sul fronte di salita
		Delay_SPI();			//3.5MHz circa, 5.7.19
		m_SPISCKBit=0;
//		Delay_SPI();
		}	while(--tempC);

	}

BYTE DoSPIMI(void)	{					// Master, input (CLK negativo): esce in W il BYTE se C=0 o C=1=errore
	BYTE temp=0,tempC;			// 
													// PRIMA fare DoSPIStart!!
	tempC=8;

	do {
//  clrwdt			; già in Delay_uS
		Delay_SPI();
		m_SPISCKBit=1;
		Delay_SPI();

		temp <<= 1;						// MSB first
		if(m_SPISDIBit)
			temp |= 1;

		m_SPISCKBit=0;
		}	while(--tempC);

	return temp;
	}


#else


#endif


