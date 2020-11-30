/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430G2x33/G2x53 Demo - ADC10, DTC Sample A2-0, AVcc, Single Sequence, DCO
//
//  Description: Sample A3/A2/A1 as single sequence with reference to AVcc.
//  Software sets ADC10SC to trigger sample sequence. In Mainloop MSP430 waits
//  in LPM0 to save power until ADC10 conversion complete, ADC10_ISR(DTC) will
//  force exit from any LPMx in Mainloop on reti. ADC10_ISR will force any LPMx
//  exit. ADC10 internal oscillator times sample period (16x) and conversion
//  (13x). DTC transfers conversion code to RAM 200h - 206h. P1.0 set at start
//  of conversion burst, reset on completion.
//
//               MSP430G2x33/G2x53
//            -----------------
//        /|\|              XIN|-
//         | |                 |
//         --|RST          XOUT|-
//           |                 |
//       >---|P1.3/A3      P1.0|-->LED
//       >---|P1.2/A2          |
//       >---|P1.1/A1          |
//
//  D. Dang
//  Texas Instruments Inc.
//  December 2010
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
#include <msp430.h>
#include <math.h>
#define ADC_ref 3.3


#define RLED_ON() P1OUT |= 0x01;
#define RLED_OFF() P1OUT &= ~0x01;
#define GLED_ON() P1OUT |= 0x40;
#define GLED_OFF() P1OUT &= ~0x40;

#define Sensor1_PowerON() P2OUT |=0x01  //2.0
#define Sensor1_PowerOFF() P2OUT &=~0x01  //2.0
#define Sensor2_PowerON() P2OUT |=0x02  //2.1
#define Sensor2_PowerOFF() P2OUT &=~0x02  //2.1
#define Sensor3_PowerON() P2OUT |=0x04  //2.2
#define Sensor3_PowerOFF() P2OUT &=~0x04  //2.2

void sendChar(char);
void sendString(char *);
void delay(char);
void uart_init(void);
void adc_init(void);
void system_init(void);
void send_DataAqu(void);
void write_WordC(int, char);
void dummy_write_WordC(void);
int read_WordC(char);
void read_Config(void);
void write_Config(void);
char rxbuffer[12];
volatile int rx_value=0;
volatile int rxcount = 0;
volatile int AckEn = 0;
volatile int MemEn = 0;
volatile int SenEn =0;
unsigned int rainfall_th;
unsigned int slopeangle_th;
unsigned int moisure_th;
unsigned int read_val[3];
unsigned int read_adc[3];
unsigned int rainfall_val,slopeangle_val,moisure_val,power_val;
int DeviceID = 0x01;
char alert ='-';
int Default_Power[4]={00,77,71,70};

int rvalue;
int main(void)
{

	  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	  system_init();
	  uart_init();
	  adc_init();

	  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
	   __bis_SR_register(GIE);                      // Set P1.0 output

	   AckEn=0;
	   MemEn=0;
	   RLED_OFF();
	   read_Config();


	   Sensor1_PowerOFF();

	   Sensor2_PowerOFF();

	   Sensor3_PowerOFF();

  for (;;)
  {

	     ADC10CTL0 &= ~ENC;
	     while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
	     ADC10SA = read_val;                        // Data buffer start

	     ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
	  //   __bis_SR_register(GIE);        // LPM0, ADC10_ISR will force exit

	     rainfall_val= ((1024 - read_val[2])*100)/1024;
	     if(SenEn==1){
	     slopeangle_val=((512 - read_val[1] - 30)*360)/512;
	     moisure_val=((1024 - read_val[0])*100)/1024;
	     }
	     else{
	    	 slopeangle_val=0;
	    	 moisure_val=0;
	     }

	//     if(rainfall_val>999)
	//    	 rainfall_val=999;
	//     if(slopeangle_val>999)
	//    	 slopeangle_val=999;
	//     if(moisure_val>999)
	//    	 moisure_val=999;

	     		if( AckEn ){
	     		delay(100);
	     		sendChar('0' + DeviceID/10);
	     		sendChar('0' + DeviceID%10);
	            sendChar( alert);
	            send_DataAqu();
	        	AckEn =0;
	     		}

	     		if(MemEn){

	     			write_Config();
	     			delay(200);
	     			read_Config();

	     			MemEn=0;
	     			RLED_OFF();
	     		}


	        	if(rainfall_val<=rainfall_th){  //0 Rainfall 1 Accce  2 Moisure

	        		GLED_OFF();
	        		alert='-';
	        		SenEn=0;
	        		Sensor2_PowerOFF();
	        		Sensor3_PowerOFF();
	        		power_val = (Default_Power[DeviceID])*9;
	        	}
	        	else
	        	{
	        		 SenEn=1;
	        		 Sensor2_PowerON();
	        		 Sensor3_PowerON();

		        	if( slopeangle_val <= slopeangle_th || (moisure_val<= moisure_th)){  //0 Rainfall 1 Accce  2 Moisure
		        		power_val = (Default_Power[DeviceID] + 9)*9;
		        		GLED_OFF();
		        		alert='-';
		        	}
		        	else
		        	{
		        	 GLED_ON();
		        	 power_val = (Default_Power[DeviceID] + 10)*9;
		        	 alert = '~';
		        	 //sendChar('0' + DeviceID/10);
		        	 //sendChar('0' + DeviceID%10);
		        	 //sendChar( '~');
		        	 //send_DataAqu();
		        	 //delay(100);
		        	}

	        	}


	  }
}

// ADC10 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
#else
#error Compiler not supported!
#endif
{
 // __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}


#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{


	 	 rxbuffer[rxcount++]=UCA0RXBUF;


	 	 if(rxbuffer[0] == 'M'){

	 		if(rxcount>11){

	 			rx_value = rxbuffer[1] - '0';
	 			rx_value = (rx_value <<8 ) | (rxbuffer[2] - '0');
	 			 if(DeviceID==rx_value)                 // TX -> RXed character
	 			 {
	 				RLED_ON();
	 				MemEn=1;

	 			 }

	 			rx_value=0;
	 			rxcount=0;
	 		}


	 	 }
	 	 else{

			 if(rxcount>1){

				 rx_value = rxbuffer[0] - '0';
				 rx_value = (rx_value <<8 ) | (rxbuffer[1] - '0');
				 if(DeviceID==rx_value)                 // TX -> RXed character
				  {

					  AckEn= 1;

				  }

				 rx_value=0;
				 rxcount=0;
			 }


	 	 }




}


void send_DataAqu(){

	sendChar( '0' + (rainfall_val/100));
	sendChar( '0' + (rainfall_val%100)/10);
	sendChar( '0' + (rainfall_val%10));
	sendChar( '-');
	sendChar( '0' + (slopeangle_val/100));
	sendChar( '0' + (slopeangle_val%100)/10);
	sendChar( '0' + (slopeangle_val%10));
	sendChar( '-');
	sendChar( '0' + (moisure_val/100));
	sendChar( '0' + (moisure_val%100)/10);
	sendChar( '0' + (moisure_val%10));
	sendChar( '-');
	sendChar( '0' + (power_val/100));
	sendChar( '0' + (power_val%100)/10);
	sendChar( '0' + (power_val%10));
}

void adc_init(){

	ADC10CTL1 = INCH_5 + CONSEQ_1;            // A3/A2/A1, single sequence
	ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;
	ADC10DTC1 = 0x03;                         // 3 conversions
	ADC10AE0 |= 0x38;                         // P1.3,2,1 ADC10 option select


}

void system_init(){

	 if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
		  {
		    while(1);                               // do not load, trap CPU!!
		  }
		  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
		  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
		  DCOCTL = CALDCO_1MHZ;
		  P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
		  P1SEL2 = BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD
		  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
		  P1DIR = 0x41;                             // P1.0 P1.6 output, else input

		  P2DIR = 0x07;
		  FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator

}


void uart_init()
{


	  UCA0BR0 = 104;                            // 1MHz 9600
	  UCA0BR1 = 0;                              // 1MHz 9600
	  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
	  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

}



void dummy_write_WordC ()
{
	 char *Flash_ptr;
	 Flash_ptr = (char *) 0x1040;              // Initialize Flash pointer
	   FCTL1 = FWKEY + ERASE;                    // Set Erase bit
	   FCTL3 = FWKEY;                            // Clear Lock bit
	   *Flash_ptr = 0;                           // Dummy write to erase Flash segment
	   FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}

void write_WordC (int value, char loc)
{
  char *Flash_ptr;                          // Flash pointer
  unsigned int i;
  char value_arr[2];

  Flash_ptr = (char *) 0x1040 + loc*2;              // Initialize Flash pointer
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  FCTL3 = FWKEY;                            // Clear Lock bit
 // *Flash_ptr = 0;                           // Dummy write to erase Flash segment

  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
  value_arr[0] = (value)&0xFF;
  value_arr[1] = (value>>8)&0xFF;

  for (i=0 ; i<2; i++)
  {
    *Flash_ptr++ = value_arr[i];                   // Write value to flash
  }

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}


int read_WordC (char loc)
{
  char *Flash_ptr;                          // Flash pointer
  unsigned int i;
  int value;
  char value_arr[2];

  Flash_ptr = (char *) 0x1040 + loc*2;              // Initialize Flash pointer
  FCTL3 = FWKEY;                            // Clear Lock bit
 // *Flash_ptr = 0;                           // Dummy write to erase Flash segment


  for (i=0 ; i<2; i++)
  {
	  value_arr[i] = *Flash_ptr++;                   // read value to flash
  }

  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
  value  = value_arr[0] | value_arr[1]<<8 ;

  return value;

}


void read_Config(){

rainfall_th = read_WordC(0);
slopeangle_th = read_WordC(1);
moisure_th = read_WordC(2);
}

void write_Config(){
int rth,sth,mth;

rth = (rxbuffer[3]-'0')*100 + (rxbuffer[4]-'0')*10 + (rxbuffer[5]-'0');
sth = (rxbuffer[6]-'0')*100 + (rxbuffer[7]-'0')*10 + (rxbuffer[8]-'0');
mth = (rxbuffer[9]-'0')*100 + (rxbuffer[10]-'0')*10 + (rxbuffer[11]-'0');

dummy_write_WordC();

delay(100);
write_WordC(rth,0);
delay(100);
write_WordC(sth,1);
delay(100);
write_WordC(mth,2);

}


void sendChar(char c){


	while (!(IFG2&UCA0TXIFG));
	UCA0TXBUF=c;



}

void sendString(char *s){


	while(*s!='\0'){
	sendChar(*s++);
	}


}

void delay(char dly)
{
while(dly--){
	_delay_cycles(9000);
}

}
