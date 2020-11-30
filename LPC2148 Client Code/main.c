/******************************************************************************/
/* MAIN.C: Client Code by (www.embedded.club)                                                      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <LPC21xx.H>                       /* LPC21xx definitions */

#define BIT(x) (1<<x)

#define LED BIT(21)



#define MAX_CHAR_IN_ONE_LINE 16
#define LCD_DATA_MASK   (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7)
#define LCD_BUSY_FLAG    LCD_D7

#define LCD_DATA_DIR   IO0DIR
#define LCD_DATA_SET   IO0SET
#define LCD_DATA_CLR   IO0CLR

#define LCD_CTRL_DIR   IO1DIR
#define LCD_CTRL_SET   IO1SET
#define LCD_CTRL_CLR   IO1CLR

#define LCDRS	(1 << 24)
#define LCDRW	(1 << 23)
#define LCDEN	(1 << 22)

#define LCD_D4  (1 << 10)
#define LCD_D5  (1 << 11)
#define LCD_D6  (1 << 12)
#define LCD_D7  (1 << 13)

#define PORT1_DIR   IO1DIR
#define PORT1_SET   IO1SET
#define PORT1_CLR   IO1CLR

#define D1_STATUS_LED    (1<<16)
#define D2_STATUS_LED    (1<<17)
#define D3_STATUS_LED    (1<<18)

#define D1_ALERT_LED    (1<<19)
#define D2_ALERT_LED    (1<<20)
#define D3_ALERT_LED    (1<<21)

#define BUZZER (1<<27)
/* Macro Definitions */

long delay_val = 5000; 
int press_count =0;
int device_display_count=0;
int config_mode=0;
char device[3][3]={'D','1','\0',
									'D','2','\0',
									'D','3','\0'};

void uart0_init();
void sendChar(char);
void sendString(char*);
void uart1_init();
void printChar(char);
void printString(char*);
void delay(int);
void delay_ms(unsigned int count);
void lcd_init(void);
void lcd_command_write( unsigned char command );
void lcd_data_write( unsigned char data );
void lcd_putstring( unsigned char line, char *string );
void wait_lcd( void );
int get_rainfall_val(void);
int get_inclined_val(void);
int get_moisure_val(void);
int get_power_val(void);
int get_alert(void);
int get_alertdeviceID(void);
void get_deviceList(void);
void clearData(void);
char rxbuffer0[12];
char rxbuffer1[18];
char rainfall_val[5]={'0','0','0','%','\0'};
char inclined_val[5]={'0','0','0','\xDF','\0'};
char moisure_val[5]={'0','0','0','%','\0'};
char power_val[6]={'0','0','0','m','W','\0'};
char device_list[16]={'D','-','-',' ','D','-','-',' ','D','-','-',' ','[','-',']','\0'};
char alertfrom[15]={'A','l','e','r','t',' ','D','-',' ','D','-',' ','D','-','\0'};
char d1_active=0;
char d2_active=0;
char d3_active=0;
char alert_detected=0;
int rainfall_ival,inclined_ival,moisure_ival,power_ival;

int rxcount0 = 0;
int rxcount1 = 0;
int loopcount=0;
int main()
{

   IODIR0 |= 0x00FF0000;
	 PORT1_DIR |= ( D1_STATUS_LED | D1_ALERT_LED | D2_STATUS_LED | D2_ALERT_LED | D3_STATUS_LED | D3_ALERT_LED | BUZZER );
 	 PORT1_CLR |= ( D1_STATUS_LED | D1_ALERT_LED | D2_STATUS_LED | D2_ALERT_LED | D3_STATUS_LED | D3_ALERT_LED | BUZZER );
 	 	
	
	 uart0_init();
	 uart1_init();
	 lcd_init();
	 lcd_putstring(0,"   LAND SLIDE   ");
	 lcd_putstring(1," DETECT SYSTEM ");
	
		
	
  //lcd_putstring(1," ASHOK");
 //clearData();

	//test
		
while(1){

	if(!config_mode){
	 get_deviceList();
	 lcd_putstring(0,"DEVICE SELECTION");
	 lcd_putstring(1,"                ");
	 lcd_putstring(1,device_list);
	 delay_ms(5000);
	

	if(!((IO0PIN>>14)& 0x01 )){
		loopcount =0;
		lcd_putstring(0,"SELECT DEVICE   ");
		lcd_putstring(1,"                ");
		delay(1000);
		while(loopcount < 1000){
	   lcd_putstring(1,device[press_count]);
			if( (!((IO0PIN>>14)& 0x01 )) && (loopcount<1000)){
				while((!((IO0PIN>>14)& 0x01 )) && (loopcount<1000)){
				PORT1_SET |= BUZZER;
				loopcount++;
				delay(1000);
					
				}
				PORT1_CLR |= BUZZER;
				if(loopcount<1000){
				 loopcount=0;
			   press_count++;
				}
			}
			if(press_count>2)
				press_count=0;
			
			}
		
			if(press_count==0){
				
				
				lcd_putstring(0,"DEVICE 1        ");
				lcd_putstring(1,"SELECTED        ");
			}
			else if(press_count==1){
				
				lcd_putstring(0,"DEVICE 2        ");
				lcd_putstring(1,"SELECTED        ");				
			}
			else{
				
				lcd_putstring(0,"DEVICE 3        ");
				lcd_putstring(1,"SELECTED        ");				
			}
      delay_ms(500);
		
	}
	
	delay_ms(1000);
	
	
	if (alert_detected){

		  lcd_putstring(0,"                ");
		  lcd_putstring(0,alertfrom);	
	    lcd_putstring(1,"Hazard Detected ");
		  PORT1_SET |= BUZZER;
	  }
	else{
		 lcd_putstring(0,"   No Alert!    ");
	   lcd_putstring(1,"                ");
		 PORT1_CLR |= BUZZER;
	}

	 	 delay_ms(5000);
	
	}
	
	
	if(!config_mode){
	 
	 clearData();
	
   if ((!get_alert()) && !config_mode){
	  sendChar('0');
	  sendChar(press_count + '1');
	 }
	
	 delay_ms(2000);
	 
  }
	
	if(!config_mode){
	 rainfall_ival= get_rainfall_val();	
	 lcd_putstring(0,"RAIN FALL LEVEL ");
	 lcd_putstring(1,"                ");
	 lcd_putstring(1,rainfall_val);	
	 delay_ms(3000);
	 //clearData();
	// sendChar('0');
	// sendChar('2');
	}
	if(!config_mode){
	 inclined_ival = get_inclined_val();
	 lcd_putstring(0," INCLINATION    ");
	 lcd_putstring(1,"                ");	
	 lcd_putstring(1,inclined_val);	
	 delay_ms(3000);
   }
	 if(!config_mode){
	 moisure_ival= get_moisure_val();
	 lcd_putstring(0," MOISURE LEVEL  ");
	 lcd_putstring(1,"                ");	
   lcd_putstring(1,moisure_val);	
	 delay_ms(3000);	
	 //clearData();

  	}
	 if(!config_mode){
	 power_ival= get_power_val();
	 lcd_putstring(0," POWER CONSUME  ");
	 lcd_putstring(1,"                ");	
   lcd_putstring(1,power_val);	
	 delay_ms(1000);	
	 //clearData();

  	}
	if(config_mode){
		lcd_putstring(0,"DEVICE CONGIF  ");
	  lcd_putstring(1,"                ");
		while(config_mode);
		rxbuffer0[0]='M';
		sendString(rxbuffer0);
		delay(1000);
	}

}






 return 0;
}



__irq void Uart0_ISR(void)
{


	
	
	if(rxcount0<12)
	  rxbuffer0[rxcount0++] = U0RBR;                    // RX  character
	else {
				//printString("ACK1");
	  rxcount0 =0;
	
	}
	
	if(rxbuffer0[0]=='M')
		config_mode=1;
		else{
			//printString("ACK0");
			config_mode=0;
	    rxcount0 =0;
		}
		VICVectAddr = 0x0;
     // Acknowledge that ISR has finished execution
}


__irq void Uart1_ISR(void)
{


	if(rxcount1<18)
	  rxbuffer1[rxcount1++] = U1RBR;                    // RX  character
	else 
	rxcount1 =0;
    VICVectAddr = 0x0; // Acknowledge that ISR has finished execution
}

void clearData(){

 int i;
 for(i=0;i<18;i++){
  //rxbuffer0[i]='0';
	rxbuffer1[i]='0';
 }

// rxcount0 =0;
  // rxcount1 =0;
}
int get_rainfall_val(){
   	 
	 rainfall_val[0]=rxbuffer1[3];
	 rainfall_val[1]=rxbuffer1[4];
	 rainfall_val[2]=rxbuffer1[5];
	 rainfall_val[3]='%';
	
	return 0;
	
	
}

 int get_inclined_val(){
	 
	 inclined_val[0]=rxbuffer1[7];
	 inclined_val[1]=rxbuffer1[8];
	 inclined_val[2]=rxbuffer1[9];
	 inclined_val[3]='\xDF';
	 
	 return 0;
}

  int get_moisure_val(){
	 
	 moisure_val[0]=rxbuffer1[11];
	 moisure_val[1]=rxbuffer1[12];
	 moisure_val[2]=rxbuffer1[13];
	 moisure_val[3]='%';
		return 0;
}
	
 int get_power_val(){
	 
	 power_val[0]=rxbuffer1[15];
	 power_val[1]=rxbuffer1[16];
	 power_val[2]=rxbuffer1[17];
	 power_val[3]='m';
	 power_val[4]='W';
	 
	 return 0;
}

int get_alert(){
	
	if(rxbuffer1[2]=='~'){
		return 1;
	}
	else{
	
	return 0;
}
	
}

int get_alertdeviceID(){
	int id=0;
	id= rxbuffer1[0] - '0';
	id = (id<<8) | (rxbuffer1[1]-'0');
	
	return id;
}

void get_deviceList(){
	int id,i;
	alert_detected=0;
	for(i=0;i<3;i++){
	
   clearData();
	  sendChar('0');
	  sendChar(i + '1');
		delay_ms(2000); 
		printString(rxbuffer1);
		if(get_alert()){
		alertfrom[i*3 +6]= 'D';
		alertfrom[i*3 +7]= i + '1';
		alertfrom[i*3 +8]= ' ';
		alert_detected++;
		if(i==0)
			PORT1_SET |= D1_ALERT_LED;
		else if(i==1)
			PORT1_SET |= D2_ALERT_LED;
		else
			PORT1_SET |= D3_ALERT_LED;
		}
		else{
		alertfrom[i*3 +6]= 'D';
		alertfrom[i*3 +7]= '-';
		alertfrom[i*3 +8]= ' ';
		if(i==0)
			PORT1_CLR |= D1_ALERT_LED;
		else if(i==1)
			PORT1_CLR |= D2_ALERT_LED;
		else
			PORT1_CLR |= D3_ALERT_LED;
		
		}
		
		id = get_alertdeviceID();
		
		
		if(id>0){
		device_list[i*4 ] = 'D';
		device_list[i*4 +1] = rxbuffer1[0];
		device_list[i*4 +2] = rxbuffer1[1];
		device_list[i*4 +3] = ' ';
		if(i==0)
			PORT1_SET |= D1_STATUS_LED;
		else if(i==1)
			PORT1_SET |= D2_STATUS_LED;
		else
			PORT1_SET |= D3_STATUS_LED;
	  
		device_list[13] = '0' + press_count + 1;
		}
		else{
		device_list[i*4] = 'D';
		device_list[i*4 +1] = '-';
		device_list[i*4 +2] = '-';
		device_list[i*4 +3] = ' ';		
    if(i==0)
			PORT1_CLR |= D1_STATUS_LED;
		else if(i==1)
			PORT1_CLR |= D2_STATUS_LED;
		else
			PORT1_CLR |= D3_STATUS_LED;			
		}
		
	}
	}
	



void lcd_init(){

	LCD_CTRL_DIR |= ( LCDEN | LCDRS | LCDRW );
 	LCD_CTRL_CLR |= ( LCDEN | LCDRS | LCDRW );	
 	LCD_DATA_DIR |= LCD_DATA_MASK;
	lcd_command_write(0x28);  
	lcd_command_write(0x01);  
 	lcd_command_write(0x02);  
	lcd_command_write(0x06);  
	lcd_command_write(0x0C);  
}


void lcd_command_write( unsigned char command )
{
  unsigned char temp=0;
  unsigned int temp1=0;

  temp=command;
  temp=(temp>>4)&0x0F;                             // for msb byte
  temp1=(temp<<10)&LCD_DATA_MASK;

  LCD_CTRL_CLR = LCDRS;
  LCD_CTRL_SET = LCDEN;
  LCD_DATA_CLR = LCD_DATA_MASK;
  LCD_DATA_SET = temp1;
  delay(10000);
  LCD_CTRL_CLR = LCDEN;

  temp=command;
  temp&=0x0F;                                      // for lsb byte
  temp1=(temp<<10)&LCD_DATA_MASK;
  delay(100*2);

  LCD_CTRL_CLR |= LCDRS;
  LCD_CTRL_SET |= LCDEN;
  LCD_DATA_CLR  = LCD_DATA_MASK;
  LCD_DATA_SET  = temp1;
  delay(10000);	
  LCD_CTRL_CLR |= LCDEN;
  wait_lcd();
}

void lcd_data_write( unsigned char data )
{
  unsigned char temp=0;
  unsigned int temp1=0;

  temp=data;
  temp=(temp>>4)&0x0F;                           // for msb byte
  temp1=(temp<<10)&LCD_DATA_MASK;

  LCD_CTRL_SET |= LCDEN|LCDRS;
  LCD_DATA_CLR = LCD_DATA_MASK;						 
  LCD_DATA_SET = temp1;
  LCD_CTRL_CLR |= LCDEN;

  temp=data;
  temp&=0x0F;                                   // for lsb byte
  temp1=(temp<<10)&LCD_DATA_MASK;

  LCD_CTRL_SET |= LCDEN|LCDRS;
  LCD_DATA_CLR  = LCD_DATA_MASK;
  LCD_DATA_SET  = temp1;
  LCD_CTRL_CLR |= LCDEN;
  wait_lcd();
}
 
void wait_lcd( void )
{
  LCD_CTRL_CLR |=  LCDRS;	
  LCD_CTRL_SET |=  LCDRW |LCDEN;
  while(IO1PIN & LCD_BUSY_FLAG);		         //wait for busy flag to become low  
  LCD_CTRL_CLR |= LCDEN | LCDRW;
  LCD_DATA_DIR |= LCD_DATA_MASK;
  delay(100);  
}


void lcd_putstring( unsigned char line, char *string )
{
  unsigned char len = MAX_CHAR_IN_ONE_LINE;
  
  if( line == 0 )
		lcd_command_write( 0x80  );		/* command - position cursor at 0x00 (0x80 + 0x00 ) */
  else if( line==1 )
		lcd_command_write( 0xC0  );		/* command - position cursor at 0x40 (0x80 + 0x00 ) */
 
  while(*string != '\0' && len--)
  {
    lcd_data_write( *string );
    string++;
  }
}

void uart0_init()
{

    //Assign P0.0 as TXD and P0.1 as RXD
    PINSEL0 |= ((1<<0) | (1<<2));
    //UART0 Line Control Register, used to configure Serial Port Parameters
    U0LCR = 0x83;    //8-bit Data, No Parity and 1-Stop Bit

    //U0DLL and U0DLM Values to be loaded, to get Baud Rate of 9600bps
    U0DLL = 0X61;
    U0DLM = 0X00;

    U0LCR = 0X03;  
 	U0IER = 0x1; // to keep things simple just enable "Receive Data Available(RDA)" interrupt

 
VICVectAddr1 = (unsigned)Uart0_ISR; //Pointer Interrupt Function (ISR)


VICVectCntl1 = 0x20 | 6;

VICIntEnable |= (1<<6); //Enable Uart0 interrupt , 6th bit=1

}

void uart1_init()
{

    //Assign P0.8 as RXD and P0.9 as TXD
    PINSEL0 |= 0x00050000;
    //UART0 Line Control Register, used to configure Serial Port Parameters
    U1LCR = 0x83;    //8-bit Data, No Parity and 1-Stop Bit

    //U0DLL and U0DLM Values to be loaded, to get Baud Rate of 9600bps
    U1DLL = 0X61;
    U1DLM = 0X00;
    U1LCR = 0X03;  
 	U1IER = 0x1; // to keep things simple just enable "Receive Data Available(RDA)" interrupt

 
	VICVectAddr2 = (unsigned)Uart1_ISR; //Pointer Interrupt Function (ISR)


	VICVectCntl2 = 0x20 | 7;

	VICIntEnable |= (1<<7); //Enable Uart1 interrupt , 7th bit=1

}

void sendChar(char c){


     while ( !(U1LSR & 0x20 ) ); // wait till the THR is empty
     // now we can write to the Tx FIFO
     U1THR = c;


}

void sendString(char *s){


	while(*s!='\0'){
	sendChar(*s++);
	}


}

void printChar(char c){


     while ( !(U0LSR & 0x20 ) ); // wait till the THR is empty
     // now we can write to the Tx FIFO
     U0THR = c;


}

void printString(char *s){


	while(*s!='\0'){
	printChar(*s++);
	}


}

void delay(int count)
{
  int j=0,i=0;

  for(j=0;j<count;j++)
  {
    /* At 60Mhz, the below loop introduces
    delay of 10 us */
    for(i=0;i<35;i++);

  }
}

void delay_ms(unsigned int count)
{
  unsigned int j=0,i=0;

  for(j=0;j<count;j++)
  {
    /* At 60Mhz, the below loop introduces
    delay of 1000 us */
    for(i=0;i<3500;i++);

  }
}

