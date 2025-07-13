//smart_parking

//Naor David 206493546
//Saar Moreno 209015643
//Itay Kassos 206963027


#include <stdio.h>
#include <string.h>
#include "NUC1xx.h"
#include "DrvGPIO.h"
#include "DrvSYS.h"
#include "LCD_Driver.h"
#include "NUC1xx-LB_002\LCD_Driver.h"
#include "Driver\DrvUART.h"
#include "Driver_PWM_Servo.h"
#include "Driver\DrvSPI.h"
#include "SPI_RC522.h"


// Global definition
#define	_SR04A_ECHO		   (GPB_2)			
#define	_SR04A_TRIG		   (GPB_4)			
#define	_SR04A_TRIG_Low	 (GPB_4=0)
#define	_SR04A_TRIG_High (GPB_4=1)

#define	_SR04B_ECHO		   (GPB_3)			
#define	_SR04B_TRIG		   (GPB_5)	
#define	_SR04B_TRIG_Low	 (GPB_5=0)
#define	_SR04B_TRIG_High (GPB_5=1)

#define HITIME_MIN 70  
#define HITIME_MAX 140 

#define MAX_PARKING_SPOTS 3


// Global variables
volatile uint32_t SR04A_Echo_Width = 0;
volatile uint32_t SR04A_Echo_Flag  = FALSE;
volatile uint32_t SR04B_Echo_Width = 0;
volatile uint32_t SR04B_Echo_Flag  = FALSE;


// Initial TMR2
// Timer Clock:	12 MHz
// Prescale:		11
// Compare:		0xffffff
// Mode:			One-Shot mode
// Capture:		Enable, Capture with Falling Edge
void Init_TMR2(void)
{	
	//Step 1: T2EX pin Enable (PB.2, Pin34)
	SYS->GPBMFP.UART0_nRTS_nWRL = 1;
	SYS->ALTMFP.PB2_T2EX = 1;

    //Step 2: Timer Controller Reset and Setting Clock Source
	SYS->IPRSTC2.TMR2_RST = 1;          //Timer Controller: Reset
	SYS->IPRSTC2.TMR2_RST = 0;          //Timer Controller: Normal
	SYSCLK->CLKSEL1.TMR2_S = 0;	        //Timer Clock = 12 MHz
	SYSCLK->APBCLK.TMR2_EN = 1;         //Timer C lock Enable

	//Step 3: Timer Controller Setting
	//  TMR0_T = (12 MHz / (11+1) / 1000000)^-1 = 1.000 Second
	TIMER2->TCMPR = 0xffffff;           //Timer Compare Value:  [0~16777215]
	TIMER2->TCSR.PRESCALE = 11;         //Timer Prescale:       [0~255]
	TIMER2->TCSR.MODE = 0;              //Timer Operation Mode: One-Shot

	//Step 4: External Capture Mode Setting
	TIMER2->TEXCON.TEXEN = 1;	          //External Capture Function Enable
	TIMER2->TEXCON.RSTCAPSEL = 0;	      //Capture Mode Select: Capture Mode
	TIMER2->TEXCON.TEX_EDGE = 2;	      //Capture Edge: Rising & Falling

	//Step 5: Timer Interrupt Setting
//	TIMER2->TCSR.IE = 1;				      //Timeout Interrupt Enable
//	TIMER2->u32TISR |= 0x01;		      //Clear Timeout Flag (TIF)
	TIMER2->TEXCON.TEXIEN = 1;		      //Capture Interrupt Enable
	TIMER2->u32TEXISR |= 0x01;		      //Clear Capture Flag (TEXIF)
	NVIC_EnableIRQ(TMR2_IRQn);		      //Timer NVIC IRQ Enable

	//Step 6: Start Timer Capture (Set by Ultrasonic_Trigger() Function)
// 	TIMER2->TCSR.CRST = 1;			      //Timer Counter Reset
// 	TIMER2->TCSR.CEN = 1;				      //Timer Start
}

// TMR2 Interrupt Handler
void TMR2_IRQHandler(void)
{
	TIMER2->TEXCON.RSTCAPSEL = 0;       // set back for falling edge to capture
	TIMER2->TCSR.CEN = 1;					      //Timer Start

	if(TIMER2->TEXISR.TEXIF == 1)	      //Capture Flag (TEXIF)
	{
	 	TIMER2->u32TEXISR |= 0x01;				//Clear Capture Flag (TEXIF)
		SR04A_Echo_Width = TIMER2->TCAP;	//Load Capture Value (Unit: us)
		SR04A_Echo_Flag  = TRUE;
	}
}


// Initial TMR3
// Timer Clock:	12 MHz
// Prescale:		11
// Compare:		0xffffff
// Mode:			One-Shot mode
// Capture:		Enable, Capture with Falling Edge
void Init_TMR3(void)
{	
	//Step 1: T3EX pin Enable (PB.3, Pin35)
	SYS->GPBMFP.UART0_nCTS_nWRH = 1;
	SYS->ALTMFP.PB3_T3EX = 1;

    //Step 2: Timer Controller Reset and Setting Clock Source
	SYS->IPRSTC2.TMR3_RST = 1;            //Timer Controller: Reset
	SYS->IPRSTC2.TMR3_RST = 0;            //Timer Controller: Normal
	SYSCLK->CLKSEL1.TMR3_S = 0;	          //Timer Clock = 12 MHz
	SYSCLK->APBCLK.TMR3_EN = 1;           //Timer C lock Enable

	//Step 3: Timer Controller Setting
	//  TMR3_T = (12 MHz / (11+1) / 1000000)^-1 = 1.000 Second
	TIMER3->TCMPR = 0xffffff;             //Timer Compare Value:  [0~16777215]
	TIMER3->TCSR.PRESCALE = 11;           //Timer Prescale:       [0~255]
	TIMER3->TCSR.MODE = 0;                //Timer Operation Mode: One-Shot

	//Step 4: External Capture Mode Setting
	TIMER3->TEXCON.TEXEN = 1;	            //External Capture Function Enable
	TIMER3->TEXCON.RSTCAPSEL = 0;	        //Capture Mode Select: Capture Mode
	TIMER3->TEXCON.TEX_EDGE = 2;	        //Capture Edge: Rising & Falling

	//Step 5: Timer Interrupt Setting
//	TIMER3->TCSR.IE = 1;				        //Timeout Interrupt Enable
//	TIMER3->u32TISR |= 0x01;		        //Clear Timeout Flag (TIF)
	TIMER3->TEXCON.TEXIEN = 1;		        //Capture Interrupt Enable
	TIMER3->u32TEXISR |= 0x01;		        //Clear Capture Flag (TEXIF)
	NVIC_EnableIRQ(TMR3_IRQn);		        //Timer NVIC IRQ Enable

	//Step 6: Start Timer Capture (Set by Ultrasonic_Trigger() Function)
// 	TIMER3->TCSR.CRST = 1;			        //Timer Counter Reset
// 	TIMER3->TCSR.CEN = 1;				        //Timer Start
}


// TMR3 Interrupt Handler
void TMR3_IRQHandler(void)
{
	TIMER3->TEXCON.RSTCAPSEL = 0;         // set back for falling edge to capture
	TIMER3->TCSR.CEN = 1;					        //Timer Start
	
	if(TIMER3->TEXISR.TEXIF == 1)         //Capture Flag (TEXIF)
	{
	 	TIMER3->u32TEXISR |= 0x01;					//Clear Capture Flag (TEXIF)
		SR04B_Echo_Width = TIMER3->TCAP;		//Load Capture Value (Unit: us)
		SR04B_Echo_Flag  = TRUE;
	}
}


// Ultrasonic Trigger
void SR04_Trigger(void)
{
	//Trigger of Ultrasonic Sensor
	_SR04A_TRIG_High;
  _SR04B_TRIG_High;	
	DrvSYS_Delay(10);							// 10us for TRIG width
	_SR04A_TRIG_Low;
	_SR04B_TRIG_Low;
	
  TIMER2->TEXCON.RSTCAPSEL = 1; // set for rising edge trigger to reset counter
	TIMER3->TEXCON.RSTCAPSEL = 1; 
}


void Init_GPIO_SR04(void)
{
	//Ultrasonic I/O Pins Initial
	GPIOB->PMD.PMD2 = 0;							//_SR04_ECHO pin, Input
	GPIOB->PMD.PMD3 = 0;							
	GPIOB->PMD.PMD4 = 1;              //_SR04_TRIG pin, Output
	GPIOB->PMD.PMD5 = 1;
  _SR04A_TRIG_Low;                  // set Trig output to Low
	_SR04B_TRIG_Low;	
}



void Init_LED(void) 
{
	// initialize GPIO pins
	DrvGPIO_Open(E_GPA, 13, E_IO_OUTPUT); // GPA13 pin set to output mode
	DrvGPIO_Open(E_GPA, 14, E_IO_OUTPUT); // GPA14 pin set to output mode
	// set GPIO pins output Hi to disable LEDs
	DrvGPIO_SetBit(E_GPA, 13); // GPA13 pin output Hi to turn off Green LED
	DrvGPIO_SetBit(E_GPA, 14); // GPA14 pin output Hi to turn off Red   LED
}

void led_open_gate(void) 
{
		//green led
		DrvGPIO_ClrBit(E_GPA, 13); // GPA13 = Green, 0 : on, 1 : off
		DrvGPIO_SetBit(E_GPA, 14);
}

void led_close_gate(void) 
{
 	  //red led
		DrvGPIO_SetBit(E_GPA, 13);
		DrvGPIO_ClrBit(E_GPA, 14); // GPA14 = Red,   0 : on, 1 : off
}


/////------pwm servo--------
uint8_t hitime;
void open_gate(void)
{
   for (hitime=HITIME_MIN; hitime<=HITIME_MAX; hitime++)
	{
		PWM_Servo(0, hitime);	
		DrvSYS_Delay(100000);
	}
}


void close_gate(void)
{
   for (hitime=HITIME_MAX; hitime>=HITIME_MIN; hitime--) 
	{
		PWM_Servo(0, hitime);	
		DrvSYS_Delay(100000);
	}
}



/////------UART--------

char TEXT[16];                   
volatile uint8_t comRbuf[9];    // UART receive buffer
volatile uint8_t comRbytes = 0; // Byte counter for UART input
uint8_t uart_flag = 0;         // Set when a full UART message is received
uint8_t uart_pay_flag = 0;     // Set when "pay" command is received via UART
uint8_t sensor_active_flag = 0;// Block input during active operation
	
void UART_INT_HANDLE(void)
{
	while(UART0->ISR.RDA_IF==1) 
	{
		comRbuf[comRbytes]=UART0->DATA;
		comRbytes++;		
		if (comRbytes==3) 
		{	
			sprintf(TEXT,"%s",comRbuf);
      print_lcd(1,TEXT);			
		  comRbytes=0;
			uart_flag = 1;
		}
	}
}



/////------RFID--------
unsigned char UID[4],Temp[4];    // UID storage for scanned card
unsigned char RF_Buffer[18];     // General buffer for RFID data
unsigned char Password_Buffer[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}   ; 
int counter = 0;         // Retry counter for card reading
int read_flag = 0;       // Set when UID is successfully read
int ok_flag = 0;         // Set when UID matches known cards

unsigned char card1[4] = {0xc3, 0xe3, 0xa7, 0xaa}; 
unsigned char card2[4] =  {0x57, 0xb5, 0x9a, 0x2e}; 
unsigned char card3[4] = {0x6c, 0xa8, 0xb4, 0x38}; 
unsigned char card4[4] = {0x3c, 0xb3, 0x26, 0x31}; 

void Init_SPI()
{
	DrvSPI_Open(eDRVSPI_PORT1, eDRVSPI_MASTER, eDRVSPI_TYPE1, 8);
	DrvSPI_SetEndian(eDRVSPI_PORT1, eDRVSPI_MSB_FIRST);
	DrvSPI_DisableAutoSS(eDRVSPI_PORT1);
	DrvSPI_SetClockFreq(eDRVSPI_PORT1, 50000, 0); // set SPI clock = 50KHz
}


// RFID Card Reader Function
void Reader(void)
{

	if(PcdRequest(0x52,Temp)==MI_OK)  // Send request to detect RFID tag
    {
      if(PcdAnticoll(UID)==MI_OK)
      { 
				read_flag = 1;
				counter = 0;
			}
    }
		else 
		{
        counter++;
        if (counter >= 3)
        {
            counter = 0;
        }
    }		 
}


// Compare two UID arrays
int compare_uid(unsigned char *uid1, unsigned char *uid2)
{
    int i;
    for (i = 0; i < 4; i++) {
        if (uid1[i] != uid2[i])
            return 0;// UID mismatch
    }
    return 1;// UID match
}



//------------------------------
// MAIN function
//------------------------------
int main (void)
{
	
	STR_UART_T sParam;
	uint8_t isCarA_Detected = 0;
  uint8_t isCarB_Detected = 0;
  int car_count = 0;
	char sendBuf1[19];
	char     TEXT2[17] = "SR04a: ";
	char     TEXT3[17] = "SR04b: ";
	char     TEXT4[25] = " ";
	uint32_t distance_mm;
	
	//System Clock Initial
	UNLOCKREG();
	DrvSYS_SetOscCtrl(E_SYS_XTL12M, ENABLE);
	while(DrvSYS_GetChipClockSourceStatus(E_SYS_XTL12M) == 0);
	DrvSYS_Open(50000000);
	LOCKREG();

	DrvSYS_SetClockDivider(E_SYS_HCLK_DIV, 0); /* HCLK clock frequency = HCLK clock source / (HCLK_N + 1) */

	DrvGPIO_InitFunction(E_FUNC_UART0);	// Set UART pins
	/* UART Setting */
	sParam.u32BaudRate 		  = 9600;
	sParam.u8cDataBits 		  = DRVUART_DATABITS_8;
	sParam.u8cStopBits 		  = DRVUART_STOPBITS_1;
	sParam.u8cParity 		    = DRVUART_PARITY_NONE;
	sParam.u8cRxTriggerLevel= DRVUART_FIFO_1BYTES;
  DrvUART_Open(UART_PORT0, &sParam);
	DrvUART_EnableInt(UART_PORT0, DRVUART_RDAINT, UART_INT_HANDLE);
	
	//------SPI----------
	DrvGPIO_InitFunction(E_FUNC_SPI1);
	Init_SPI();
	PcdReset();
	PcdAntennaOn();
	
	Initial_panel();                  // initialize LCD
	clr_all_panel();                  // clear LCD display                       
	Init_TMR2();                      // initialize Timer2 Capture
	Init_TMR3();                      // initialize Timer3 Capture
  Init_LED();                       // initialize LEDS 
	InitPWM(0);                       // Initialize PWM channel 0 for servo control
  
	
	while(1) 
	{
	
		if (uart_flag ) 
		{
			uart_flag = 0;
			if (strcmp(TEXT, "opn") == 0 && sensor_active_flag == 0) 
			{
				  Init_LED();
					led_open_gate();          
				  open_gate();  //  open gate
					DrvSYS_Delay(1000000);
			}
			
		 if (strcmp(TEXT, "clo") == 0 && sensor_active_flag == 0) 
			{
				  Init_LED();
					led_close_gate();
				  close_gate();   // Close gate
					DrvSYS_Delay(1000000);
			}
			
		 if (strcmp(TEXT, "pay") == 0) 
			{
				  uart_pay_flag = 1;	
				  clr_all_panel();
				  DrvSYS_Delay(3000000);
			}
		}
			
		SR04_Trigger();             // Trigger Ultrasound Sensor for 10us   		
		DrvSYS_Delay(40000);        // Wait 40ms for Echo to trigger interrupt
		Reader();                   // Try to read RFID tag and set read_flag if successful
		
		if (read_flag && ok_flag == 0)  // Check if a valid UID was read
    {
      if (compare_uid(UID, card1) || compare_uid(UID, card2) || compare_uid(UID, card3) || compare_uid(UID, card4))
      {
        ok_flag = 1; // UID is valid and recognized
      }
    read_flag = 0;   // Reset flag after processing
   }
			
		
		if( SR04A_Echo_Flag==TRUE)   // If echo received from entrance sensor (SR04A)
		{
			SR04A_Echo_Flag = FALSE;			
			distance_mm = SR04A_Echo_Width * (340/2) / 1000;  // Calculate distance
			sprintf(TEXT2+6, " %d mm  ", distance_mm);	
			print_lcd(2, TEXT2);	    //Line 2: distance [mm]
			
			   // If a car is detected close to gate, not already detected,
         // space is available and RFID was verified
				if (distance_mm < 70 && isCarA_Detected == 0 && car_count < MAX_PARKING_SPOTS && ok_flag == 1 ) 
				{
					sensor_active_flag = 1;   // Block other activity UART
					isCarA_Detected = 1;       // Mark car as being processed
					led_open_gate();              // Turn on green LED
					DrvSYS_Delay(1000000);
					open_gate();                //  open the gate
					DrvSYS_Delay(10000000); // 10 second delay while gate is open	
					
				}
				
				if (distance_mm >= 140 && isCarA_Detected == 1 )   // If the car has passed through (no longer close)
				{
					led_close_gate();  // Turn on red LED
					DrvSYS_Delay(1000000);
					close_gate();       // Close the gate
					car_count++;         // Increase car count 
					isCarA_Detected = 0;	   // Reset car detection state
					sensor_active_flag = 0;  
					ok_flag = 0;         // Require new RFID scan
				}
	  }
		
	  if (SR04B_Echo_Flag==TRUE)
		{
			SR04B_Echo_Flag = FALSE;			
			distance_mm = SR04B_Echo_Width * (340/2) / 1000;
			sprintf(TEXT3+6, " %d mm  ", distance_mm);	
			print_lcd(3, TEXT3);	    //Line 3: distance [mm]
			
		     	// Check if a car is at the exit:
         // close enough (< 60 mm) , hasn't already been processed
         //  parking lot is not empty and payment confirmed (uart_pay_flag == 1)
				if (distance_mm < 70 &&  isCarB_Detected == 0 && car_count > 0 &&  uart_pay_flag == 1  ) 
				{
					sensor_active_flag = 1;  // Block other activity UART
					isCarB_Detected = 1;  // Mark car as detected at exit
				  uart_pay_flag = 0;	 // Clear payment flag
					led_open_gate();     // Turn on green LED
					DrvSYS_Delay(1000000);
					open_gate();         //  open gate
					DrvSYS_Delay(10000000); // 10 second delay while gate is open
					
				}
				if (distance_mm >= 140 && isCarB_Detected == 1 ) 
				{
					led_close_gate();  // Turn on red LED
					DrvSYS_Delay(1000000);
					close_gate();      // Close gate
					car_count--;         // Decrement number of cars in parking
					isCarB_Detected = 0; // Reset car detection state
					sensor_active_flag = 0;
				}
		}	
     
		// Format a string showing the number of available parking spots
		 sprintf(sendBuf1, "Available Parking: %d\n", 3 - car_count);
		
		// Send the formatted string via UART to the connected device Android
     DrvUART_Write(UART_PORT0, (uint8_t*)sendBuf1, strlen(sendBuf1));
		  DrvSYS_Delay(1000000); 
		
		 sprintf(TEXT4, "parking: %d", 3 - car_count);
     print_lcd(0, TEXT4);
		 DrvSYS_Delay(1000000); 
		
		 sprintf(TEXT4, "Cars: %d", car_count);
     print_lcd(1, TEXT4);
		 DrvSYS_Delay(1000000);           
 }
}

