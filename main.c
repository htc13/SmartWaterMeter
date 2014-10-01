#include "stm32f10x.h"
#include <string.h>
#include "uart_dma.h"
#include <stdlib.h>
 
//#define SERIAL_DEMO
#define USART3_DMA_DEMO
//#define VALVE_CONTROL_DEMO

// macros to turn LED2 on and off
#define LED_ON GPIOA->BSRR|=0x00000020 // set PA5
#define LED_OFF GPIOA->BSRR|=0x00200000 // reset PA5
// macros to turn on/off the GSM module
// on the GSM module the *logic inverts*
#define GSM_HIGH GPIOC->BSRR|=0x00000100 // set PC8 (goes to D9 on iComsat board)
#define GSM_LOW GPIOC->BSRR|=0x01000000 // reset PC8 (goes to D9 on iComsat board)

#define GSM_DE_ASSERT_RST GPIOC->BSRR|=0x00400000 // reset PC6
#define GSM_ASSERT_RST GPIOC->BSRR|=0x00000040 // reset PC6

volatile unsigned int msTicks=0;

__INLINE static void msDelay (uint32_t dlyTicks) {
  unsigned int curTicks = msTicks;

  while ((msTicks - curTicks) < dlyTicks);
}

void gsmTurnOn(void);
char sendATCmdWaitResp( char const *AT_cmd_string, char const *response_string,char no_of_attempts);
char sendSMS(char *number_str, char *message_str);
char checkGsmRegistration(void);
char isSMSPresent( char smsType);
char getSMS(char position, char *phone_number, char *SMS_text, int max_SMS_len);
char* bufferFindChar(char* str, char c);
#ifdef SERIAL_DEMO
unsigned int rxData=0;
#endif

/*
#ifdef USART3_DMA_DEMO
#define MAX_DMA_TX_BUFFER 32
char txBuffer[MAX_DMA_TX_BUFFER];// = "ABCDEFGHIJKLMNOPQRSTUVWXYZ 1234567890 !@#$%^&*()_+,.\r\n"; // buffer to be transmitted from MCU
#define MAX_DMA_RX_BUFFER 32
char rxBuffer[MAX_DMA_RX_BUFFER];
#endif
*/

char dmaTxComplete=0;
char phone_num[20]; // array for the phone number string
#define MAX_SMS_LEN 256
char sms_text[MAX_SMS_LEN]; // array for the SMS text string
				
enum at_resp_enum 
{
  AT_RESP_ERR_NO_RESP = -1,   // nothing received
  AT_RESP_ERR_DIF_RESP = 0,   // response_string is different from the response
  AT_RESP_OK = 1,             // response_string was included in the response

  AT_RESP_LAST_ITEM
};

enum sms_type_enum 
{
  SMS_UNREAD = 0,    // new SMS - not read yet
  SMS_READ ,   			 // already read SMS
  SMS_ALL            //all stored SMS
};

int main(void)
{
	char position;
	// to set system clock:
  // go to file: system_stm32f10x.c ; line 108
  // if all commented then sysclk=8Hz (HSI)
	SysTick_Config(SystemCoreClock/1000); // systick every 1ms
		
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Port A clock enable
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Port B clock enable (for USART3)
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Port C clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Timer 3 clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Timer 2 clock enable
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable alternate function i/o clk (it must be enabled - see RM page 201 note 1)
		
	// LED2 is on PA.5
	GPIOA->CRL &= 0xFF0FFFFF; // output mode, push pull
	GPIOA->CRL |= 0x00200000; // output mode, max speed 2MHz
	
	//GSM pwrkey is on PC8
	GPIOC->CRH &= 0xFFFFFFF0; // output mode, push pull
	GPIOC->CRH |= 0x00000002; // output mode, max speed 2MHz
	//GSM reset# is on PC6
	GPIOC->CRL &= 0xF0FFFFFF; // output mode, push pull
	GPIOC->CRL |= 0x02000000; // output mode, max speed 2MHz
	
	GSM_LOW;
#ifdef SERIAL_DEMO
	//USART3 Byte Interrupt mode (PB10 (TX),PB11 (RX)) - 5V tolerant I/O
	AFIO->MAPR &= 0xFFFFFFCF; // No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) - RM p181
	GPIOB->CRH &= 0xFFFF00FF; // Clear PB10 & PB11
	GPIOB->CRH |= 0x00000B00; // USART3 TX (PB10) alternate output push-pull
	GPIOB->CRH |= 0x00004000; // USART3 RX (PB11) input floating
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // USART3 clock enable
	
	USART3->CR1 = 0x00000000; // reset the register
	USART3->CR2 = 0x00000000; // 1 stop bit
	USART3->CR3 = 0x00000000; // HW control disabled
	
	USART3->CR1 |= 0x00002000; // Enable USART3
	USART3->CR1 |= 0x00000000; // 1 Start bit, 8 Data bits, n Stop bit, parity control disabled, all USART3 interrupts disabled	
	USART3->CR1 |= 0x0000000C; // Enale USART3 transmitter & reciever
	USART3->BRR = (SystemCoreClock / 19200); // BR=19200bps; PCLK1=8Mhz; BRR=8MHz/19,200=416 (integer division); (416)d == 0x000001A0
	
	USART3->CR1 |= 0x00000020;// Enable RXNEIE bit
	NVIC_EnableIRQ(USART3_IRQn); // Enable interrupts from USART3
#endif //SERIAL_DEMO

#ifdef USART3_DMA_DEMO
	//USART3 DMA mode (PB10 (TX),PB11 (RX)) - 5V tolerant I/O
	// DMA TX (MCU to PC): DMA1_Channel2
	// DMA RX (PC to MCU): DMA1_Channel3
	uartConfig();
	
	// USART3_TX DMA1 Channel 2
	
	// de-init the TX channel
	uartTxDeInit();
	// configure the TX channel
	uartDmaTxInit();
	
	
	// de-init the RX channel
	uartRxDeInit();
	// configure the RX channel: DMA1, channel 3
	uartDmaRxInit();
	
#endif


	// User Push Button is on PC.13
	// The push button is used to generate pulses to TIM2
	GPIOC->CRH &=0xFF0FFFFF; // RM page 167
	GPIOC->CRH |=0x00400000; // input mode; floating input
		
	AFIO->EXTICR[3]&=0xFFFFFF0F; // Map Port C.13 to EXTI13 (RM 201 fig 21 & RM 186, sec 9.4.6)
	AFIO->EXTICR[3]|=0x00000020;
	EXTI->IMR |= 0x00002000; // interrupt req from line 13 is NOT masked (RM page 202)
	EXTI->RTSR &= 0x00000000; // disable rising edge trigger for line 13
	EXTI->FTSR |= 0x00002000; // enable falling edge trigger for line 13
#ifdef VALVE_CONTROL_DEMO
	//NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable interrupt from EXT line 13 (NVIC level); same as the next line
#endif	
	//=========================================================================================================================
	// Use TIM2_CH2 to count hall effect sensor pulses (PA1)
	// Timer 2 -> External clock mode 1
	// Follow configuration steps on RM page 364
	// To demo: connect CN8.A1 to B1 (user pb, blue pb) bottom left pin (next to D4 or CN7.23); Count the user pressing the button
	//=========================================================================================================================
#ifdef VALVE_CONTROL_DEMO	
	TIM2->CCMR1 &=0xFCFF; // Clear CC2S bits
	TIM2->CCMR1 |=0x0100; // CC2 channel is configured as input, IC2 is mapped on TI2
	TIM2->CCMR1 &=0x0FFF; // IC2F=0000 -> no digital filter
	TIM2->CCER &=0xFFDF; // non-inverted: capture is done on a rising edge of IC2
	TIM2->SMCR &=0xFFF8; // clear bits SMS[2:0]  
	TIM2->SMCR |=0x0007; // External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter
	TIM2->SMCR &=0xFF8F; // clear bits TS[2:0]
	TIM2->SMCR |=0x0060; // Filtered Timer Input 2 (TI2FP2)
	TIM2->ARR = 4; // Auto reload register value is 4+1=5
	TIM2->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	TIM2->CR1 = TIM_CR1_CEN;   // Enable timer
	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt from TIM2 (NVIC level)
#endif //VALVE_CONTROL_DEMO
	//==========================================================
	// Timer 3 configuration => Generate the valve control pulse
	//==========================================================
#ifdef VALVE_CONTROL_DEMO
	// Set PC10 & PC11 as the control signals to H Bridge (AIN1-PC10 & AIN2-PC11)
	GPIOC->CRH &= 0xFFFF00FF; // output mode, push pull, max speed: 2MHz
	GPIOC->CRH |= 0x00002200;
	GPIOC->BSRR &=0xF0FFF0FF; // reset PC10 & PC11
	GPIOC->BSRR |=0x0C000000;
	
	TIM3->PSC = 7999;	         // Set prescaler to 8000 (PSC + 1) ==> 8MHz/800=1Khz
	//TIM3->ARR = 1000;	          // Auto reload value 1000ms ==> interrupt every 1 sec
	TIM3->ARR = 100;	          // Auto reload value 100ms ==> interrupt every 100ms
	TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	//TIM3->CR1 = TIM_CR1_CEN;   // Enable timer
	NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt from TIM3 (NVIC level)
#endif //VALVE_CONTROL_DEMO	
	LED_OFF;
	//USART3->DR = 0x00;
	//printf("Interrupt driven Serial I/O\r\n\r\n");

	gsmTurnOn();
	checkGsmRegistration();
	//sendSMS("6478633667", "SMS from ARM & GPRS module"); // send sms to my mobile!
	//==========
	// main loop
	//==========
	while (1) 
	{	
		position=isSMSPresent(2); // check for all sms messages (read + unread)
		if(position>0)
		{
			getSMS(position, phone_num, sms_text, MAX_SMS_LEN);
		}
		msDelay(30000); // 30 seconds delay
		//__WFI(); // enter sleep mode; any interrupt will wake us up
		//while(!(USART3->SR & 0x00000080));
		//USART3->DR= 0x55;	
		//while(!(USART3->SR & 0x00000080));
		//USART3->DR= 0xAA;		
	}
}
//=============================================================================
// TIM2 Interrupt Handler
//=============================================================================
#ifdef VALVE_CONTROL_DEMO
void TIM2_IRQHandler(void)
{
	static int cnt=0;
	
	cnt++;
	if(TIM2->SR & TIM_SR_UIF) // if UIF flag is set
  {
		TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag
		//Toggle LED2
		if(cnt%2==1){
			LED_ON;
			// Turn on valve (forward polarity)
			// PC10=AIN1='1'; PC11=AIN2='0'
			GPIOC->BSRR |=0x08000400;
		}
		else{
			LED_OFF;
			// Turn off valve (Reverse polarity)
			// PC10=AIN1='0'; PC11=AIN2='1'
			GPIOC->BSRR |=0x04000800;
		}
		TIM3->CR1 = TIM_CR1_CEN;   // Enable timer - generate the 50ms pulse		
  }
}
#endif //VALVE_CONTROL_DEMO  
//=============================================================================
// TIM3 Interrupt Handler
//=============================================================================
#ifdef VALVE_CONTROL_DEMO
void TIM3_IRQHandler(void)
{
	static int cnt=0;
	
	cnt++;
	if(TIM3->SR & TIM_SR_UIF) // if UIF flag is set
  {
		TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
		
		TIM3->CR1 &= ~TIM_CR1_CEN;   // disable timer
		//GPIOC->BSRR &=0xF0FFFFFF; // reset PC10 & PC11
		GPIOC->BSRR |=0x0C000000; // reset PC10 & PC11
		// Toggle LED2
		/*
		if(cnt%2==0){
			LED_ON;
		}
		else{
			LED_OFF;
		}
		*/
  }
}
#endif //VALVE_CONTROL_DEMO
//==================================================
// Handler for EXT interrupts pins 10 to 15
//==================================================
#if 0 // not used
void EXTI15_10_IRQHandler (void)
{
	static int pb_cnt=0;
	
	if (EXTI->PR & (1<<13)){	// check if pin 13 generated the interrupt	
		pb_cnt++;
		EXTI->PR |= 1<<13; // clear EXTI13 int bit
		if(pb_cnt%2==1){
				LED_ON;
			// Turn on valve (forward polarity)
			// PC10=AIN1='1'; PC11=AIN2='0'
			GPIOC->BSRR |=0x08000400;
			
		}
		else{
				LED_OFF;
			// Turn off valve (Reverse polarity)
			// PC10=AIN1='0'; PC11=AIN2='1'
			GPIOC->BSRR |=0x04000800;
		}
		
	}
}
#endif 
/*----------------------------------------------------------------------------
  USART3_IRQHandler
  Handles USART3 global interrupt request.
 *----------------------------------------------------------------------------*/
#ifdef SERIAL_DEMO
void USART3_IRQHandler (void) {
	
	
  static int uart_rx_cnt=0;
	static int uart_tx_cnt=0;
	
	if (USART3->SR & 0x00000020) {   // RX interrupt (read data register NOT empty)
		USART3->SR &= 0xFFFFFFDF;	   // clear interrupt	
		rxData=(USART3->DR) & 0x000001FF;
		//storeRxData(rxData,uart_rx_cnt);
		uart_rx_cnt++;
		//if(uart_rx_cnt%2==0) LED_ON;
		//else LED_OFF;

		//USART3->DR= rxData;//uart_rx_cnt;		
		
	}

	if (USART3->SR & 0x00000080) {  // TX interrupt (transmit data register empty)
		USART3->SR &= 0xFFFFFF7F;	  // clear interrupt
		uart_tx_cnt++;
		/*
		USART3->DR = 170; //uart_tx_cnt;
		if(uart_tx_cnt%200==0){
			LED_OFF;
		}
		else if(uart_tx_cnt%100==0){
			LED_ON;
		}
		*/
	}

	
}
#endif //SERIAL_DEMO

#ifdef USART3_DMA_DEMO
void DMA1_Channel2_IRQHandler(void) //USART3_DMA_TX
{
	if(DMA1->ISR & 0x00000020) // check if transfer complete for channel 2
	{
		DMA1->IFCR |= 0x00000020; // clear channel2 transfer complete flag
		dmaTxComplete=1;
	}
}

void DMA1_Channel3_IRQHandler(void) // USART3_DMA_RX
{
	static unsigned char rx_count=0;
  if(DMA1->ISR & 0x00000200) // check if transfer complete for channel 3
	{
		DMA1->IFCR |= 0x00000200; // clear channel3 transfer complete flag
		rx_count++;
		if(rx_count%2==1){
				LED_ON;			
		}
		else{
				LED_OFF;
		}
	}
}
#endif

void SysTick_Handler(void)
{
	msTicks++;
}


void gsmTurnOn(void)
{
	// 1.Turn SIM900 on
	GSM_LOW; // gsm pwrkey->1
	msDelay(1500);
	GSM_HIGH; //gsm pwrkey->0
	//GSM_DE_ASSERT_RST;
	msDelay(5000);
	GSM_LOW; // gsm pwrkey->1
	msDelay(5000);
	// 2. Establish communication with SIM900
	if(sendATCmdWaitResp("AT\r\n","OK",5)==AT_RESP_OK)
	{
		// Reset to the factory settings
     sendATCmdWaitResp("AT&F0\r\n", "OK", 5);      
     // switch off echo
     sendATCmdWaitResp("ATE0\r\n","OK", 5);
     // setup auto baud rate
     sendATCmdWaitResp("AT+IPR=0\r\n","OK", 5);
	}
	else
	{
		// cannot communicate with SIM900
	}
}
char sendATCmdWaitResp( char const *AT_cmd_string, char const *response_string,char no_of_attempts)
{
	char i;
	char retVal=AT_RESP_ERR_NO_RESP;
	
	for (i=0;i<no_of_attempts;i++)
	{
		if (i>0){
			msDelay(500);
		}
		resetTxBuffer();
		resetRxBuffer();
		// de-init the TX channel
		uartTxDeInit();
		// configure the TX channel
		uartDmaTxInit();
		loadTxBuffer(AT_cmd_string);
		DMA1_Channel2->CNDTR = strlen(AT_cmd_string);//-1;
		dmaTxComplete=0;
		// de-init the RX channel
		uartRxDeInit();
		// configure the RX channel: DMA1, channel 3
		uartDmaRxInit();
		/* Enable the DMA TX Channel */
	  DMA1_Channel2->CCR |= 0x00000001; // Start the transfer (from txBuffer)...
		while(dmaTxComplete==0);
		msDelay(3000); // wait for GSM Module reply (1sec)
		// verify respons
		
		if(rxBufferCmp(response_string)!=NULL)
		{
			retVal=AT_RESP_OK;
			break;
		}
		else
		{
			retVal=AT_RESP_ERR_DIF_RESP;
		}		
	}
	return retVal;
}
void sendATCmd(char const *AT_cmd_string)
{
		resetTxBuffer();
		resetRxBuffer();
		loadTxBuffer(AT_cmd_string);
		
		dmaTxComplete=0;
		// de-init the RX channel
		uartRxDeInit();
		// configure the RX channel: DMA1, channel 3
		uartDmaRxInit();
		uartTxDeInit();
		// configure the TX channel
		uartDmaTxInit();
		DMA1_Channel2->CNDTR = strlen(AT_cmd_string);//-1;
		/* Enable the DMA TX Channel */
	  DMA1_Channel2->CCR |= 0x00000001; // Start the transfer (from txBuffer)...
		while(dmaTxComplete==0);
		//msDelay(2000); // wait for GSM Module reply (1sec)
}
void sendByte(char data)
{
	resetTxBuffer();
	resetRxBuffer();
	
	
	dmaTxComplete=0;
	// de-init the RX channel
	uartRxDeInit();
	uartTxDeInit();
	// configure the TX channel
	uartDmaTxInit();
	setTxBuffer(data, 0);
	DMA1_Channel2->CNDTR = 1;
	/* Enable the DMA TX Channel */
	DMA1_Channel2->CCR |= 0x00000001; // Start the transfer (from txBuffer)...
	while(dmaTxComplete==0);
	//msDelay(2000); // wait for GSM Module reply (1sec)
}

char sendSMS(char *number_str, char *message_str)
{
	sendATCmd("AT+CMGS=\"");
	sendATCmd(number_str);
	//sendATCmd("\"\r");
	if(AT_RESP_OK==sendATCmdWaitResp( "\"\r", ">",1))
	{
		sendATCmd(message_str);
		sendByte(26);
		msDelay(2000); // wait for GSM Module reply
		if(rxBufferCmp("+CMGS")!=NULL)
		{
			return 0; // everything OK
		}
	}
	return 1; // problem!!!
	
}
char checkGsmRegistration(void)
{
	if(AT_RESP_OK==sendATCmdWaitResp("AT+CREG?\r\n","+CREG: 0,", 1))
	{
		sendATCmdWaitResp("AT+CMGF=1\r\n","OK", 5);
    // init SMS storage
    // Disable messages about new SMS from the GSM module 
		sendATCmdWaitResp("AT+CNMI=2,0\r\n","OK", 2);
		sendATCmdWaitResp("AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n","+CPMS:", 10);
    // select phonebook memory storage
    sendATCmdWaitResp("AT+CPBS=\"SM\"\r\n","OK", 5);
	}
}

char isSMSPresent(char smsType)
{
	char retVal=-1;
	char temp;
	char *pChar;
	
	switch (smsType) {
    case SMS_UNREAD:
      temp=sendATCmdWaitResp("AT+CMGL=\"REC UNREAD\"\r","OK",1);
      break;
    case SMS_READ:
      temp=sendATCmdWaitResp("AT+CMGL=\"REC READ\"\r","OK",1);
      break;
    case SMS_ALL:
      temp=sendATCmdWaitResp("AT+CMGL=\"ALL\"\r","OK",1);
      break;
  }
	if(temp==AT_RESP_OK)
	{
		if(rxBufferCmp("+CMGL:")!=NULL)
		{
			pChar=rxBufferFindChr(':');
			if(pChar!=NULL)
			{
				retVal = atoi(pChar+1);
			}			
		}
	}	
	msDelay(20);
	return retVal;
}
char getSMS(char position, char *phone_number, char *SMS_text, int max_SMS_len)
{
	char retVal;
	char *pChar, *pChar1;
	
	if(position<=0) return (-3);
	phone_number[0] = 0;  
	retVal=0; // still GETSMS_NO_SMS
	sendATCmd("AT+CMGR=");
	sendByte(48+position);
	if(AT_RESP_OK==sendATCmdWaitResp("\r","+CMGR",1))
	{
		if(rxBufferCmp("\"REC UNREAD\"")!=NULL)
		{
			retVal=1;//GETSMS_UNREAD_SMS
		}
		else if(rxBufferCmp("\"REC READ\"")!=NULL)
		{
			retVal=2;//GETSMS_READ_SMS
		}
		else
		{
			retVal=3;//GETSMS_OTHER_SMS
		}	
		// extract phone number
		pChar=rxBufferFindChr(',');
		pChar1=pChar+2;
		pChar=bufferFindChar(pChar1,'"');
		if (pChar != NULL) 
		{
        *pChar = 0; // end of string
        strcpy(phone_number, (char *)(pChar1));
    }
		// extract text from sms
		pChar=bufferFindChar(pChar+1,0x0A);
		if(pChar!=NULL)
		{
			pChar++;
			strcpy(SMS_text,pChar);
		}
	}
	else if(rxBufferCmp("OK")!=NULL)
	{
		retVal=0; // still GETSMS_NO_SMS
	}
	else if(rxBufferCmp("ERROR")!=NULL)
	{
		retVal=0; // still GETSMS_NO_SMS
	}
	return retVal;
}
char* bufferFindChar(char* str, char c)
{
	return strchr(str,c);
}




