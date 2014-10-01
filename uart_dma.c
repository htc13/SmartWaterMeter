#include "stm32f10x.h"
#include "uart_dma.h"
#include <string.h>

 char txBuffer[MAX_DMA_TX_BUFFER];
 char rxBuffer[MAX_DMA_RX_BUFFER];

void uartConfig(void)
{
	//USART3 DMA mode (PB10 (TX),PB11 (RX)) - 5V tolerant I/O
	// DMA TX (MCU to PC): DMA1_Channel2
	// DMA RX (PC to MCU): DMA1_Channel3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // USART3 clock enable
	RCC->AHBENR |= 0x00000001; // DMA1 clock enable
	/* Configure the Priority Group to 2 bits */
	SCB->AIRCR = 0x05FA0000 | 0x00000500; // NVIC_PriorityGroup_2 => see programming manuel (PM0056) page 134
	/* Enable the USART3 TX DMA Interrupt */
	NVIC->IP[DMA1_Channel2_IRQn]=0x0; // DMA1 Channel 2 global interrupt see RM page 203; DMA1_Channel2_IRQn=12 (see stm32f10x.h file)
	/* Enable the Selected IRQ Channels --------------------------------------*/
	NVIC->ISER[DMA1_Channel2_IRQn>>5]=(uint32_t)0x01 << (DMA1_Channel2_IRQn & (uint8_t)0x1F);
	/* Enable the USART3 RX DMA Interrupt */
	NVIC->IP[DMA1_Channel3_IRQn]=0x0; // DMA1 Channel 3 global interrupt see RM page 204; DMA1_Channel3_IRQn=13
	/* Enable the Selected IRQ Channels --------------------------------------*/
	NVIC->ISER[DMA1_Channel3_IRQn>>5]=(uint32_t)0x01 << (DMA1_Channel3_IRQn & (uint8_t)0x1F);
	/* GPIO configuration */
	AFIO->MAPR &= 0xFFFFFFCF; // No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) - RM p181
	GPIOB->CRH &= 0xFFFF00FF; // Clear PB10 & PB11
	GPIOB->CRH |= 0x00000B00; // USART3 TX (PB10) alternate output push-pull, 50MHz
	GPIOB->CRH |= 0x00004000; // USART3 RX (PB11) input floating
	/* UART3 Configuration */
	USART3->CR1 = 0x00000000; // reset the register
	USART3->CR2 = 0x00000000; // 1 stop bit
	USART3->CR3 = 0x00000000; // HW control disabled
	
	USART3->CR1 |= 0x00002000; // Enable USART3
	USART3->CR1 |= 0x00000000; // 1 Start bit, 8 Data bits, 1 Stop bit, parity control disabled, all USART3 interrupts disabled	
	USART3->CR1 |= 0x0000000C; // Enable USART3 transmitter & reciever
	USART3->BRR = (SystemCoreClock / 9600); // set baudrate (both DMA tx & rx)
}

void uartDmaTxInit(void)
{
	// configure the TX channel
	DMA1_Channel2->CCR |= 0x00000010; // direction: memory to peripheral (RM page 287) - read from memory (and transfer to peropheral)
	DMA1_Channel2->CMAR = (unsigned int)&txBuffer;
	DMA1_Channel2->CNDTR = sizeof(txBuffer)-1;
	DMA1_Channel2->CPAR = (unsigned int)&USART3->DR; // peripheral base address
	DMA1_Channel2->CCR &= ~0x00000040; // disable peripheral increment
	DMA1_Channel2->CCR |= 0x00000080; // enable memory increment
	DMA1_Channel2->CCR &= ~0x00000300; // Periphereal size: 8 bits (byte)
	DMA1_Channel2->CCR &= ~0x00000C00; // Memory size: 8 bits (byte)
	DMA1_Channel2->CCR &= ~0x00000020; // DMA circular mode disabled (no auto start)
	DMA1_Channel2->CCR |= 0x00002000; // channel priority: High
	DMA1_Channel2->CCR &= ~0x00004000; // mem 2 mem disabled
	/* Enable the USART Tx DMA request */
	USART3->CR3 |= 0x00000080; // Enable DMA transmitter
	/* Enable DMA Stream Transfer Complete interrupt */
	DMA1_Channel2->CCR |= 0x00000002;
	/* Enable the DMA TX Channel */
	//DMA1_Channel2->CCR |= 0x00000001; // Start the transfer (from txBuffer)...
}
void uartDmaRxInit(void)
{
	// configure the RX channel: DMA1, channel 3
	DMA1_Channel3->CCR &= ~0x00000010; // direction: peripheral to memory (RM page 287) - read from peripheral (and transfer to mem)
	DMA1_Channel3->CMAR = (unsigned int)&rxBuffer;
	DMA1_Channel3->CNDTR = sizeof(rxBuffer);//-1;
	DMA1_Channel3->CPAR = (unsigned int)&USART3->DR; // peripheral base address
	DMA1_Channel3->CCR &= ~0x00000040; // disable peripheral increment
	DMA1_Channel3->CCR |= 0x00000080; // enable memory increment
	DMA1_Channel3->CCR &= ~0x00000300; // Periphereal size: 8 bits (byte)
	DMA1_Channel3->CCR &= ~0x00000C00; // Memory size: 8 bits (byte)
	DMA1_Channel3->CCR |= 0x00000020; // DMA circular mode enabled
	DMA1_Channel3->CCR |= 0x00002000; // channel priority: High
	DMA1_Channel3->CCR &= ~0x00004000; // mem 2 mem disabled
	/* Enable the USART Rx DMA request */
	USART3->CR3 |= 0x0000040; // Enable DMA reciever
	/* Enable DMA Stream Transfer Complete interrupt */
	DMA1_Channel3->CCR |= 0x00000002;
	/* Enable the DMA RX Channel */
	DMA1_Channel3->CCR |= 0x00000001;
}

void uartTxDeInit(void)
{
	// de-init the TX channel
	DMA1_Channel2->CCR &= 0xFFFFFFFE; // disable the channel (RM page 286)
	DMA1_Channel2->CCR = 0;
	DMA1_Channel2->CNDTR = 0;
	DMA1_Channel2->CPAR = 0;
	DMA1_Channel2->CMAR = 0;
	DMA1->IFCR &= 0xFFFFFF0F; // reset all channel2 pending interrupt bits
}

void uartRxDeInit(void)
{
	// de-init the RX channel
	DMA1_Channel3->CCR &= 0xFFFFFFFE; // disable the channel (RM page 286)
	DMA1_Channel3->CCR = 0;
	DMA1_Channel3->CNDTR = 0;
	DMA1_Channel3->CPAR = 0;
	DMA1_Channel3->CMAR = 0;
	DMA1->IFCR &= 0xFFFFF0FF; // reset all channel3 pending interrupt bits
}
void resetTxBuffer(void)
{
	memset(txBuffer,0x0A,sizeof(txBuffer));
}
void resetRxBuffer(void)
{
	memset(rxBuffer,0x00,sizeof(rxBuffer));
}
void loadTxBuffer( char const *str)
{
	strcpy(txBuffer,str);
}
void setTxBuffer(char data, char index)
{
	txBuffer[index]=data;
}
char* rxBufferCmp( char const *str)
{
	return strstr(rxBuffer,str);
}
char* rxBufferFindChr(char c)
{
	return strchr(rxBuffer,c);
}
unsigned char getRxByteNumer(unsigned char n)
{
	return rxBuffer[n];
}
void storeRxData(char rxData,unsigned int uart_rx_cnt)
{
	rxBuffer[uart_rx_cnt]=rxData;
}


