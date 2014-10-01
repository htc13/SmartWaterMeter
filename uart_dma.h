#ifndef UART_DMA_H
#define UART_DMA_H

#define MAX_DMA_TX_BUFFER 64
#define MAX_DMA_RX_BUFFER 1024

void uartConfig(void);
void uartDmaTxInit(void);
void uartDmaRxInit(void);
void uartTxDeInit(void);
void uartRxDeInit(void);
void resetTxBuffer(void);
void resetRxBuffer(void);
void loadTxBuffer( char const *str);
void setTxBuffer(char data, char index);
char* rxBufferCmp( char const *str);
char* rxBufferFindChr(char c);
unsigned char getRxByteNumer(unsigned char n);
void storeRxData(char rxData,unsigned int uart_rx_cnt);
#endif
