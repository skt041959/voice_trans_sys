#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#ifndef _NRF24L01_H
#define _NRF24L01_H

#define PORT1 1
#define PORT2 2

//===================================
#define PORT1_SET_CE  GPIOC->BSRR = GPIO_Pin_4
#define PORT1_CLR_CE  GPIOC->BRR = GPIO_Pin_4

#define PORT2_SET_CE  GPIOA->BSRR = GPIO_Pin_2
#define PORT2_CLR_CE  GPIOA->BRR = GPIO_Pin_2

#define CSN_DESEL      GPIOA->BSRR = GPIO_Pin_3;GPIOC->BSRR = GPIO_Pin_5
#define SEL_CSN_PORT1  GPIOC->BRR = GPIO_Pin_5
#define SEL_CSN_PORT2  GPIOA->BRR = GPIO_Pin_3

#define PORT1_IRQ GPIOB->IDR & GPIO_Pin_0
#define PORT2_IRQ GPIOA->IDR & GPIO_Pin_1
//===================================

// SPI(nRF24L01) commands
#define READ_REG_NRF24L01    	0x00 				// Define read command to register
#define WRITE_REG_NRF24L01   	0x20 				// Define write command to register
#define RD_RX_PLOAD 			0x61 				// Define RX payload register address
#define WR_TX_PLOAD 			0xA0 				// Define TX payload register address
#define FLUSH_TX    			0xE1 				// Define flush TX register command
#define FLUSH_RX    			0xE2 				// Define flush RX register command
#define REUSE_TX_PL 			0xE3 				// Define reuse TX payload register command
#define NOP         			0xFF 				// Define No Operation, might be used to read status register
//***************************************************//
// SPI(nRF24L01) registers(addresses)
#define CONFIG      			0x00				// 'Config' register address
#define EN_AA       			0x01               	// 'Enable Auto Acknowledgment' register address
#define EN_RXADDR   			0x02               	// 'Enabled RX addresses' register address
#define SETUP_AW    			0x03               	// 'Setup address width' register address
#define SETUP_RETR  			0x04               	// 'Setup Auto. Retrans' register address
#define RF_CH       			0x05               	// 'RF channel' register address
#define RF_SETUP    			0x06 				// 'RF setup' register address
#define STATUS      			0x07 				// 'Status' register address
#define OBSERVE_TX  			0x08 				// 'Observe TX' register address
#define CD          			0x09 				// 'Carrier Detect' register address
#define RX_ADDR_P0  			0x0A				// 'RX address pipe0' register address
#define RX_ADDR_P1  			0x0B 				// 'RX address pipe1' register address
#define RX_ADDR_P2  			0x0C 				// 'RX address pipe2' register address
#define RX_ADDR_P3  			0x0D 				// 'RX address pipe3' register address
#define RX_ADDR_P4  			0x0E 				// 'RX address pipe4' register address
#define RX_ADDR_P5  			0x0F				// 'RX address pipe5' register address
#define TX_ADDR     			0x10 				// 'TX address' register address
#define RX_PW_P0    			0x11 				// 'RX payload width, pipe0' register address
#define RX_PW_P1    			0x12 				// 'RX payload width, pipe1' register address
#define RX_PW_P2    			0x13 				// 'RX payload width, pipe2' register address
#define RX_PW_P3    			0x14 				// 'RX payload width, pipe3' register address
#define RX_PW_P4    			0x15 				// 'RX payload width, pipe4' register address
#define RX_PW_P5    			0x16 				// 'RX payload width, pipe5' register address
#define FIFO_STATUS 			0x17 			   	// 'FIFO Status Register' register address

#define TX_ADR_WIDTH   	5  // 5字节宽度的发送/接收地址
#define TX_PLOAD_WIDTH 	32  // 数据通道有效数据宽度

#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10

u8 SPI_WRR(u8 port, u8 reg,u8 value);
u8 SPI_RDR(u8 port, u8 reg);
u8 SPI_Read_Buf(u8 port, u8 reg,u8 *pBuf,u8 bytes);
u8 SPI_Write_Buf(u8 port, u8 reg,u8 *pBuf,u8 bytes);

void Config_Send_PORT();
void Config_Receive_PORT();

void nRF24L01_Initial();
void Repower_NRF24L01();
void PORT1_Send(u8 *);
void PORT2_Send(u8 *);
void PORT1_Receive();
void PORT2_Receive();

#endif /*_NRF24L01_H*/

