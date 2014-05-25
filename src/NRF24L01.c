#include "stm32f10x.h"
#include "NRF24L01.h"
#include <stdio.h>
#include <string.h>

u8 TX_ADDRESS_0[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};  // 定义一个静态发送地址
u8 TX_ADDRESS_1[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x02};  // 定义一个静态发送地址

u8 RX_BUF[TX_PLOAD_WIDTH];

u8 TX_BUF[TX_PLOAD_WIDTH];

static void Initial_SPI(SPI_TypeDef* SPIx)  //初始化IOB端口
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;
    if(SPIx==SPI1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(SPIx==SPI2)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_Init(GPIOB, &GPIO_InitStruct);
    }

    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx, &SPI_InitStruct);

    SPI_Cmd(SPIx, ENABLE);
}

static void SPI_Send_byte(SPI_TypeDef* SPIx,u8 data)
{
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE)==RESET);
    SPI_I2S_SendData(SPIx,data);

    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE)==RESET);
    SPI_I2S_ReceiveData(SPIx);
}

static u8 SPI_Receive_byte(SPI_TypeDef* SPIx,u8 data)
{
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE)==RESET);
    SPI_I2S_SendData(SPIx,data);

    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE)==RESET);
    return SPI_I2S_ReceiveData(SPIx);
}

static void delay1us(u8 t)
{
    while(--t);
} 

/****向寄存器reg写一个字节，同时返回状态字节**************/
u8 SPI_WRR(u8 port, u8 reg,u8 value)
{
    u8 status;
    if(port == PORT1)
    {
        SEL_CSN_PORT1;
    }
    else if(port == PORT2)
    {
        SEL_CSN_PORT2;
    }
    status=SPI_Receive_byte(SPI1,reg);   //select register  and write value to it
    SPI_Send_byte(SPI1,value);   
    CSN_DESEL;
    return(status); 
}

/****向寄存器reg读一个字节，同时返回状态字节**************/
u8 SPI_RDR(u8 port, u8 reg)
{
    u8 status;
    if(port == PORT1)
    {
        SEL_CSN_PORT1;
    }
    else if(port == PORT2)
    {
        SEL_CSN_PORT2;
    }
    SPI_Send_byte(SPI1,reg);
    status=SPI_Receive_byte(SPI1,0);   //select register  and write value to it
    CSN_DESEL;
    return(status);
}

/********读出bytes字节的数据*************************/
u8 SPI_Read_Buf(u8 port, u8 reg,u8 *pBuf,u8 bytes)
{
    u8 status,byte_ctr;
    if(port == PORT1)
    {
        SEL_CSN_PORT1;
    }
    else if(port == PORT2)
    {
        SEL_CSN_PORT2;
    }
    status=SPI_Receive_byte(SPI1,reg);       
    for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
        pBuf[byte_ctr]=SPI_Receive_byte(SPI1,0);
    CSN_DESEL;
    return(status);
}

/****************写入bytes字节的数据*******************/
u8 SPI_Write_Buf(u8 port, u8 reg,u8 *pBuf,u8 bytes)
{
    u8 status,byte_ctr;
    if(port == PORT1)
    {
        SEL_CSN_PORT1;
    }
    else if(port == PORT2)
    {
        SEL_CSN_PORT2;
    }
    status=SPI_Receive_byte(SPI1,reg); 
    delay1us(1);
    for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
        SPI_Send_byte(SPI1,*pBuf++);
    CSN_DESEL;
    return(status);
}

/*接收函数，返回1表示有数据收到，否则没有数据接收到**/
/*
 *u8 nRF24L01_RxPacket(u8 port, u8* rx_buf)
 *{
 *    u8 status,revale=0;
 *    if(port == PORT1)
 *        PORT1_CLR_CE;
 *    else if(port == PORT2)
 *        PORT2_CLR_CE;
 *
 *    delay1us(10);
 *    status=SPI_Receive_byte(SPI1,STATUS);	// 读取状态寄存其来判断数据接收状况
 *    //	CE(0);
 *    //	status=0x40;
 *    //printf("STATUS接受状态：0x%2x\r\n",status);
 *
 *    if(status & RX_DR)				// 判断是否接收到数据
 *    {
 *        //		CE(1);
 *        SPI_Read_Buf(port, RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
 *        //		CE(0);
 *        revale =1;			//读取数据完成标志
 *    }
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + STATUS,status);   //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
 *    if(port == PORT1)
 *        PORT1_SET_CE;
 *    else if(port == PORT2)
 *        PORT2_SET_CE;
 *    return revale;	
 *}
 */

/****************发送函数***************************/
/*
 *void nRF24L01_TxPacket(u8 port, u8 * tx_buf)
 *{
 *    if(port == PORT1)
 *        PORT1_CLR_CE;
 *    else if(port == PORT2)
 *        PORT2_CLR_CE;
 *    SPI_Write_Buf(port, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
 *    SPI_Write_Buf(port, WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据	
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + CONFIG, 0x0e);   		 // IRQ收发完成中断响应，16位CRC，主发送
 *    if(port == PORT1)
 *        PORT1_SET_CE;
 *    else if(port == PORT2)
 *        PORT2_SET_CE;
 *    delay1us(10);
 *}
 */

/*
 *void RX_Mode(u8 port)
 *{
 *    if(port == PORT1)
 *    {
 *        PORT1_CLR_CE;
 *        SPI_Write_Buf(port, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
 *    }
 *    else if(port == PORT2)
 *    {
 *        PORT2_CLR_CE;
 *        SPI_Write_Buf(port, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
 *    }
 *
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度 
 *
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + EN_AA, 0x3f);               // 使能接收通道0自动应答
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // 使能接收通道0
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + RF_CH, 40);                 // 选择射频通道0x40
 *
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + CONFIG, 0x0f);              // CRC使能，16位CRC校验，上电，接收模式
 *
 *    if(port == PORT1)
 *        PORT1_SET_CE;
 *    else if(port == PORT2)
 *        PORT2_SET_CE;
 *}
 *
 *void TX_Mode(u8 port, u8 * tx_buf)
 *{
 *    if(port == PORT1)
 *    {
 *        PORT1_CLR_CE;
 *        SPI_Write_Buf(port, WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS_0, TX_ADR_WIDTH);     // 写入发送地址
 *        SPI_Write_Buf(port, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道0地址和发送地址相同
 *    }
 *    else if(port == PORT2)
 *    {
 *        PORT2_CLR_CE;
 *        SPI_Write_Buf(port, WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS_1, TX_ADR_WIDTH);     // 写入发送地址
 *        SPI_Write_Buf(port, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_1, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道1地址和发送地址相同
 *    }
 *
 *    SPI_Write_Buf(port, WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);  // 装载数据
 *
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + EN_AA, 0x3f);       // 使能接收通道0自动应答
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // 使能接收通道0
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // 自动重发延时等待250us+86us，自动重发10次
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + RF_CH, 40);         // 选择射频通道0x40
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度
 *    SPI_WRR(port, WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // CRC使能，16位CRC校验，上电
 *
 *    if(port == PORT1)
 *        PORT1_SET_CE;
 *    else if(port == PORT2)
 *        PORT2_SET_CE;
 *    delay1us(10);
 *}
 */

void nRF24L01_Initial()
{
#if defined TRANSMITTOR || defined SELF
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA ,ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    /*CE Initial*/

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);	

    /*IRQ Initial*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);	

    /*CSN Initial*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);	

    Initial_SPI(SPI1);
#endif

#ifdef TRANSMITTOR
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    EXTI_InitTypeDef   EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
#endif

#ifdef RECEIVER
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA ,ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);	

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);	

    Initial_SPI(SPI1);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

    EXTI_InitTypeDef   EXTI_InitStructure;

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
#endif
}

#if defined TRANSMITTOR || defined SELF
void Config_Send_PORT()
{
    PORT1_CLR_CE;

    SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS_0, TX_ADR_WIDTH);     // 写入发送地址
    SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道0地址和发送地址相同

    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_AA, 0x3f);       // 使能接收通道0自动应答
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // 使能接收通道0
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // 自动重发延时等待250us+86us，自动重发10次
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_CH, 0x40);         // 选择射频通道0x40
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x7f);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // CRC使能，16位CRC校验，上电

#ifdef TRANSMITTOR
    PORT2_CLR_CE;

    SPI_Write_Buf(PORT2, WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS_1, TX_ADR_WIDTH);     // 写入发送地址
    SPI_Write_Buf(PORT2, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_1, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道1地址和发送地址相同

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_AA, 0x3f);       // 使能接收通道0自动应答
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // 使能接收通道0
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // 自动重发延时等待250us+86us，自动重发10次
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_CH, 0x40);         // 选择射频通道0x40
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x7f);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // CRC使能，16位CRC校验，上电
#endif

    delay1us(10);
}
#endif

void Config_Receive_PORT()
{
#ifdef SELF
    PORT2_CLR_CE;

    SPI_Write_Buf(PORT2, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度 

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_AA, 0x3f);               // 使能接收通道0自动应答
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // 使能接收通道0
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_CH, 0x40);                 // 选择射频通道0x40

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x7f);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + CONFIG, 0x0f);              // CRC使能，16位CRC校验，上电，接收模式
    PORT2_SET_CE;
#endif

#ifdef RECEIVER
    PORT2_CLR_CE;

    SPI_Write_Buf(PORT2, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度 

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_AA, 0x3f);               // 使能接收通道0自动应答
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // 使能接收通道0
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_CH, 0x40);                 // 选择射频通道0x40

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x7f);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + CONFIG, 0x0f);              // CRC使能，16位CRC校验，上电，接收模式
    PORT2_SET_CE;
#endif
}

/*
 *void NRF24L01_Send(u8 port)
 *{
 *    u8 status=0x00;
 *    //	CSN(0);
 *    TX_Mode(port, TX_BUF);
 *
 *    while(PORT1_IRQ);
 *
 *    if(port == PORT1)
 *        PORT1_CLR_CE;
 *    else if(port == PORT2)
 *        PORT2_CLR_CE;
 *
 *    delay1us(10);
 *    status = SPI_RDR(port, STATUS);	// 读取状态寄存其来判断数据接收状况
 *    if(status & TX_DS)	[>tx_ds == 0x20<]
 *    {
 *        //printf("STATUS接受状态：0x%2x\r\n",status);
 *        //printf("\r\n发送完数据：%s\r\n",RX_BUF);	
 *        SPI_WRR(port, WRITE_REG_NRF24L01 + STATUS, 0x20);      // 清除TX，让IRQ拉低；
 *    }
 *    else if(status & MAX_RT)
 *    {
 *        //printf("发送达到最大发送次数");	
 *        SPI_WRR(port, WRITE_REG_NRF24L01 + STATUS, 0x10);      // 清除TX，让IRQ拉低；			
 *    }
 *
 *    if(port == PORT1)
 *        PORT1_SET_CE;
 *    else if(port == PORT2)
 *        PORT2_SET_CE;
 *    //	status=20;
 *}
 *
 *void NRF24L01_Receive(u8 port)
 *{
 *    u8 status=0x01;  
 *    //Initial_SPI(SPI1);
 *    //RX_Mode(port);
 *    while(PORT2_IRQ);
 *    //	printf("产生中断\n");
 *    if(port == PORT1)
 *        PORT1_CLR_CE;
 *    else if(port == PORT2)
 *        PORT2_CLR_CE;
 *    delay1us(10);
 *    status=SPI_RDR(port, STATUS);	// 读取状态寄存其来判断数据接收状况
 *    //printf("STATUS接受状态：0x%2x\r\n",status);
 *    if(status & 0x40)			//接受中断标志位
 *    {
 *        //printf("接受成功");
 *        //		CE(1);
 *        SPI_Read_Buf(port, RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
 *
 *        //printf("\r\n i=%d,接收到数据：%x\r\n",i,RX_BUF[0]);
 *        //		for(i=0;i<32;i++)
 *        //		{
 *        //			RX_BUF[1] = 0X06;
 *        //			printf("\r\n i=%d,接收到数据：%x\r\n",i,RX_BUF[i]);
 *        //		}
 *        SPI_WRR(port, WRITE_REG_NRF24L01 + STATUS, 0x40);      // 清除TX，让IRQ拉低
 *    }  
 *    if(port == PORT1)
 *        PORT1_SET_CE;
 *    else if(port == PORT2)
 *        PORT2_SET_CE;
 *}
 */

#if defined TRANSMITTOR || defined SELF
void PORT1_Send(u8 * tx_buf)
{
    u8 status=0x00;

    SPI_Write_Buf(PORT1, WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);

#ifdef SELF
    PORT1_SET_CE;
    while(PORT1_IRQ);
    PORT1_CLR_CE;

    status = SPI_RDR(PORT1, STATUS);
    if(status & TX_DS)	/*tx_ds == 0x20*/
    {
        SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x20);
    }
    else if(status & MAX_RT)
    {
        SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x10);
    }
#else
#ifdef TRANSMITTOR
    PORT1_SET_CE;
#endif
#endif
}
#endif

#ifdef TRANSMITTOR
void PORT2_Send(u8 * tx_buf)
{
    SPI_Write_Buf(PORT2, WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);  // 装载数据

    PORT2_SET_CE;
}
#endif

#if defined RECEIVER || defined SELF
void PORT2_Receive()
{
#ifdef SELF
    u8 status=0x00;  
    while(PORT2_IRQ);

    status=SPI_RDR(PORT2, STATUS);
    if(status & 0x40)
    {
        SPI_Read_Buf(PORT2, RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);

        SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x40);
    }  
    PORT2_SET_CE;
#endif
}
#endif
