#include "stm32f10x.h"
#include "NRF24L01.h"

u8 TX_ADDRESS_0[TX_ADR_WIDTH] = {0xb2, 0xb2, 0xb3, 0xb4, 0x01};
u8 TX_ADDRESS_1[TX_ADR_WIDTH] = {0xc2, 0xc2, 0xc3, 0xc4, 0x02};
u8 TX_ADDRESS_2[TX_ADR_WIDTH] = {0xc3, 0xc2, 0xc3, 0xc4, 0x02};

u8 RX_BUF[TX_PLOAD_WIDTH];

static void Initial_NRF24L01_SPI()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &GPIO_InitStruct);

    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStruct);

    SPI_Cmd(SPI1, ENABLE);
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

u8 SPI_WRR(u8 port, u8 reg,u8 value)
{
    u8 status;
    SEL_CSN_PORT1;
    status=SPI_Receive_byte(SPI1,reg);   //select register  and write value to it
    SPI_Send_byte(SPI1,value);
    CSN_DESEL;
    return(status);
}

u8 SPI_RDR(u8 port, u8 reg)
{
    u8 status;
    SEL_CSN_PORT1;
    SPI_Send_byte(SPI1,reg);
    status=SPI_Receive_byte(SPI1,0);   //select register  and write value to it
    CSN_DESEL;
    return(status);
}

u8 SPI_Read_Buf(u8 port, u8 reg, u8 *pBuf,u8 bytes)
{
    u8 status,byte_ctr;
    SEL_CSN_PORT1;

    status=SPI_Receive_byte(SPI1,reg);
    for(byte_ctr=0; byte_ctr<bytes; byte_ctr++)
        pBuf[byte_ctr] = SPI_Receive_byte(SPI1,0);

    CSN_DESEL;
    return(status);
}

u8 SPI_Write_Buf(u8 port, u8 reg, u8 *pBuf,u8 bytes)
{
    u8 status,byte_ctr;
    SEL_CSN_PORT1;
    status = SPI_Receive_byte(SPI1,reg);
    //delay1us(1);

    for(byte_ctr=0; byte_ctr<bytes; byte_ctr++)
        SPI_Send_byte(SPI1,*pBuf++);

    CSN_DESEL;
    return(status);
}

void nRF24L01_Initial()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    /*CE Initial*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*IRQ Initial*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*CSN Initial*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    Initial_NRF24L01_SPI();

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

    EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

#ifdef DEBUG2
    /*CE Initial*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*IRQ Initial*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*CSN Initial*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
}

void Repower_NRF24L01()
{
    PORT1_CLR_CE;

    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + CONFIG, 0x00);
    TIM3->CNT = 20;
    TIM_Cmd(TIM3, ENABLE);
    while(TIM3->CNT);
    TIM_Cmd(TIM3, DISABLE);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + CONFIG, 0x02);
}

void Config_Send_PORT(u8 index)
{
    PORT1_CLR_CE;

    if(index == 1)
    {
        SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS_0, TX_ADR_WIDTH);
        SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);
    }
    else
    {
        SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS_1, TX_ADR_WIDTH);
        SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_1, TX_ADR_WIDTH);
    }
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);

    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_AA, 0x01);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_RXADDR, 0x01);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + SETUP_RETR, 0x05);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_CH, 0x40);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_SETUP, 0x0F);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x70);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + CONFIG, 0x0E);
    PORT1_SET_CE;

#ifdef DEBUG2
    PORT2_CLR_CE;

    SPI_Write_Buf(PORT2, WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS_1, TX_ADR_WIDTH);
    SPI_Write_Buf(PORT2, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_1, TX_ADR_WIDTH);

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_AA, 0x3f);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_CH, 0x40);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_SETUP, 0x07);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x70);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + CONFIG, 0x0e);
    PORT2_SET_CE;
#endif

    delay1us(10);
}

void Config_Receive_PORT()
{
    PORT1_CLR_CE;

    SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);
    SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + RX_ADDR_P1, TX_ADDRESS_1, TX_ADR_WIDTH);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RX_PW_P1, TX_PLOAD_WIDTH);
    //SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RX_ADDR_P2, TX_ADDRESS_2[0]);
    //SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RX_PW_P2, TX_PLOAD_WIDTH);

    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_AA, 0x03);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_RXADDR, 0x03);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_CH, 0x40);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_SETUP, 0x0F);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x70);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + CONFIG, 0x0f);
    PORT1_SET_CE;

#ifdef DEBUG2
    PORT2_CLR_CE;

    SPI_Write_Buf(PORT2, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_AA, 0x3f);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_CH, 0x40);

    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + RF_SETUP, 0x07);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x70);
    SPI_WRR(PORT2, WRITE_REG_NRF24L01 + CONFIG, 0x0f);
    PORT2_SET_CE;
#endif
}

void PORT1_Send(u8 * tx_buf)
{
    SPI_Write_Buf(PORT1, WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
    u8 status=0x00;
    GPIOD->BRR = GPIO_Pin_10 | GPIO_Pin_11;

    //PORT1_SET_CE;
    while(PORT1_IRQ);
    //PORT1_CLR_CE;

    status = SPI_RDR(PORT1, STATUS);
    if(status & TX_DS)	/*tx_ds == 0x20*/
    {
        SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x20);
        GPIOD->BSRR = GPIO_Pin_10;
    }
    else if(status & MAX_RT)
    {
        SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x10);
        GPIOD->BSRR = GPIO_Pin_11;
    }
}

void PORT1_Receive()
{
    u8 status=0x00;
    while(PORT1_IRQ);

    status=SPI_RDR(PORT1, STATUS);
    if(status & 0x40)
    {
        USART1->DR = status;
        GPIOD->BSRR = GPIO_Pin_10;
        SPI_Read_Buf(PORT1, RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);

        SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x40);
        GPIOD->BRR = GPIO_Pin_10 | GPIO_Pin_11;
    }
    //PORT1_SET_CE;
}

#if defined DEBUG2
void PORT2_Receive()
{
    u8 status=0x00;
    while(PORT2_IRQ);

    status=SPI_RDR(PORT2, STATUS);
    if(status & 0x40)
    {
        SPI_Read_Buf(PORT2, RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);

        SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x40);
    }
    //PORT2_SET_CE;
}
#endif

#ifdef DEBUG2
void PORT2_Send(u8 * tx_buf)
{
    SPI_Write_Buf(PORT2, WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);  // 装载数据
    //PORT2_SET_CE;
}
#endif

