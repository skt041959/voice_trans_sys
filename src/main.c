#include "stm32f10x.h"
#include "NRF24L01.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include <stdio.h>

extern u8 RX_BUF[];
extern u8 TX_BUF[];

extern __IO uint8_t Receive_Buffer[64];
extern __IO uint32_t Receive_length;
extern __IO uint32_t length;
uint8_t Send_Buffer[64];
uint32_t packet_sent = 1;
uint32_t packet_receive = 1;

u8 PORT1_SENT=1, PORT2_SENT=1;
u8 PORT_RECEIVE = 0;

u8 i=0;

void Delay (uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM3_Config() 
{
    u16 PrescalerValue = 7200 - 1;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    /* Compute the prescaler value */
    /* Time base configuration */
    TIM_TimeBaseStruct.TIM_Period = 10; /*ARR*/
    TIM_TimeBaseStruct.TIM_Prescaler = PrescalerValue; /*RSC*/
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Down;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    //TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);
    //TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
}

int main(void)
{
    SystemInit();

    //SysTick_Config(SystemCoreClock/1000);

    TIM3_Config();
    TIM3->CNT = 100;
    TIM_Cmd(TIM3, ENABLE);
    while(TIM3->CNT);
    TIM_Cmd(TIM3, DISABLE);

#ifdef SELF
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    nRF24L01_Initial();

    //GPIO_InitTypeDef GPIO_InitStruct;
    //GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    //GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    //GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_Init(GPIOB, &GPIO_InitStruct);	

    Config_Send_PORT();
    Config_Receive_PORT();

    TX_BUF[0]=0x55;

    while(1)
    {
        //GPIOB->BSRR = GPIO_Pin_2;
        PORT1_Send(TX_BUF);
        //GPIOB->BRR = GPIO_Pin_2;

        PORT2_Receive();
    }
#endif

#ifdef TRANSMITTOR
    NVIC_Config();
    nRF24L01_Initial();

    Config_Send_PORT();

    for(i=0; i<10; i++)
    {
        TX_BUF[i]=0x30+i;
    }
    u8 ADDR[5];

    PORT1_SENT = 1;
    PORT2_SENT = 1;
    SPI_Read_Buf(PORT2, READ_REG_NRF24L01 + RX_ADDR_P0, ADDR, 5);

    while(1)
    {
        //if(PORT1_SENT && PORT2_SENT)
        if(PORT1_SENT)
        {
            //PORT2_Send(TX_BUF);
            //PORT2_SENT = 0;

            //TIM3->CNT = 1;
            //TIM_Cmd(TIM3, ENABLE);
            //while(TIM3->CNT);
            //TIM_Cmd(TIM3, DISABLE);

            PORT1_Send(TX_BUF);
            PORT1_SENT = 0;
        }
    }
#endif

#ifdef RECEIVER
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    NVIC_Config();
    nRF24L01_Initial();

    Config_Receive_PORT();
    u8 ADDR[5];
    u8 rpd;
    u8 rf_ch;
    SPI_Read_Buf(PORT2, READ_REG_NRF24L01 + RX_ADDR_P0, ADDR, 5);
    rf_ch = SPI_RDR(PORT2, READ_REG_NRF24L01 + RF_CH);

    while(1)
    {
        rpd = SPI_RDR(PORT2, READ_REG_NRF24L01 + CD);
        if(PORT_RECEIVE)
        {
            //CDC_Send_DATA(RX_BUF, TX_PLOAD_WIDTH);
            PORT_RECEIVE = 0;
        }
    }
#endif
    while(1);
}

void EXTI0_IRQHandler(void)
{
    u8 status = 0x00;
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
        //while(PORT1_IRQ);
#ifdef TRANSMITTOR
        PORT1_CLR_CE;

        status = SPI_RDR(PORT1, STATUS);
        if(status & TX_DS)	/*tx_ds == 0x20*/
        {
            PORT1_SENT = 1;
            SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x20);
        }
        else if(status & MAX_RT)
        {
            PORT1_SENT = 1;
            SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x10);
        }
#endif

#ifdef RECEIVER
        status=SPI_RDR(PORT2, STATUS);
        if(status & 0x40)
        {
            PORT_RECEIVE = 1;
            SPI_Read_Buf(PORT2, RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);
            SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x40);
        }  
    //PORT2_SET_CE;
#endif
    }
}

#ifdef TRANSMITTOR
void EXTI1_IRQHandler(void)
{
    u8 status = 0x00;
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);
        //while(PORT2_IRQ);
        PORT2_CLR_CE;

        status = SPI_RDR(PORT2, STATUS);
        if(status & TX_DS)	/*tx_ds == 0x20*/
        {
            PORT2_SENT = 1;
            SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x20);
        }
        else if(status & MAX_RT)
        {
            PORT2_SENT = 1;
            SPI_WRR(PORT2, WRITE_REG_NRF24L01 + STATUS, 0x10);
        }
	}
}
#endif

