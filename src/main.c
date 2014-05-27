#include "stm32f10x.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

#include "NRF24L01.h"
#include "oled.h"
#include "spi_flash.h"

extern u8 RX_BUF[];
u8 TX_BUF[TX_PLOAD_WIDTH];
u8 sample_values[2][32];
u8 buffer_index = 0;
u8 buffer_index2 = 0;
u8 buffer_full = 0;

extern u8 TX_ADDRESS_0[];
extern u8 TX_ADDRESS_1[];
extern u8 TX_ADDRESS_2[];

extern __IO uint8_t Receive_Buffer[64];
extern __IO uint32_t Receive_length;
extern __IO uint32_t length;
uint8_t Send_Buffer[64];
uint32_t packet_sent = 1;
uint32_t packet_receive = 1;
typedef enum {
    center = 3,
    storage = 2,
    terminal = 1,
    trunk = 0,
}work_mode;

void Delay (uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    //NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);

    //NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
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

void Sampling_Config()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    //GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    //GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    //GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TimeBaseStruct.TIM_Period=125;
    //FIXME:fix the period
    TIM_TimeBaseStruct.TIM_Prescaler= (72 - 1);
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0x10;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);

    DMA_InitTypeDef DMA_InitStruct;
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&ADC1->DR + 1;
    DMA_InitStruct.DMA_MemoryBaseAddr = (u32)sample_values;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize = 32;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    ADC_InitTypeDef ADC_InitStruct;
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
    ADC_InitStruct.ADC_NbrOfChannel = 1;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_Init(ADC1, &ADC_InitStruct);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_1Cycles5);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_ExternalTrigConvCmd(ADC1,ENABLE);
}

void switch_config()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void LED_Config()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

#ifdef DEBUG
void USART_Config()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

    USART_InitTypeDef USART_InitStruct;
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate=115200;
    USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    USART_Init(USART1,&USART_InitStruct);
    USART_Cmd(USART1,ENABLE);
}
#endif

u8 PORT1_SENT=1, PORT2_SENT=1;
u8 PORT_RECEIVE = 0;

u8 i=0;
work_mode mode;
void (*main_thread)();

void center_main();
void storage_main();
void terminal_main();
void trunk_main();
void terminal_detect();
void center_detect(u8);
u8 virtual_com_send();
u8 spi_flash_store();

int main(void)
{
    SystemInit();
    //SysTick_Config(SystemCoreClock/1000);

    TIM3_Config();
    TIM3->CNT = 1000;
    TIM_Cmd(TIM3, ENABLE);
    while(TIM3->CNT);
    TIM_Cmd(TIM3, DISABLE);

#ifndef DEBUG
    LED_Config();
    switch_config();
    mode = (uint16_t)(GPIOD->IDR & (GPIO_Pin_14 | GPIO_Pin_15)) >> 14;
    //switch(mode)
    //{
    //    case center:main_thread = &center_main;
    //                break;
    //    case storage:main_thread = &storage_main;
    //                break;
    //    case terminal:main_thread = &terminal_main;
    //                break;
    //    case trunk:main_thread = &trunk_main;
    //                break;
    //}

    main_thread = &terminal_main;
    main_thread();
#endif

#ifdef DEBUG
    //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    //Set_System();
    //Set_USBClock();
    //USB_Interrupts_Config();
    //USB_Init();

    /*NVIC_Config();*/
    /*Sampling_Config();*/
    /*DMA_Cmd(DMA1_Channel1, ENABLE);*/
    /*TIM_Cmd(TIM2, ENABLE);*/
    USART_Config();
    nRF24L01_Initial();

    LED_Config();
    //GPIO_Pin_10 green
    //GPIO_Pin_11 red

    Config_Send_PORT();
    //Config_Receive_PORT();
    TX_BUF[0]=0x55;
    u8 ADDR[5];
    u8 tmp;
    SPI_Read_Buf(PORT1, READ_REG_NRF24L01 + RX_ADDR_P0, ADDR, 5);
    tmp = SPI_RDR(PORT1, READ_REG_NRF24L01 + RX_PW_P0);
    //SPI_Read_Buf(PORT1, READ_REG_NRF24L01 + RX_ADDR_P1, ADDR, 5);
    //tmp = SPI_RDR(PORT1, READ_REG_NRF24L01 + RX_PW_P1);

    for(i=0; i<5; i++)
    {
        USART1->DR = ADDR[i];
        while((USART1->SR & USART_FLAG_TXE) == (uint16_t)RESET);
    }


    while(1)
    {
        PORT1_Send(TX_BUF);

        //PORT1_Receive();
    }
#endif

    while(1);
}

void terminal_main()
{
    NVIC_Config();
    Sampling_Config();

    //u8 index = 1;
    //if( (uint16_t)(GPIOD->IDR & (GPIO_Pin_12 | GPIO_Pin_13)) == 0x1000 )
    //    index = 1;
    //else if( (uint16_t)(GPIOD->IDR & (GPIO_Pin_12 | GPIO_Pin_13)) == 0x2000 )
    //    index = 2;

    nRF24L01_Initial();

    Config_Send_PORT();
    //center_detect(index);

    //u8 ADDR[5];

    //SPI_Read_Buf(PORT2, READ_REG_NRF24L01 + RX_ADDR_P0, ADDR, 5);

    DMA_Cmd(DMA1_Channel1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    while(1)
    {
        if(buffer_full)
        {
            buffer_full = 0;
            PORT1_Send((u8 *)(sample_values + (buffer_index2++)%2*32));
            if( (buffer_index - buffer_index) > 1)
            {
                //TODO
            }
        }
    }
}

void center_main()
{
    u8 (*process_packge)();

    //if( GPIOD->IDR & GPIO_Pin_12  )
    //{
        //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
        //Set_System();
        //Set_USBClock();
        //USB_Interrupts_Config();
        //USB_Init();
        //process_packge = &virtual_com_send;
    //}
    //else
        //process_packge = &spi_flash_store;
    process_packge = &virtual_com_send;

    Send_Buffer[0] = 0xFF;
    Send_Buffer[34] = 0xFF;
    Send_Buffer[35] = 0x00;

    //NVIC_Config();
    nRF24L01_Initial();

    //u8 ADDR[5];
    //u8 rpd;
    u8 status;
    u8 * rx_buffer = Send_Buffer + 2;
    Config_Receive_PORT();
    terminal_detect();

    //SPI_Read_Buf(PORT2, READ_REG_NRF24L01 + RX_ADDR_P0, ADDR, 5);

    while(1)
    {
        //rpd = SPI_RDR(PORT2, READ_REG_NRF24L01 + CD);
        while(PORT1_IRQ);

        status = SPI_RDR(PORT1, STATUS);
        if(status & 0x40)
        {
            Send_Buffer[1] = (status & 0x0E) >> 1;
            SPI_Read_Buf(PORT1, RD_RX_PLOAD, rx_buffer, TX_PLOAD_WIDTH);

            SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x40);
        }
        process_packge();
    }
}

void trunk_main()
{
}

void terminal_detect()
{
    u8 status = 0x00;
    u8 terminals = 0;
    while( terminals < 2 )
    {
        while(PORT1_IRQ);
        PORT1_CLR_CE;

        status=SPI_RDR(PORT1, STATUS);
        if(status & 0x40)
        {
            if( (status & 0x0E) == 0x00 )
            {
                SPI_Read_Buf(PORT1, RD_RX_PLOAD, RX_BUF, TX_PLOAD_WIDTH);
                SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + RX_ADDR_P0 + terminals, RX_BUF, TX_ADR_WIDTH);
                SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RX_PW_P1, TX_PLOAD_WIDTH);
                terminals ++;
            }
            SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x40);
        }
        PORT1_SET_CE;
    }
}

void center_detect(u8 index)
{
    u8 status=0x00;
    u8 detected = 0;
    u8 * addr;
    if(index == 1)
        addr = TX_ADDRESS_1;
    else if(index == 2)
        addr = TX_ADDRESS_2;
    while(!detected)
    {
        SPI_Write_Buf(PORT1, WR_TX_PLOAD, addr, 5);

        GPIOD->BRR = GPIO_Pin_10 | GPIO_Pin_11;

        PORT1_SET_CE;
        while(PORT1_IRQ);
        PORT1_CLR_CE;

        status = SPI_RDR(PORT1, STATUS);
        if(status & TX_DS)	/*tx_ds == 0x20*/
        {
            detected = 1;
            SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x20);
            SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + TX_ADDR, addr, TX_ADR_WIDTH);
        }
        else if(status & MAX_RT)
        {
            SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x10);
            GPIOD->BSRR = GPIO_Pin_11;
        }
    }
}

void storage_main()
{
}

u8 virtual_com_send()
{
}

u8 spi_flash_store()
{
}

void DMA1_Channel1_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_IT_TC1) == SET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        //TIM_Cmd(TIM2, DISABLE);
        DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR1_EN);
        DMA1_Channel1->CNDTR = 0x20;
        DMA1_Channel1->CMAR = (u32)(sample_values+(++buffer_index)%2*32);
        DMA1_Channel1->CCR |= DMA_CCR1_EN;
        buffer_full = 1;
    }
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

