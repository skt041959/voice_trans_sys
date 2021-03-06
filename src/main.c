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
u8 sample_values[96];
u8 buffer_cnt = 0;
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
    storage = 1,
    terminal = 2,
    trunk = 0,
}work_mode;

u8 sync[32] = {
    0x00,'s' ,'y' ,'n' ,'c' ,0x00,0xFF,0x00,
    0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,
    0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,
    0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00};

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

    TIM_TimeBaseStruct.TIM_Period = 0xFFFF; /*ARR*/
    TIM_TimeBaseStruct.TIM_Prescaler = PrescalerValue; /*RSC*/
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Down;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);
    //TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Single);
}

void Sampling_Config()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    //GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    //GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    //GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    TIM_TimeBaseStruct.TIM_Period=125;
    //FIXME:fix the period
    TIM_TimeBaseStruct.TIM_Prescaler= (72 - 1);
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0x10;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init(TIM2, &TIM_OCInitStruct);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);

    DMA_InitTypeDef DMA_InitStruct;
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&ADC1->DR + 1;
    DMA_InitStruct.DMA_MemoryBaseAddr = (u32)sample_values;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize = 64;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);

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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

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

u8 PORT1_SENT=1, PORT2_SENT=1;
u8 PORT_RECEIVE = 0;

u8 i=0;
work_mode mode;
void (*main_thread)();

void center_main();
void storage_main();
void terminal_main();
void trunk_main();
u8 virtual_com_send();
u8 spi_flash_store();
void handshaking_rx();
void handshaking_tx(u8);

int main(void)
{
    SystemInit();
    //SysTick_Config(SystemCoreClock/1000);
    USART_Config();
    switch_config();
    LED_Config();
    TIM3_Config();

    TIM3->CNT = 100;
    TIM_Cmd(TIM3, ENABLE);
    while(TIM3->CNT);
    TIM_Cmd(TIM3, DISABLE);

#ifdef RELEASE
    mode = (uint16_t)(GPIOD->IDR & (GPIO_Pin_12 | GPIO_Pin_13)) >> 12;
    switch(mode)
    {
        case center:main_thread = &center_main;
                    break;
        case storage:main_thread = &storage_main;
                    break;
        case terminal:main_thread = &terminal_main;
                    break;
        case trunk:main_thread = &trunk_main;
                    break;
    }

    main_thread();
#endif

#ifdef DEBUG_SM
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    /*NVIC_Config();*/
    Sampling_Config();
    USART_Config();

    DMA_Cmd(DMA1_Channel1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    while(1)
    {
        GPIOA->BRR = GPIO_Pin_4;
        if( (DMA1->ISR & DMA1_IT_HT1) != (uint32_t)RESET )
        {
            GPIOA->BSRR = GPIO_Pin_4;
            DMA1->IFCR = DMA1_IT_HT1;

            //PORT1_Send((u8 *)(sample_values));
            CDC_Send_DATA(sample_values, 32);
        }
        if( (DMA1->ISR & DMA1_IT_TC1) != (uint32_t)RESET )
        {
            GPIOA->BSRR = GPIO_Pin_4;
            DMA1->IFCR = DMA1_IT_TC1;

            //PORT1_Send((u8 *)(sample_values + 32));
            CDC_Send_DATA(sample_values + 32, 32);
        }
    }
#endif

    while(1);
}

void index1()
{
    while(1)
    {
        GPIOA->BRR = GPIO_Pin_4;
        if( (DMA1->ISR & DMA1_IT_HT1) != (uint32_t)RESET )
        {
            GPIOA->BSRR = GPIO_Pin_4;
            DMA1->IFCR = DMA1_IT_HT1;

            PORT1_Send((u8 *)(sample_values));
        }
        if( (DMA1->ISR & DMA1_IT_TC1) != (uint32_t)RESET )
        {
            GPIOA->BSRR = GPIO_Pin_4;
            DMA1->IFCR = DMA1_IT_TC1;

            PORT1_Send((u8 *)(sample_values + 32));
        }
    }
}

void index2()
{
    while(1)
    {
        GPIOA->BRR = GPIO_Pin_4;
        if( (DMA1->ISR & DMA1_IT_HT1) != (uint32_t)RESET )
        {
            DMA1->IFCR = DMA1_IT_HT1;
            GPIOA->BSRR = GPIO_Pin_4;

            TIM3->CNT = 20;

            TIM3->CR1 |= TIM_CR1_CEN;
            while(TIM3->CNT);
            TIM3->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));

            PORT1_Send((u8 *)(sample_values));
        }
        if( (DMA1->ISR & DMA1_IT_TC1) != (uint32_t)RESET )
        {
            DMA1->IFCR = DMA1_IT_TC1;
            GPIOA->BSRR = GPIO_Pin_4;

            TIM3->CNT = 20;

            TIM3->CR1 |= TIM_CR1_CEN;
            while(TIM3->CNT);
            TIM3->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));

            PORT1_Send((u8 *)(sample_values + 32));
        }
    }
}

void terminal_main()
{
    //NVIC_Config();
    Sampling_Config();
    void (*main_thread)();

    u8 index = 0;
    if( GPIOD->IDR & GPIO_Pin_15 )
    {
        index = 1;
        main_thread = &index1;
    }
    else
    {
        index = 2;
        main_thread = &index2;
    }

    nRF24L01_Initial();
    Repower_NRF24L01();

    GPIOD->BSRR = GPIO_Pin_10 | GPIO_Pin_11;
    GPIOA->BRR = GPIO_Pin_4;
    handshaking_tx(index);

    //u8 ADDR[5];
    //SPI_Read_Buf(PORT2, READ_REG_NRF24L01 + RX_ADDR_P0, ADDR, 5);

    main_thread();
}

void center_main()
{
    //u8 (*process_packge)();

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

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
    //process_packge = &virtual_com_send;

    Send_Buffer[0] = 0xFF;
    Send_Buffer[34] = 0xFF;
    Send_Buffer[35] = 0x00;

    USART_Config();
    //NVIC_Config();
    nRF24L01_Initial();
    Repower_NRF24L01();

    //u8 ADDR[5];
    u8 status;
    u8 * rx_buffer = Send_Buffer + 2;

    GPIOD->BSRR = GPIO_Pin_10 | GPIO_Pin_11;
    handshaking_rx();

    //SPI_Read_Buf(PORT2, READ_REG_NRF24L01 + RX_ADDR_P0, ADDR, 5);
    
    TIM_Cmd(TIM3, ENABLE);

    while(1)
    {
        GPIOD->BRR = GPIO_Pin_10;
        while(PORT1_IRQ || !TIM3->CNT);
        TIM3->CNT = 20;

        status = SPI_RDR(PORT1, STATUS);
        if(status & 0x40)
        {
            GPIOD->BSRR = GPIO_Pin_10;
            GPIOD->BRR = GPIO_Pin_11;
            SPI_Read_Buf(PORT1, RD_RX_PLOAD, rx_buffer, TX_PLOAD_WIDTH);
            SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x40);

            Send_Buffer[1] = (status & 0x0E) >> 1;
        }
        else
            GPIOD->BSRR = GPIO_Pin_11;

        //process_packge();
        CDC_Send_DATA(Send_Buffer, 36);
    }
}

void trunk_main()
{
}

void storage_main()
{
}

void handshaking_rx()
{
    u8 replyed = 0;
    u8 status = 0x00;

    PORT1_CLR_CE;
    SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS_0, TX_ADR_WIDTH);

    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_AA, 0x00);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_RXADDR, 0x00);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_CH, 0x40);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_SETUP, 0x0F);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x70);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + CONFIG, 0x06);
    SPI_Write_Buf(PORT1, WR_TX_PLOAD, sync, TX_PLOAD_WIDTH);

    PORT1_SET_CE;
    while(PORT1_IRQ);
    status = SPI_RDR(PORT1, STATUS);
    if(status & TX_DS)	/*tx_ds == 0x20*/
    {
        SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x20);
    }

    Config_Receive_PORT();
    while(replyed < 3)
    {
invalidreply:
        while(PORT1_IRQ);

        status = SPI_RDR(PORT1, STATUS);
        if(status & 0x40)
        {
            GPIOD->BRR = GPIO_Pin_10;
            SPI_Read_Buf(PORT1, RD_RX_PLOAD, RX_BUF, TX_PLOAD_WIDTH);
            SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x40);

            for(i=1; i<6; i++)
            {
                if(RX_BUF[i] != sync[i])
                {
                    goto invalidreply;
                }
            }
            USART1->DR = RX_BUF[0];
            replyed |= RX_BUF[0];
        }
    }
}

void handshaking_tx(u8 index)
{
    u8 status=0x00;
    u8 synced = 1;
    u8 i;

    PORT1_CLR_CE;

    SPI_Write_Buf(PORT1, WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS_0, TX_ADR_WIDTH);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);

    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_AA, 0x00);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + EN_RXADDR, 0x01);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_CH, 0x40);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + RF_SETUP, 0x0F);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x70);
    SPI_WRR(PORT1, WRITE_REG_NRF24L01 + CONFIG, 0x07);

    PORT1_SET_CE;

    while(synced)
    {
unsynced:
        while(PORT1_IRQ);

        status=SPI_RDR(PORT1, STATUS);
        if(status & 0x40)
        {
            SPI_Read_Buf(PORT1, RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);
            SPI_WRR(PORT1, WRITE_REG_NRF24L01 + STATUS, 0x40);
        }
        for(i=0; i<6; i++)
        {
            if(RX_BUF[i] != sync[i])
                goto unsynced;
        }
        synced = 0;
    }
    sync[0]=index;

    Config_Send_PORT(index);

    if(index == 2)
    {
        TIM3->CNT = 20;

        TIM_Cmd(TIM3, ENABLE);
        while(TIM3->CNT);
        TIM_Cmd(TIM3, DISABLE);
    }
    GPIOA->BSRR = GPIO_Pin_4;


    PORT1_Send(sync);

    TIM3->CNT = 20;
    TIM_Cmd(TIM3, ENABLE);
    while(TIM3->CNT);
    TIM_Cmd(TIM3, DISABLE);

    TIM_Cmd(TIM2, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

u8 spi_flash_store()
{
}

