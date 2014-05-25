#include "stm32f10x_conf.h"

#ifndef _OLED_H_
#define _OLED_H_

#define OLED_CS_SET  GPIOB->BSRR = GPIO_Pin_8
#define OLED_RST_SET GPIOB->BSRR = GPIO_Pin_7
#define OLED_DC_SET  GPIOB->BSRR = GPIO_Pin_6

#define OLED_CS_CLR  GPIOB->BRR = GPIO_Pin_8
#define OLED_RST_CLR GPIOB->BRR = GPIO_Pin_7
#define OLED_DC_CLR  GPIOB->BRR = GPIO_Pin_6

void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   
							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);	 

#endif
