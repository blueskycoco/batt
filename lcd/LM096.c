#include "stm32f10x.h"
#include "glcdfont.c"
#include "LM096.h"
#define SSD1306_LCDWIDTH                    128
#define SSD1306_LCDHEIGHT                   64
#define I2C1_DR_Address                     0x40005410
#define _BV(bit) (1<<(bit))
#define INIT_SSD1306_REG    0
#define SET_DISPLAY_BUF     2
#define FILL_DISPLAY_BUF    1
I2C_InitTypeDef  I2C_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;
#define OP_DMA 0
void myDelay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}
void pin_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
#if	OP_DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif
    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    /* Enable I2C1 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    //DMA Config
#if	OP_DMA	
    DMA_DeInit(DMA1_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)init_reg;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 50;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
#endif
    //I2C Config
    I2C_SoftwareResetCmd(I2C1,ENABLE);
    I2C_SoftwareResetCmd(I2C1,DISABLE);
    I2C_Cmd(I2C1, ENABLE);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x79;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    I2C_Init(I2C1, &I2C_InitStructure);
}
#if	OP_DMA
void DMA_ReConfig(bool is_command)
{
    DMA_DeInit(DMA1_Channel6);
    if(is_command)
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)init_reg;
        DMA_InitStructure.DMA_BufferSize = sizeof(init_reg);
    }
    else
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)init_data;
        DMA_InitStructure.DMA_BufferSize = sizeof(init_data);
    }
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
}
#endif
void ssd1306_send(int type)
{
    int i;
    if(type==INIT_SSD1306_REG)
    {
        for(i=0;i<sizeof(init_reg);i++)
        {
            I2C_SendData(I2C1,init_reg[i]);
            while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  
        }
    }
    else if(type==FILL_DISPLAY_BUF)
    {
        I2C_SendData(I2C1,0xC0);
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
        for(i=0;i<sizeof(buffer);i++)
        {
            I2C_SendData(I2C1,buffer[i]);
            while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  
        }
    }else{
        for(i=0;i<sizeof(draw_reg);i++)
        {
            I2C_SendData(I2C1,draw_reg[i]);
            while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  
        }
    }

}
void ssd1306_config(int type)
{
    //int CurrDataCounterEnd,CurrDataCounterEnd1;
#if	OP_DMA
    DMA_ReConfig(is_command);
#endif    
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

    I2C_Send7bitAddress(I2C1, 0x78, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  
    //CurrDataCounterEnd = DMA_GetCurrDataCounter(DMA1_Channel6);
#if	OP_DMA	
    I2C_DMACmd(I2C1, ENABLE);
    DMA_Cmd(DMA1_Channel6, ENABLE);
    while(!DMA_GetFlagStatus(DMA1_FLAG_TC6));
    //CurrDataCounterEnd1 = DMA_GetCurrDataCounter(DMA1_Channel6);
#else
    ssd1306_send(type);
#endif		
    I2C_GenerateSTOP(I2C1, ENABLE);
#if	OP_DMA    
    DMA_Cmd(DMA1_Channel6, DISABLE);
    I2C_DMACmd(I2C1,DISABLE);
    DMA_ClearFlag(DMA1_FLAG_TC6);
#endif
}

void ssd1306_send_byte(uint8_t data,bool is_command)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  
    I2C_Send7bitAddress(I2C1, 0x78, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); 
		if(is_command)
			I2C_SendData(I2C1,0x00);
		else
			I2C_SendData(I2C1,0x40);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  	
    I2C_SendData(I2C1,data);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  
    I2C_GenerateSTOP(I2C1, ENABLE);
}
void device_rst()
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_1);//config iic address
    GPIO_SetBits(GPIOE, GPIO_Pin_0);//rst
    myDelay(10000);
    GPIO_ResetBits(GPIOE, GPIO_Pin_0);
    myDelay(10000);
    GPIO_SetBits(GPIOE, GPIO_Pin_0);
}    

// clear everything
void clear(void) {
    memset(buffer, 0, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8));
}
void ssd1306_init() {
    int i;
    pin_init();
    device_rst();
    //ssd1306_config(INIT_SSD1306_REG);
        for(i=0;i<sizeof(init_reg);i++)
        {
						ssd1306_send_byte(init_reg[i],TRUE);
        }
    clear();
}

void  drawchar(uint8_t x, uint8_t line, uint8_t c) {
    unsigned char i;
    if((line >= SSD1306_LCDHEIGHT/8) || (x >= (SSD1306_LCDWIDTH - 6)))
        return;
    for(i =0; i<5; i++ )
    {
        buffer[x + (line*128) ] =font[c*5+i];// (*(unsigned char *)(font+(c*5)+i)/*& ~_BV(((line*128)%8))*/) ;
        x++;
    }
}
void drawstring(uint8_t x, uint8_t line, char *c) {
    while (c[0] != 0) {
        drawchar(x, line, c[0]);
        c++;
        x += 6; // 6 pixels wide
        if (x + 6 >= SSD1306_LCDWIDTH) {
            x = 0;    // ran out of this line
            line++;
        }
        if (line >= (SSD1306_LCDHEIGHT/8))
            return;        // ran out of space :(
    }

}

void display(void) {
    //ssd1306_config(SET_DISPLAY_BUF);
    //ssd1306_config(FILL_DISPLAY_BUF);
    int i,j;
    for(j=0;j<sizeof(draw_reg);j++)
							{						
								ssd1306_send_byte(draw_reg[j],TRUE);
							}
            
        for(i=0;i<sizeof(buffer);i++)
        {
						//if(i%128==0)
						//{
						//	for(j=0;j<sizeof(draw_reg);j++)
						//	{						
						//		ssd1306_send_byte(draw_reg[j],TRUE);
						//	}
						//}
            ssd1306_send_byte(buffer[i],FALSE);
        }
}

