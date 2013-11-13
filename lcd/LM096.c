#include "stm32f10x.h"
#include "numfont.h"
#include "LM096.h"
#define SSD1306_LCDWIDTH                    128
#define SSD1306_LCDHEIGHT                   64
#define I2C1_DR_Address                     0x40005410
# define _BV(bit) (1<<(bit))
I2C_InitTypeDef  I2C_InitStructure;
void myDelay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}
void pin_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
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
    for(i=0;i<sizeof(init_reg);i++)
    {
        ssd1306_send_byte(init_reg[i],TRUE);
    }
    clear();
}
#if 0
void  drawchar(uint8_t x, uint8_t line, uint8_t c) {
    unsigned char i;
    if((line >= SSD1306_LCDHEIGHT/8) || (x >= (SSD1306_LCDWIDTH - 6)))
        return;
    for(i =0; i<5; i++ )
    {
        buffer[x + (line*128) ] =font[c*5+i];// (*(unsigned char *)(font+(c*5)+i)/*& ~_BV(((line*128)%8))*/) ;
        x++;
    }
}/*11*15*/
void  drawchar2(uint8_t x, uint8_t line, uint8_t c) {
    unsigned char i,j;
    if((line >= SSD1306_LCDHEIGHT/8) || (x >= (SSD1306_LCDWIDTH - 9)))
        return;
		j=x;
    for(i =0; i<8; i++ )
    {
        buffer[x + (line*128) ] =font2[(c)*16+i];// (*(unsigned char *)(font+(c*5)+i)/*& ~_BV(((line*128)%8))*/) ;
        x++;
    }
		for(i =0; i<8; i++ )
    {
        buffer[x + ((line+1)*128)-8 ] =font2[(c)*16+8+i];// (*(unsigned char *)(font+(c*5)+i)/*& ~_BV(((line*128)%8))*/) ;
        x++;
    }
}

void drawstring(uint8_t x, uint8_t line, char *c) {
    while (c[0] != 0) {
				if(line==2 || line==0)
				{
					drawchar2(x, line, c[0]);
					c++;
					x += 9; // 6 pixels wide
					if (x + 9 >= SSD1306_LCDWIDTH) {
            x = 0;    // ran out of this line
            line=line+2;
        }
				}else
				{
					drawchar(x, line, c[0]);
					c++;
					x += 6; // 6 pixels wide
					if (x + 6 >= SSD1306_LCDWIDTH) {
            x = 0;    // ran out of this line
            line++;
        }
			}
        if (line >= (SSD1306_LCDHEIGHT/8))
            return;        // ran out of space :(
    }

}
#endif
void setpixel(uint8_t x, uint8_t y) {
  if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
    return;

  // x is which column
  //if (color == WHITE) 
    buffer[x+ (y/8)*SSD1306_LCDWIDTH] |= _BV((y%8));  
  //else
    //buffer[x+ (y/8)*SSD1306_LCDWIDTH] &= ~_BV((y%8)); 
}
void drawline(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
  uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
    uint8_t dx, dy,tmp;
	int8_t err;
	int8_t ystep;
  if (steep) {
    //swap(x0, y0);
    //swap(x1, y1);
	tmp=x0;
	x0=y0;
	y0=tmp;
	tmp=x1;
	x1=y1;
	y1=tmp;
  }

  if (x0 > x1) {
    //swap(x0, x1);
    //swap(y0, y1);
	tmp=x0;
	x0=x1;
	x1=tmp;
	tmp=y0;
	y0=y1;
	y1=tmp;
  }


  dx = x1 - x0;
  dy = abs(y1 - y0);

  err = dx / 2;
  

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;}

  for (; x0<x1; x0++) {
    if (steep) {
      setpixel(y0, x0);
    } else {
      setpixel(x0, y0);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// filled rectangle
void fillrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {

  // stupidest version - just pixels - but fast with internal buffer!
  uint8_t i,j;
  for (i=x; i<x+w; i++) {
    for (j=y; j<y+h; j++) {
      setpixel(i, j);
    }
  }
}
void draw(uint8_t bat1,uint8_t bat2,char *c)
{
	int x=0,line=0,x1=0,i,j;
	if(bat1>100||bat1<0||bat2>100||bat2<0||strlen(c)!=7)
		return ;
	//draw batt icon
	drawline(0,32,28,32);
	drawline(28,32,28,36);
	drawline(28,36,32,36);
	drawline(32,36,32,44);
	drawline(32,44,28,44);
	drawline(28,44,28,48);
	drawline(28,48,0,48);
	drawline(0,48,0,32);
	fillrect(0,32,(uint8_t)(bat1*28/100),16);
	drawline(0,48,28,48);
	drawline(28,48,28,52);
	drawline(28,52,32,52);
	drawline(32,52,32,60);
	drawline(32,60,28,60);
	drawline(28,60,28,64);
	drawline(28,64,0,64);
	drawline(0,64,0,48);
	fillrect(0,48,(uint8_t)(bat1*28/100),16);
	while(c[0]!=0)
	{
		//need to draw char ,like 3300 4.7
		if(c[0]!='.')
		{
			for(j=0;j<4;j++)
			{
				x=x1;
				for(i=j*32;i<32*(j+1);i++)
				{
					buffer[x+((line+j)*128)]=font32[(c[0]-48)*128+i];
					x++;
				}
			}
		}
		else
		{
			for(j=0;j<4;j++)
			{
				x=x1;
				for(i=j*32;i<32*(j+1);i++)
				{
					buffer[x+((line+j)*128)]=font32[1280+i];
					x++;
				}
			}
		}
		c++;
		x1=x1+33;
		if(x1+33>=SSD1306_LCDWIDTH)
		{
			x1=0;
			line=line+4;
		}
		if(line==4)//igore batt zero
			x1=33;
	}	
}
void display(void) {
    int i,j;
    for(j=0;j<sizeof(draw_reg);j++)
    {						
        ssd1306_send_byte(draw_reg[j],TRUE);
    }

    for(i=0;i<sizeof(buffer);i++)
    {
        ssd1306_send_byte(buffer[i],FALSE);
    }
}

