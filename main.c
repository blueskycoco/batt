//u-easytech cache lee
//QQ:53755787
//16BIT 9325	   103ZET
//20110406
////Á¬½Ó103°åÉÏµÄpA9,PA10,·Ö±ð½ÓUSB×ªTTLÏßµÄ°×É«¼°ÂÌÉ«Ïß£¬ÇÒGNDÓëºÚÉ«ÏßÏàÁ¬¡££
//ÔÚÒº¾§µÄÉÏÏÔÊ¾´®¿Ú1µ±Ç°Ëù·¢ËÍµÄÊý¾Ý¼°½ÓÊÕµ½µÄÊý¾Ý£¬£¨×¢Òâ£¬Òº¾§ÉÏÏÔÊ¾µÄÎª10½øÖÆ£©
//²¨ÌØÂÊÎª19200£¬ÎÞÆæÅ¼Ð£Ñé£¬8Êý¾ÝÎ»

#include "stm32f10x.h"
#include "stm32_2.8_lcd.h"
#include "LM096.h"
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
unsigned int ADC_ConvertedValue;

void Stm32_Clock_Init(void)
{
    unsigned char temp=0;   

    RCC->CR|=0x00010000;  //Íâ²¿¸ßËÙÊ±ÖÓÊ¹ÄÜHSEON
    while(!(RCC->CR>>17));//µÈ´ýÍâ²¿Ê±ÖÓ¾ÍÐ÷
    RCC->CFGR=0X1C0400;   //APB1/2=DIV2;AHB=DIV1;PLL=9*CLK;
    RCC->CFGR|=1<<16;	  //PLLSRC ON 
    FLASH->ACR|=0x32;	  //FLASH 2¸öÑÓÊ±ÖÜÆÚ

    RCC->CR|=0x01000000;  //PLLON
    while(!(RCC->CR>>25));//µÈ´ýPLLËø¶¨
    RCC->CFGR|=0x00000002;//PLL×÷ÎªÏµÍ³Ê±ÖÓ	 
    while(temp!=0x02)     //µÈ´ýPLL×÷ÎªÏµÍ³Ê±ÖÓÉèÖÃ³É¹¦
    {   
        temp=RCC->CFGR>>2;
        temp&=0x03;
    }     
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE, ENABLE); 

    //LEDS
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE,&GPIO_InitStructure);	     

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;  
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;  
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    //keys							  
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;    
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;  
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;  
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;   
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;   
    GPIO_Init(GPIOE,&GPIO_InitStructure);
}

void LCD_ShowNum(uint8_t x,uint16_t y,uint16_t data)
{
    LCD_DisplayChar(x,y,data/10000+48); 
    LCD_DisplayChar(x,(y+25),data%10000/1000+48);   // %10000
    LCD_DisplayChar(x,(y+50),data%1000/100+48); 
    LCD_DisplayChar(x,(y+75),data%100/10+48);	 
    LCD_DisplayChar(x,(y+100),data%10+48);
}

void AD_init()
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    ADC_InitTypeDef   ADC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;

    /* GPIOC Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* ADC1 Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

    /* pc4:pointer,PC0 vbus                            */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (u32)&ADC_ConvertedValue;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize         = 1;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Word;;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel1,  ENABLE);        /* Enable DMA Channel1                */	  

    /* ADC1 Configuration (ADC1CLK = 18 MHz) -----------------------------------*/
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;						  
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = 1;		//AD×ª»»Í¨µÀÊý
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 Regular Channel1 Configuration                                      */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);	  //PC0

    ADC_Cmd(ADC1, ENABLE); 
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1)); 
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    ADC_DMACmd(ADC1, ENABLE);             /* Enable ADC1's DMA interface        */

    ADC_SoftwareStartConvCmd(ADC1,ENABLE);/* Start ADC1 Software Conversion     */
}


void Usart_int(void)
{
    USART_InitTypeDef	 USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    /* Configure USART1 Rx (PA10) as input floating                             */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;	 		    
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART1 Tx (PA9) as alternate function push-pull                */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);	

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

    USART_DeInit(USART1);
    /* ---------------------------------------------------------------
       USART Configuration: 
       FPCLK = 36 MHz, Baud rate =36000,MODE: RX & TX
       --------------------------------------------------------------- */
    USART_InitStructure.USART_BaudRate            = 19200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    /* Enable the USART Receive interrupt: this interrupt is generated when the 
       USART1 receive data register is not empty */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
    USART_ClearFlag(USART1,USART_FLAG_TXE);
}

void USART1_IRQHandler()
{
    u8 temp;

    if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);

        //½ÓÊÕÊý¾ÝÔÚÒÔÏÂµØµã¼Ó´úÂë
        temp=USART_ReceiveData(USART1);
        LCD_ShowNum(200,5,temp);
    }
}
#define USART1_IRQChannel            ((u8)0x25)  /* USART1 global Interrupt */
void NVIC_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

    // Enable the USART1 gloabal Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);
}



main()
{  
    u32 i;
    u8 count=0;
#if 0
    Stm32_Clock_Init();
    GPIO_Configuration();

    LCD_Setup();

    LCD_Clear(Blue);	  
    LCD_SetTextColor(White); 
    LCD_SetBackColor(Red); 	
    LCD_DisplayStringLine(0,"USART1 TEST: "); 
    LCD_DisplayStringLine(2,"USART1 Send: "); 
    LCD_DisplayStringLine(6,"USART1 Receive: ");
    LCD_SetTextColor(White);

    NVIC_init();
    Usart_int();	

    while(1)	
    {						 
        for(i=0;i<0x500000;i++);	
        for(i=0;i<0x500000;i++);

        //´®¿Ú·¢ËÍ0xbb
        USART_SendData(USART1,count);
        while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
        USART_ClearFlag(USART1,USART_FLAG_TXE);
        LCD_ShowNum(107,5,count++);	
    }
#endif
    int val1=0,val2=1,val3=2,val4=3,val5=4,val6=5;
    int bat1=0,bat2=0;
	u8 str[100];
		sprintf(str,"%d%d%d%d%d.%d",val1,val2,val3,val4,val5,val6);
		//str[16]='\0';
    ssd1306_init();
    //drawstring(0,0,str);
	draw(bat1,bat2,str);
    display();
		while(1){
			val1++;
			val2++;
			val3++;
			val4++;
			val5++;			
			sprintf(str,"%d%d%d%d%d.%d",val1,val2,val3,val4,val5,val6);
			//drawstring(0,0,str);
			draw(bat1,bat2,str);
			display();
			Delay(100000);
			if(val1==9)
				val1=0;
			if(val2==9)
				val2=0;
			if(val3==9)
				val3=0;
			if(val4==9)
				val4=0;
			if(val5==9)
				val5=0;
			
		}
}
