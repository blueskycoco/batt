#include "stm32f10x.h"
#include "power.h"
#define INVALID_ADC_VALUE 100
static uint8_t reserved[] =
                            /* As defined by SMBus Spec. Appendix C */
                            {0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x28,
                            0x37, ARP_ADDRESS,
                            /* As defined by SMBus Spec. Sect. 5.2 */
                            0x01, 0x02, 0x03, 0x04, 0x05,
                            0x06, 0x07, 0x78, 0x79, 0x7a, 0x7b,
                            0x7c, 0x7d, 0x7e, 0x7f,
                            /* Common PC addresses (bad idea) */
                            0x2d, 0x48, 0x49, /* sensors */
                            0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, /* eeproms */
                            0x69, /* clock chips */
                            /* Must end in 0 which is also reserved */
                            0x00};
uint8_t address_pool[SMBUS_ADDRESS_SIZE];
uint8_t	g_i2c_addr[2];/*系统中最多同时存在两块智能电池,g_i2c_addr[0]固定为电池A即ADC3_IN6，g_i2c_addr[1]固定为电池B即ADC3_IN7*/
uint8_t g_batt_li_num=0;
PowerMan_t pm = {0};
I2C_InitTypeDef  I2C_InitStructure;
/*延时函数*/
void myDelay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}
/*stm32 iic初始化*/
void pin_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    /* Enable GPIOB,E,F,G clock */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    /* Enable I2C2 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin =  BATS_I2C_SCL_PIN;
    GPIO_Init(BATS_I2C_SCL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_I2C_SDA_PIN;
    GPIO_Init(BATS_I2C_SDA_PORT, &GPIO_InitStructure);
    /* Config pin */ 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin =  BATS_SEL_C_PIN;
    GPIO_Init(BATS_SEL_C_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_SEL_B_PIN;
    GPIO_Init(BATS_SEL_B_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_SEL_A_PIN;
    GPIO_Init(BATS_SEL_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_SEL_STAC_PIN;
    GPIO_Init(BATS_SEL_STAC_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_SEL_STAB_PIN;
    GPIO_Init(BATS_SEL_STAB_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_SEL_STAA_PIN;
    GPIO_Init(BATS_SEL_STAA_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_C_CHARGE_CTL_PIN;
    GPIO_Init(BATS_C_CHARGE_CTL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_C_CHARGE_FAULT_PIN;
    GPIO_Init(BATS_C_CHARGE_FAULT_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_C_CHARGE_CHRG_PIN;
    GPIO_Init(BATS_C_CHARGE_CHRG_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  PG_3V3_PIN;
    GPIO_Init(PG_3V3_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_A_CHARGE_STAT_PIN;
    GPIO_Init(BATS_A_CHARGE_STAT_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_A_CHARGE_CTL_PIN;
    GPIO_Init(BATS_A_CHARGE_CTL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_B_CHARGE_STAT_PIN
    GPIO_Init(BATS_B_CHARGE_STAT_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_B_CHARGE_CTL_PIN;
    GPIO_Init(BATS_B_CHARGE_CTL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_C_CHARGE_STAT_PIN;
    GPIO_Init(BATS_C_CHARGE_STAT_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_SEL_C_PIN;
    GPIO_Init(BATS_SEL_C_PORT, &GPIO_InitStructure);
    //ADC channel config
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin =  BATS_AB_MON_I_PIN;
    GPIO_Init(BATS_AB_MON_I_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  V3P3_MON_V_PIN;
    GPIO_Init(V3P3_MON_V_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_A_V_MON_PIN;
    GPIO_Init(BATS_A_V_MON_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_B_V_MON_PIN;
    GPIO_Init(BATS_B_V_MON_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_C_V_MON_PIN;
    GPIO_Init(BATS_C_V_MON_PORT, &GPIO_InitStructure);
    //I2C Config
    I2C_SoftwareResetCmd(I2C2,ENABLE);
    I2C_SoftwareResetCmd(I2C2,DISABLE);
    I2C_Cmd(I2C2, ENABLE);
    I2C_InitStructure.I2C_Mode = I2C_Mode_SMBusHost;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x79;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 50000;
    I2C_Init(I2C2, &I2C_InitStructure);
	I2C_CalculatePEC(I2C2, ENABLE);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC3, &ADC_InitStructure);
    ADC_Cmd(ADC3, ENABLE);
    ADC_ResetCalibration(ADC3);
    while(ADC_GetResetCalibrationStatus(ADC3));

    ADC_StartCalibration(ADC3);
    while(ADC_GetCalibrationStatus(ADC3));

}

uint16_t read_adc(uint8_t channel)
{
    uint16_t value=0;

    ADC_RegularChannelConfig(ADC3, channel, 1, ADC_SampleTime_7Cycles5);
    ADC_SoftwareStartConvCmd(ADC3, ENABLE);

    while(ADC_GetFlagStatus(ADC3,ADC_FLAG_EOC)==RESET);

    value=ADC_GetConversionValue(ADC3);

    return value;
}
uint8_t choose_addr(u8 * pool)
{
	int i;

	for(i = 0; i < 0x7f; i++) {
			if(pool[i] == ARP_FREE)
					return ((uint8_t) i);
	}
	return 0xff;
}
bool check_timeout(int flag)
{
    uint32_t timeout=100000;
    while(!I2C_CheckEvent(I2C2,flag)&&timeout)
        timeout--;
    if(timeout)
        return true;
    else
        return false;
}
bool i2c_smbus_write_byte(uint8_t addr,uint8_t command)
{
	I2C_GenerateSTART(I2C2, ENABLE);
    if(!check_timeout(I2C_EVENT_MASTER_MODE_SELECT) return false;  
    
	I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Transmitter);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
    
    I2C_SendData(I2C2,command);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
    
	I2C_TransmitPEC(I2C2, ENABLE);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
    
	I2C_GenerateSTOP(I2C2, ENABLE);

    return true;
}
bool i2c_smbus_read_block_data(uint8_t addr, uint8_t command, uint18_t len,uint8_t *blk)
{
	I2C_GenerateSTART(I2C2, ENABLE);
    if(!check_timeout(I2C_EVENT_MASTER_MODE_SELECT) return false;  
    
	I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Transmitter);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
    
    I2C_SendData(I2C2,command);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
    
	I2C_GenerateSTART(I2C2, ENABLE);
    if(!check_timeout(I2C_EVENT_MASTER_MODE_SELECT) return false;  
    
	I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Receiver);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
	for(i=0;i<len;i++)
	{
        if(!check_timeout(I2C_EVENT_MASTER_BYTE_RECEIVED) return false;  
		blk[i] = I2C_ReceiveData(I2C2);
	}
	
    I2C_AcknowledgeConfig(I2C2, DISABLE);

	I2C_GenerateSTOP(I2C2, ENABLE);
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
    return true;
}
bool i2c_smbus_write_block_data(uint8_t addr, uint8_t command, uint8_t len,uint8_t *blk)
{
	I2C_GenerateSTART(I2C2, ENABLE);
    if(!check_timeout(I2C_EVENT_MASTER_MODE_SELECT) return false;  
    
	I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Transmitter);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
    
    I2C_SendData(I2C2,command);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
    
	for(i=0;i<len;i++)
	{
		I2C_SendData(I2C2,blk[i]);
        if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
	}
	
	I2C_TransmitPEC(I2C2, ENABLE);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED) return false;  
	
	I2C_GenerateSTOP(I2C2, ENABLE);
	
    return true;
}
int16_t vol_abs(int16_t v1,int16_t v2)
{
    if(v1-v2>0)
        return v1-v2;
    else
        return v2-v1;
}
bool read_batt_info()
{
    int16_t vol1,vol2;
    if(g_batt_li_num==1)
    {

        i2c_smbus_read_block_data(g_i2c_addr[0],0x09,2,(uint8_t *)&vol1);
        if(vol_abs(vol1-read_adc(ADC_Channel_6))>vol_abs(vol1-read_adc(ADC_Channel_7)))
        {
            pm.ps[1].type=POWER_BATTERY_LI;
            pm.ps[1].volatge=vol1;
            i2c_smbus_read_block_data(g_i2c_addr[0],0x0a,2,(uint8_t *)&(pm.ps[1].current));
            pm.ps[1].power=pm.ps[1].current*pm.ps[1].volatge;
            i2c_smbus_read_block_data(g_i2c_addr[0],0x10,2,(uint8_t *)&(pm.ps[1].energyAll));
            i2c_smbus_read_block_data(g_i2c_addr[0],0x0f,2,(uint8_t *)&(pm.ps[1].energyNow));
            i2c_smbus_read_block_data(g_i2c_addr[0],0x17,2,(uint8_t *)&(pm.ps[1].cycleNum));
            i2c_smbus_read_block_data(g_i2c_addr[0],0x12,2,(uint8_t *)&(pm.ps[1].thisWorkTime));
        }else
        {
            pm.ps[0].type=POWER_BATTERY_LI;
            pm.ps[0].volatge=vol1;
            i2c_smbus_read_block_data(g_i2c_addr[0],0x0a,2,(uint8_t *)&(pm.ps[0].current));
            pm.ps[0].power=pm.ps[0].current*pm.ps[0].volatge;
            i2c_smbus_read_block_data(g_i2c_addr[0],0x10,2,(uint8_t *)&(pm.ps[0].energyAll));
            i2c_smbus_read_block_data(g_i2c_addr[0],0x0f,2,(uint8_t *)&(pm.ps[0].energyNow));
            i2c_smbus_read_block_data(g_i2c_addr[0],0x17,2,(uint8_t *)&(pm.ps[0].cycleNum));
            i2c_smbus_read_block_data(g_i2c_addr[0],0x12,2,(uint8_t *)&(pm.ps[0].thisWorkTime));
        }
    }
    else if(g_batt_li_num==2)
    {
        i2c_smbus_read_block_data(g_i2c_addr[0],0x09,2,(uint8_t *)&vol1);
        i2c_smbus_read_block_data(g_i2c_addr[1],0x09,2,(uint8_t *)&vol2);
        if(vol_abs(vol1-read_adc(ADC_Channel_6))>vol_abs(vol2-read_adc(ADC_Channel_6)))
        {
            uint8_t tmp;
            uint16_t tmp1;
            tmp=g_i2c_addr[0];
            g_i2c_addr[0]=g_i2c_addr[1];
            g_i2c_addr[1]=tmp;
            tmp1=vol1;
            vol1=vol2;
            vol2=tmp1;
        }
        pm.ps[0].volatge=vol1;
        i2c_smbus_read_block_data(g_i2c_addr[0],0x0a,2,(uint8_t *)&(pm.ps[0].current));
        pm.ps[0].power=pm.ps[0].current*pm.ps[0].volatge;
        i2c_smbus_read_block_data(g_i2c_addr[0],0x10,2,(uint8_t *)&(pm.ps[0].energyAll));
        i2c_smbus_read_block_data(g_i2c_addr[0],0x0f,2,(uint8_t *)&(pm.ps[0].energyNow));
        i2c_smbus_read_block_data(g_i2c_addr[0],0x17,2,(uint8_t *)&(pm.ps[0].cycleNum));
        i2c_smbus_read_block_data(g_i2c_addr[0],0x12,2,(uint8_t *)&(pm.ps[0].thisWorkTime));
        pm.ps[1].volatge=vol2;
        i2c_smbus_read_block_data(g_i2c_addr[1],0x0a,2,(uint8_t *)&(pm.ps[1].current));
        pm.ps[1].power=pm.ps[1].current*pm.ps[1].volatge;
        i2c_smbus_read_block_data(g_i2c_addr[1],0x10,2,(uint8_t *)&(pm.ps[1].energyAll));
        i2c_smbus_read_block_data(g_i2c_addr[1],0x0f,2,(uint8_t *)&(pm.ps[1].energyNow));
        i2c_smbus_read_block_data(g_i2c_addr[1],0x17,2,(uint8_t *)&(pm.ps[1].cycleNum));
        i2c_smbus_read_block_data(g_i2c_addr[1],0x12,2,(uint8_t *)&(pm.ps[1].thisWorkTime));

    }
    return false;
}
/*为三块智能电池分别分配iic地址,返回值代表smbus上有几块电池，并填充他们的iic地址到g_i2c_addr数组里*/
uint8_t batt_arp()
{
    int i;
	int found = 0;
    uint8_t *r,addr;
    bool ret = false;
	uint8_t blk[I2C_SMBUS_BLOCK_MAX];
	/*初始化i2c地址池里所有地址为FREE状态*/
    for(i = 0; i < SMBUS_ADDRESS_SIZE; i++)
        address_pool[i] = ARP_FREE;
		
	/*将一些默认的i2c地址所在地址池坐标设置为RESERVED状态，以免分配给智能电池*/
    r=reserved;
    do {
        address_pool[*r] = ARP_RESERVED;
    } while(*r++);
	
	/*给所有电池发送ARP_PREPARE命令*/
    ret = i2c_smbus_write_byte(ARP_PREPARE);
    if(!ret) 
	{
        return 0;
    }

	while(1) 
	{
		/*依次读取电池的UDID信息*/
		ret = i2c_smbus_read_block_data(client, ARP_GET_UDID_GEN, 19,blk);
		if(!ret) 
		{
			/*返回值不等于UDID_LENGTH说明目前系统中所有电池已枚举完，也可能是没有ack*/
			return (found);
		}
		
		/*检测这个电池的slave addr*/           
		addr = blk[16];
		if(addr != 0xFF) 
		{
			/*说明是已经分配过i2c地址或者固定i2c地址的电池*/
			addr >>= 1;
			if(blk[0] & 0xC0) 
			{
				/*UDID的127，126不为0，说明是可以分配i2c地址的，否则这个电池的i2c地址就固定的，对于固定地址的电池我们不再从地址池里分配地址给他*/
				if(address_pool[addr] != ARP_FREE) 
				{
					/*这个i2c地址所在的地址池坐标不是FREE状态说明了这次枚举到的电池与之前枚举到电池的i2c地址冲突，需要重新从地址池里为他分配一个*/
					if((addr = choose_addr(address_pool)) == 0xff) 
					{
						return 0;
					}
				}
			}	 
			
		} 
		else 
		{
			/*说明这个电池从未分配过i2c地址，从地址池里选择一个FREE的地址给他*/
			if((addr = choose_addr(address_pool)) == 0xff) 
			{				
				return 0;
			}

		}
		/*发送assign addr命令给这个udid所在的电池分配i2c地址*/
		blk[16] = addr << 1;
		ret = i2c_smbus_write_block_data(client, ARP_ASSIGN_ADDR,UDID_LENGTH, blk);
		if(ret)
		{	
			/*更新地址池里这个电池的i2c状态为已分配*/
			address_pool[addr] = ARP_BUSY;
			if(g_i2c_addr[found]!=0)
                found++;
            g_i2c_addr[found]=addr;
		}
		else
			return 0;
	    found++;	
    } /* while 1  */

    return found;
}

uint8_t power_available(uint8_t channel)
{
    if(pm.ps[channel].volatge>pm.minVolatge && pm.ps[channel].volatge<pm.maxVolatge)
        return 1;
    else
        return 0;
}
uint8_t select_power()
{
    if(pm.ps[0].available&&pm.ps[1].available)
    {
        if(pm.ps[0].type==POWER_ADAPTER)
        {
            if(pm.ps[1].type==POWER_BATTERY_LI)
                return 0;
            else
            {
                if(pm.ps[0].volatge>pm.ps[1].volatge)
                    return 1;
                else
                    return 0;
            }
        }
        else
        {
            if(pm.ps[1].type==POWER_ADAPTER)
                return 1;
            else
            {
                if(pm.ps[0].volatge>pm.ps[1].volatge)
                    return 1;
                else
                    return 0;
            }

        }
    }
    else
    {
        if(pm.ps[0].available)
            return 0;
        if(pm.ps[1].available)
            return 1;
        return 2;
    }
}
void switch_power(uint8_t no)
{
    if(no==0)/*选择插槽A*/
    {
        GPIO_SetBits(BATS_SEL_A_PORT, BATS_SEL_A_PIN);
        GPIO_SetBits(BATS_SEL_STAA_PORT, BATS_SEL_STAA_PIN);
        GPIO_SetBits(BATS_SEL_B_PORT, BATS_SEL_B_PIN);
        GPIO_SetBits(BATS_SEL_STAB_PORT, BATS_SEL_STAB_PIN);
        GPIO_SetBits(BATS_SEL_C_PORT, BATS_SEL_C_PIN);
        GPIO_ResetBits(BATS_SEL_STAC_PORT, BATS_SEL_STAC_PIN);
    }else if(no==1)/*选择插槽B*/
    {
        GPIO_ResetBits(BATS_SEL_A_PORT, BATS_SEL_A_PIN);
        GPIO_ResetBits(BATS_SEL_STAA_PORT, BATS_SEL_STAA_PIN);
        GPIO_ResetBits(BATS_SEL_B_PORT, BATS_SEL_B_PIN);
        GPIO_ResetBits(BATS_SEL_STAB_PORT, BATS_SEL_STAB_PIN);
        GPIO_SetBits(BATS_SEL_C_PORT, BATS_SEL_C_PIN);
        GPIO_ResetBits(BATS_SEL_STAC_PORT, BATS_SEL_STAC_PIN);
    }else/*选择插槽C*/
    {
        GPIO_ResetBits(BATS_SEL_A_PORT, BATS_SEL_A_PIN);
        GPIO_ResetBits(BATS_SEL_STAA_PORT, BATS_SEL_STAA_PIN);
        GPIO_SetBits(BATS_SEL_B_PORT, BATS_SEL_B_PIN);
        GPIO_SetBits(BATS_SEL_STAB_PORT, BATS_SEL_STAB_PIN);
        GPIO_ResetBits(BATS_SEL_C_PORT, BATS_SEL_C_PIN);
        GPIO_SetBits(BATS_SEL_STAC_PORT, BATS_SEL_STAC_PIN);
    }
}
uint8_t power_man_init(int16_t min_vol,int16_t max_vol)
{
    pin_init();
    /*batt_arp 用于轮训系统内存在几个智能电池*/
    g_i2c_addr[0]=0;
    g_i2c_addr[1]=0;
    pm.minVolatge=min_vol;
    pm.maxVolatge=max_vol;
    g_batt_li_num=batt_arp();
    if(g_batt_li_num==0)
    {   
        /*系统中无智能电池*/
        pm.ps[0].voltage=read_adc(ADC_Channel_6);
        if(pm.ps[0].volatge<INVALID_ADC_VALUE)
            pm.ps[0].type=POWER_UNKNOWN;
        else
            pm.ps[0].type=POWER_ADAPTER;
        pm.ps[0].available=power_available(0);
        pm.ps[1].voltage=read_adc(ADC_Channel_7);
        if(pm.ps[1].volatge<INVALID_ADC_VALUE)
            pm.ps[1].type=POWER_UNKNOWN;
        else
            pm.ps[1].type=POWER_ADAPTER;
        pm.ps[1].available=power_available(1);
        pm.ps[2].voltage=read_adc(ADC_Channel_8);
        if(pm.ps[2].volatge<INVALID_ADC_VALUE)
            pm.ps[2].type=POWER_UNKNOWN;
        else
            pm.ps[2].type=POWER_BATTERY_DRY;
        pm.ps[2].available=power_available(2);
        pm.currentPs=select_power();
        pm.power=read_adc(ADC_Channel_4)*pm.ps[pm.currentPs].volatge;
        switch_power(pm.currentPs);
    }
    else if(g_batt_li_num==1)
    {
        /*系统中有一个智能电池*/

    }
    else if(g_batt_li_num==2)
    {
        /*系统中有两个智能电池*/

    }
    else
    {

    }

    return 1;
}
