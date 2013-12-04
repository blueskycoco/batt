#include "stm32f10x.h"
#include "power.h"
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
uint8_t	g_i2c_addr[3];
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
    /* Enable GPIOB,E,F,G clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
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
bool i2c_smbus_write_byte(uint8_t addr,uint8_t command)
{
	I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  
    
	I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); 
    
    I2C_SendData(I2C2,command);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  	
    
	I2C_TransmitPEC(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  
    
	I2C_GenerateSTOP(I2C2, ENABLE);
}
bool i2c_smbus_read_block_data(uint8_t addr, uint8_t command, uint8_t *blk)
{
	I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  
    
	I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); 
    
    I2C_SendData(I2C2,command);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  	
    
	I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  
    
	I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); 
	for(i=0;i<19;i++)
	{
		while(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
		blk[i] = I2C_ReceiveData(I2C2);
	}
	
    I2C_AcknowledgeConfig(I2C2, DISABLE);

	I2C_GenerateSTOP(I2C2, ENABLE);
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
}
bool i2c_smbus_write_block_data(uint8_t addr, uint8_t command, uint8_t len,uint8_t *blk)
{
	I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  
    
	I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); 
    
    I2C_SendData(I2C2,command);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  	
    
	for(i=0;i<len;i++)
	{
		I2C_SendData(I2C2,blk[i]);
		while(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	}
	
	I2C_TransmitPEC(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  
	
	I2C_GenerateSTOP(I2C2, ENABLE);
	
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
		ret = i2c_smbus_read_block_data(client, ARP_GET_UDID_GEN, blk);
		if(!ret) 
		{
			/*返回值不等于UDID_LENGTH说明目前系统中所有电池已枚举完，也可能是没有ack*/
			return (found);
		}
		
		found++;
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
			g_i2c_addr[found]=addr;
		}
		else
			return 0;
		
    } /* while 1  */

    return found;
}
