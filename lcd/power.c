#include "stm32f10x.h"
#include "power.h"
#define INVALID_ADC_VALUE 100
PowerMan_t pm = {0};
I2C_InitTypeDef  I2C_InitStructure;
/*ÑÓÊ±º¯Êý*/
void myDelay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}
/*stm32 iic³õÊ¼»¯*/
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
    GPIO_InitStructure.GPIO_Pin =  BATS_ABC_CHARGE_FAULT_PIN;
    GPIO_Init(BATS_ABC_CHARGE_FAULT_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_ABC_CHARGE_CHRG_PIN;
    GPIO_Init(BATS_ABC_CHARGE_CHRG_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  PG_3V3_PIN;
    GPIO_Init(PG_3V3_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_A_CHARGE_STAT_PIN;
    GPIO_Init(BATS_A_CHARGE_STAT_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_A_CHARGE_CTL_PIN;
    GPIO_Init(BATS_A_CHARGE_CTL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  BATS_B_CHARGE_STAT_PIN;
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
bool check_timeout(int flag)
{
    uint32_t timeout=100000;
    while(!I2C_CheckEvent(I2C2,flag)&&timeout)
        timeout--;
    if(timeout)
        return TRUE;
    else
        return FALSE;
}
bool i2c_smbus_read_block_data(uint8_t addr, uint8_t command, uint8_t len,uint8_t *blk)
{
    int i;
    I2C_GenerateSTART(I2C2, ENABLE);
    if(!check_timeout(I2C_EVENT_MASTER_MODE_SELECT)) return FALSE;  

    I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Transmitter);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return FALSE;  

    I2C_SendData(I2C2,command);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return FALSE;  

    I2C_GenerateSTART(I2C2, ENABLE);
    if(!check_timeout(I2C_EVENT_MASTER_MODE_SELECT)) return FALSE;  

    I2C_Send7bitAddress(I2C2, addr<<1, I2C_Direction_Receiver);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return FALSE;  
    for(i=0;i<len+1;i++)
    {
        if(!check_timeout(I2C_EVENT_MASTER_BYTE_RECEIVED)) return FALSE;  
        blk[i] = I2C_ReceiveData(I2C2);
    }

    I2C_AcknowledgeConfig(I2C2, DISABLE);

    I2C_GenerateSTOP(I2C2, ENABLE);

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return TRUE;
}
bool check_battery_li()/*1 ÊÇï®µç³Ø 0ÊÇÊÊÅäÆ÷*/
{
    I2C_GenerateSTART(I2C2, ENABLE);
    if(!check_timeout(I2C_EVENT_MASTER_MODE_SELECT)) return FALSE;  

    I2C_Send7bitAddress(I2C2, 0x0B<<1, I2C_Direction_Transmitter);
    if(!check_timeout(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return FALSE;  

    I2C_GenerateSTOP(I2C2, ENABLE);
    return TRUE;
}
bool read_batt_info(uint8_t channel)
{
    if(channel==0)
    {
        //switch i2c to slot A
    }
    else
    {
        //switch i2c to slot B
    }
    i2c_smbus_read_block_data(0x0B,0x09,2,(uint8_t *)&pm.ps[channel].voltage);
    i2c_smbus_read_block_data(0x0B,0x0a,2,(uint8_t *)&(pm.ps[channel].current));
    pm.ps[channel].power=pm.ps[channel].current*pm.ps[channel].voltage;
    i2c_smbus_read_block_data(0x0B,0x10,2,(uint8_t *)&(pm.ps[channel].energyAll));
    i2c_smbus_read_block_data(0x0B,0x0f,2,(uint8_t *)&(pm.ps[channel].energyNow));
    i2c_smbus_read_block_data(0x0B,0x17,2,(uint8_t *)&(pm.ps[channel].cycleNum));
    i2c_smbus_read_block_data(0x0B,0x12,2,(uint8_t *)&(pm.ps[channel].thisWorkTime));

    return FALSE;
}
uint8_t check_power(uint8_t channel)
{
    int16_t vol;
    bool power_type;
    if(channel!=2)
    {
        //1 read the voltage from adc
        vol=read_adc(ADC_Channel_6+channel);
        //2 check power battery type
        if(channel==0)
        {
            //switch i2c to slot A
        }
        else
        {
            //switch i2c to slot B
        }
        power_type=check_battery_li();
        if(vol>pm.minVoltage && vol<pm.maxVoltage)
        {
            if(pm.ps[channel].available==0)
            {
                if(pm.ps[channel].type==POWER_UNKNOWN)
                    pm.ps[channel].available=1;
            }

        }
        else
        {

            pm.ps[channel].available=0;
        }

        if(power_type)
            pm.ps[channel].type=POWER_BATTERY_LI;
        else
        {
            if(vol>INVALID_ADC_VALUE)
            {
                pm.ps[channel].type=POWER_ADAPTER;
            }
            else
            {
                pm.ps[channel].type=POWER_UNKNOWN;
            }
        }
        pm.ps[channel].voltage=vol;
        if(pm.ps[channel].type==POWER_BATTERY_LI)
            read_batt_info(channel);

    }
    else
    {

        vol=read_adc(ADC_Channel_6+channel);
        if(vol>pm.minVoltage && vol<pm.maxVoltage)
        {
            if(pm.ps[channel].available==0)
            {
                if(pm.ps[channel].type==POWER_UNKNOWN)
                    pm.ps[channel].available=1;
            }

        }
        else
        {

            pm.ps[channel].available=0;
        }
        if(vol<INVALID_ADC_VALUE)
            pm.ps[channel].type=POWER_UNKNOWN;
        else
            pm.ps[channel].type=POWER_BATTERY_DRY;
        pm.ps[channel].voltage=vol;
    }
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
                if(pm.ps[0].voltage>pm.ps[1].voltage)
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
                if(pm.ps[0].voltage>pm.ps[1].voltage)
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
    if(no==0)/*Ñ¡Ôñ²å²ÛA*/
    {
        GPIO_SetBits(BATS_SEL_A_PORT, BATS_SEL_A_PIN);
        GPIO_SetBits(BATS_SEL_STAA_PORT, BATS_SEL_STAA_PIN);
        GPIO_SetBits(BATS_SEL_B_PORT, BATS_SEL_B_PIN);
        GPIO_SetBits(BATS_SEL_STAB_PORT, BATS_SEL_STAB_PIN);
        GPIO_SetBits(BATS_SEL_C_PORT, BATS_SEL_C_PIN);
        GPIO_ResetBits(BATS_SEL_STAC_PORT, BATS_SEL_STAC_PIN);
    }else if(no==1)/*Ñ¡Ôñ²å²ÛB*/
    {
        GPIO_ResetBits(BATS_SEL_A_PORT, BATS_SEL_A_PIN);
        GPIO_ResetBits(BATS_SEL_STAA_PORT, BATS_SEL_STAA_PIN);
        GPIO_ResetBits(BATS_SEL_B_PORT, BATS_SEL_B_PIN);
        GPIO_ResetBits(BATS_SEL_STAB_PORT, BATS_SEL_STAB_PIN);
        GPIO_SetBits(BATS_SEL_C_PORT, BATS_SEL_C_PIN);
        GPIO_ResetBits(BATS_SEL_STAC_PORT, BATS_SEL_STAC_PIN);
    }else/*Ñ¡Ôñ²å²ÛC*/
    {
        GPIO_ResetBits(BATS_SEL_A_PORT, BATS_SEL_A_PIN);
        GPIO_ResetBits(BATS_SEL_STAA_PORT, BATS_SEL_STAA_PIN);
        GPIO_SetBits(BATS_SEL_B_PORT, BATS_SEL_B_PIN);
        GPIO_SetBits(BATS_SEL_STAB_PORT, BATS_SEL_STAB_PIN);
        GPIO_ResetBits(BATS_SEL_C_PORT, BATS_SEL_C_PIN);
        GPIO_SetBits(BATS_SEL_STAC_PORT, BATS_SEL_STAC_PIN);
    }
}
PowerMan_t power_man_timer_poll(int16_t min_vol,int16_t max_vol)
{
    int i;
    pm.minVoltage=min_vol;
    pm.maxVoltage=max_vol;

    for(i=0;i<3;i++)
        check_power(i);
    pm.currentPs=select_power();
    pm.power=read_adc(ADC_Channel_4)*pm.ps[pm.currentPs].voltage;
    return pm;
}
void power_man_timer_interrupt()
{
    int16_t vol;
    int channel;
    for(channel=0;channel<3;channel++)
    {
        vol=read_adc(ADC_Channel_6);
        if(vol<pm.minVoltage || vol>pm.maxVoltage)
            pm.ps[channel].available=0;
    }

    pm.currentPs=select_power();
    switch_power(pm.currentPs);

}
uint8_t power_man_init(int16_t min_vol,int16_t max_vol)
{
    int i;
    pin_init();
    pm.minVoltage=min_vol;
    pm.maxVoltage=max_vol;
    pm.ps[0].type=POWER_UNKNOWN;
    pm.ps[0].available=0;
    pm.ps[1].type=POWER_UNKNOWN;
    pm.ps[1].available=0;
    pm.ps[2].type=POWER_UNKNOWN;
    pm.ps[2].available=0;
    for(i=0;i<3;i++)
        check_power(i);
    pm.currentPs=select_power();
    pm.power=read_adc(ADC_Channel_4)*pm.ps[pm.currentPs].voltage;
    switch_power(pm.currentPs);
    return 1;
}
