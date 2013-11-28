#ifndef __POWER_H
#define __POWER_H

#ifdef __cplusplus
 extern "C" {
#endif
#define BATS_SEL_C_PIN              (GPIO_Pin_2)
#define BATS_SEL_C_PORT             (GPIOE)
#define BATS_SEL_B_PIN              (GPIO_Pin_3)
#define BATS_SEL_B_PORT             (GPIOE)
#define BATS_SEL_A_PIN              (GPIO_Pin_4)
#define BATS_SEL_A_PORT             (GPIOE)
#define BATS_SEL_STAC_PIN           (GPIO_Pin_5)
#define BATS_SEL_STAC_PORT          (GPIOE)
#define BATS_SEL_STAB_PIN           (GPIO_Pin_6)
#define BATS_SEL_STAB_PORT          (GPIOE)
#define BATS_SEL_STAA_PIN           (GPIO_Pin_0)
#define BATS_SEL_STAA_PORT          (GPIOF)
#define BATS_ABC_CHARGE_CTL_PIN     (GPIO_Pin_1)
#define BATS_ABC_CHARGE_CTL_PORT    (GPIOF)
#define BATS_ABC_CHARGE_FAULT_PIN   (GPIO_Pin_2)
#define BATS_ABC_CHARGE_FAULT_PORT  (GPIOF)
#define BATS_ABC_CHARGE_CHRG_PIN    (GPIO_Pin_3)
#define BATS_ABC_CHARGE_CHRG_PORT   (GPIOF)
#define PG_3V3_PIN                  (GPIO_Pin_5)
#define PG_3V3_PORT                 (GPIOF)
#define BATS_A_CHARGE_STAT_PIN      (GPIO_Pin_2)
#define BATS_A_CHARGE_STAT_PORT     (GPIOG)
#define BATS_A_CHARGE_CTL_PIN       (GPIO_Pin_3)
#define BATS_A_CHARGE_CTL_PORT      (GPIOG)
#define BATS_B_CHARGE_STAT_PIN      (GPIO_Pin_4)
#define BATS_B_CHARGE_STAT_PORT     (GPIOG)
#define BATS_B_CHARGE_CTL_PIN       (GPIO_Pin_5)
#define BATS_B_CHARGE_CTL_PORT      (GPIOG)
#define BATS_C_CHARGE_STAT_PIN      (GPIO_Pin_6)
#define BATS_C_CHARGE_STAT_PORT     (GPIOG)
#define BATS_C_CHARGE_CTL_PIN       (GPIO_Pin_7)
#define BATS_C_CHARGE_CTL_PORT      (GPIOG)
#define BATS_I2C_SCL_PIN            (GPIO_Pin_10)
#define BATS_I2C_SCL_PORT           (GPIOB)
#define BATS_I2C_SDA_PIN            (GPIO_Pin_11)
#define BATS_I2C_SDA_PORT           (GPIOB)
#define BATS_AB_MON_I_PIN           (GPIO_Pin_6)
#define BATS_AB_MON_I_PORT          (GPIOF)
#define V3P3_MON_V_PIN              (GPIO_Pin_7)
#define V3P3_MON_V_PORT             (GPIOF)
#define BATS_A_V_MON_PIN            (GPIO_Pin_8)
#define BATS_A_V_MON_PORT           (GPIOF)
#define BATS_B_V_MON_PIN            (GPIO_Pin_9)
#define BATS_B_V_MON_PORT           (GPIOF)
#define BATS_C_V_MON_PIN            (GPIO_Pin_10)
#define BATS_C_V_MON_PORT           (GPIOF)
#define ARP_ADDRESS                 0x61
#define ARP_PREPARE                 0x01
#define ARP_RESET_DEV               0x02
#define ARP_GET_UDID_GEN            0x03
#define ARP_ASSIGN_ADDR             0x04
#define UDID_LENGTH                 0x11
#define SMBUS_ADDRESS_SIZE          0x80
#define ARP_FREE                    0       
#define ARP_RESERVED                1
#define ARP_BUSY                    2
#define I2C_SMBUS_BLOCK_MAX         32
/*power系统的初始化*/
void power_init(void);
#ifdef __cplusplus
}
#endif

#endif
