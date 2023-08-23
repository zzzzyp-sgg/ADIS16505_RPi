/**
 * @brief   测试树莓派与ADIS16505的连接
 * @author  zyp
 * @date    2023-08-13
*/

#include <bcm2835.h>
#include <iostream>
#include <cstring>

#include "Spi.h"

/* 定义一个引脚 */
#define ADI_PIN RPI_GPIO_P1_07

int main(int argc, char **argv) {
    if (!bcm2835_init()){
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }

    if (!bcm2835_spi_begin()){
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      return 1;
    }

    /* 高位优先 */
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    /* 模式3,CPOL = 1 (polarity), CPHA = 1 (phase) */
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
    /* 时钟分频 */
    // bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
    bcm2835_spi_set_speed_hz(1000000);
    bcm2835_gpio_fsel(ADI_PIN, BCM2835_GPIO_FSEL_OUTP);
    
    /* 重启设备 */
    bcm2835_gpio_write(ADI_PIN, LOW);
    bcm2835_delay(1000);
    bcm2835_gpio_write(ADI_PIN, HIGH);
    bcm2835_delay(1000);

    // Read PROD_ID register
    uint8_t rdat[2] = {0, 0};
    uint16_t addr = 0x72;
    addr = addr << 8;
    uint8_t wd[2] = {(addr >> 8) & 0xFF, addr & 0xFF};
    // 启动后先随便读一个
    bcm2835_gpio_write(ADI_PIN, LOW);
    //bcm2835_delayMicroseconds(2);
    bcm2835_spi_transfernb(reinterpret_cast<char*>(wd), reinterpret_cast<char*>(rdat), sizeof(wd)); // 发送和接收数据
    bcm2835_gpio_write(ADI_PIN, HIGH);
    bcm2835_delayMicroseconds(16);

    for (int i = 0; i < 10; i++) {
      // 循环读取，检查下有没有问题
      bcm2835_gpio_write(ADI_PIN, LOW);
      bcm2835_spi_transfernb(reinterpret_cast<char*>(wd), reinterpret_cast<char*>(rdat), sizeof(wd)); // 发送和接收数据
      bcm2835_gpio_write(ADI_PIN, HIGH);
      bcm2835_delayMicroseconds(16);

      uint16_t pord_id = (rdat[0] << 8) | rdat[1];
      printf("Value is:%u\n", pord_id);
    }

    bcm2835_spi_end();
    bcm2835_close();

    return 0;
}