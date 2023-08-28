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
    bcm2835_delay(500);
    bcm2835_gpio_write(ADI_PIN, HIGH);
    bcm2835_delay(500);

    // Read PROD_ID register
    uint8_t rdat[2] = {0, 0};
    uint16_t addr = 0x72;
    uint16_t zero = 0x00;
    zero = zero << 8;
    addr = addr << 8;
    uint8_t wd[2] = {(addr >> 8) & 0xFF, addr & 0xFF};
    uint8_t zd[2] = {(zero >> 8) & 0xFF, zero & 0xFF};
    // 启动后先随便读一个
    bcm2835_gpio_write(ADI_PIN, LOW);
    /* 这里发送的数据再下一次传输才会收到 */
    bcm2835_spi_transfernb(reinterpret_cast<char*>(wd), reinterpret_cast<char*>(rdat), sizeof(wd)); // 发送和接收数据
    bcm2835_gpio_write(ADI_PIN, HIGH);
    bcm2835_delayMicroseconds(16);
    /*
    for (int i = 0; i < 10; i++) {
      // 循环读取，检查下有没有问题
      bcm2835_gpio_write(ADI_PIN, LOW);
      bcm2835_spi_transfernb(reinterpret_cast<char*>(wd), reinterpret_cast<char*>(rdat), sizeof(wd)); // 发送和接收数据
      bcm2835_gpio_write(ADI_PIN, HIGH);
      bcm2835_delayMicroseconds(16);

      uint16_t prod_id = (rdat[0] << 8) | rdat[1];
      printf("Value is:%u\n", prod_id);
    }
    */

    uint16_t FILT = 0x5C;
    FILT = FILT << 8;
    uint8_t fd[2] = {(FILT >> 8) & 0xFF, FILT & 0xFF};
    /* 尝试往里面写数据 */
    uint16_t txBuf1 = ((0x5C | 0x80) << 8) | (3 & 0xFF);
    uint8_t tx_data1[2] = {txBuf1 >> 8, txBuf1 & 0xFF};
    uint16_t txBuf2 = ((0x5D | 0x80) << 8) | (0 & 0xFF);
    uint8_t tx_data2[2] = {txBuf2 >> 8, txBuf2 & 0xFF};
    bcm2835_gpio_write(ADI_PIN, LOW);
    bcm2835_spi_transfernb(reinterpret_cast<char*>(tx_data1),reinterpret_cast<char*>(rdat), sizeof(tx_data1));
    bcm2835_gpio_write(ADI_PIN, HIGH);
    bcm2835_delayMicroseconds(16);
    uint16_t prod_id = (rdat[0] << 8) | rdat[1];
    printf("PROD_ID is:%u\n", prod_id);    
    bcm2835_gpio_write(ADI_PIN, LOW);
    bcm2835_spi_transfernb(reinterpret_cast<char*>(tx_data2),reinterpret_cast<char*>(rdat), sizeof(tx_data2));
    bcm2835_gpio_write(ADI_PIN, HIGH);
    bcm2835_delayMicroseconds(16);

    /* 读FILT_CRTL */
    bcm2835_gpio_write(ADI_PIN, LOW);
    bcm2835_spi_transfernb(reinterpret_cast<char*>(fd), reinterpret_cast<char*>(rdat), sizeof(fd));
    bcm2835_gpio_write(ADI_PIN, HIGH);
    bcm2835_delayMicroseconds(16);


    bcm2835_gpio_write(ADI_PIN, LOW);
    bcm2835_spi_transfernb(reinterpret_cast<char*>(zd), reinterpret_cast<char*>(rdat), sizeof(zd));
    bcm2835_gpio_write(ADI_PIN, HIGH);
    bcm2835_delayMicroseconds(16);
    uint16_t filt_ctrl = (rdat[0] << 8) | rdat[1];
    printf("FILT_CRTL is:%u\n", filt_ctrl);
    
    bcm2835_spi_end();
    bcm2835_close();

    return 0;
}