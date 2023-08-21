/**
 * @brief   Spi相关宏定义和函数
 * @author  zyp
 * @date    2023-08-07
*/

#include "Spi.h"
#include <cstring>

void spiEnableNss() {
    // Set CS_PIN as an output
    bcm2835_gpio_fsel(CS_PIN, BCM2835_GPIO_FSEL_OUTP);

    // Clear the CS_PIN (assert it low)
    bcm2835_gpio_clr(CS_PIN);
}

void spiDisableNss() {
    // Set CS_PIN as an output
    bcm2835_gpio_fsel(CS_PIN, BCM2835_GPIO_FSEL_OUTP);

    // Set the CS_PIN (deassert it high)
    bcm2835_gpio_set(CS_PIN);
}

void spiWriteWord(uint16_t pdata)
{
    // if (bcm2835_spi_getDataMode() != BCM2835_SPI_MODE2) {
    //     return; // 如果不是所需的SPI模式，返回
    // }
    /* 设置SPI模式 */
    // bcm2835_spi_setDataMode(BCM2835_SPI_MODE2);

    uint8_t wd[2] = { pdata & 0xFF, (pdata >> 8) & 0xFF };
    // bcm2835_spi_writenb((char *)wd, 2); // 发送数据
    bcm2835_spi_transfern((char *)wd, 2);
}

void spiReadWord(uint16_t *pdata, uint16_t length)
{
    /* 设置SPI模式 */
    // bcm2835_spi_setDataMode(BCM2835_SPI_MODE2);

    uint8_t rb[128];
    memset(rb, 0, sizeof(rb));

    bcm2835_spi_transfernb((char *)rb, (char *)rb, length * 2); // 接收数据

    for (int i = 0; i < length; i++) {
        pdata[i] = (uint16_t)(rb[i * 2] | (rb[i * 2 + 1] << 8));
    }
}

void spiNonBlockingWriteWord(uint16_t wb, uint16_t *rb)
{
    /* 设置SPI模式 */
    // bcm2835_spi_setDataMode(BCM2835_SPI_MODE2);

    uint8_t rdat[2] = {0, 0};
    uint8_t wd[2] = {wb & 0xFF, (wb >> 8) & 0xFF};

    bcm2835_spi_transfernb((char *)wd, (char *)rdat, 2); // 发送和接收数据

    *rb = (uint16_t)(rdat[0] | (rdat[1] << 8));
}

void spiNonBlockingWriteArray(uint8_t *tx, uint8_t *rx, uint16_t size)
{
    /* 设置SPI模式 */
    // bcm2835_spi_setDataMode(BCM2835_SPI_MODE2);

    bcm2835_spi_transfernb((char *)tx, (char *)rx, size); // 发送和接收数据
}
