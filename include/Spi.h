/**
 * @brief   Spi相关宏定义和函数
 * @author  zyp
 * @date    2023-08-07
*/

#ifndef SPI_H
#define SPI_H

#include <bcm2835.h>
#include <cstdio>
#include <cstdint>

/* 芯片选择的引脚 */
#define CS_PIN RPI_GPIO_P1_07

/* 超时时间 */
#define TIME_OUT 20

/* 设为低电平，允许接收外部数据 */
void spiEnableNss();

/* 设为高电平，不允许接收外部数据 */
void spiDisableNss();

/* 向设备发送数据 */
void spiWriteWord(uint16_t pdata);

/* 读取数据 */
void spiReadWord(uint16_t *pdata, uint16_t length);

/* non-blocking write word */
void spiNonBlockingWriteWord(uint16_t wb,uint16_t *rb);

/* non-blocking write array */
void spiNonBlockingWriteArray(uint8_t *tx, uint8_t *rx, uint16_t size);

#endif