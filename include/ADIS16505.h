/**
 * @brief   ADIS16505 Class
 * @author  zyp
 * @date    2023-08-07
*/

#ifndef ADIS16505_H
#define ADIS16505_H

#include <bcm2835.h>
#include "Spi.h"
#include "RegisterMap.h"
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <iostream>

/* SPI模式下的数据延迟(us) */
static uint32_t tSTALL = 16;

/* 32-bit Burst command packet with trigger word embedded */
static uint8_t ADIS_Burst_Packet [34] = {0x00,0x68,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static uint8_t ADIS_Burst_Packet_Size = 34;

class ADIS16505 {
public:
    /* 角速度和加速度变量 */
    volatile uint32_t gyro_hex[3],acc_hex[3];
    double gyro_raw[3],acc_raw[3];
    double frequency;

public:
    bool setUp() {
        if(!bcm2835_spi_begin()) {
            printf("bcm2835_ispi_begin failed!\n");
            return false;
        }

        /* 高位优先 */
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
        /* 模式2,CPOL = 1 (polarity), CPHA = 1 (phase) */
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
        /* 时钟分频 */
        bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);

        /* 选中设备，将这个引脚设置为输出模式 */
        bcm2835_gpio_fsel(CS_PIN, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_set(CS_PIN); // 设备未选中

        // bcm2835_gpio_clr(CS_PIN);

        /* 初始化设备 */
        initADIS16505();
        uint16_t filter_cmd = adisReadReg(FILT_CTRL);

        /* 判断是否连接到了设备 */
        if (filter_cmd == 0x01) {
            std::cout << "Connected to SPI device." << std::endl;
        } else {
            std::cout << "Not connected to SPI device." << std::endl;
            return false;
        }

        return true;
    }

    /* 单次处理读取数据 */
    void adisSingleRead(uint16_t *rb) {
        /* 先读取 */
        adisBurstRead32bit(rb);

        /* 再转换 */
        double gyro_tmp[3];
        double acc_tmp[3];
        adisIMUConvert32bit(rb, gyro_tmp, acc_tmp);
    }
private:
    /* 初始化设备 */
    void initADIS16505() {
        /* 设备重启 */
        bcm2835_gpio_write(CS_PIN, LOW);
        bcm2835_delay(1000);
        bcm2835_gpio_write(CS_PIN, HIGH);
        bcm2835_delay(1000);

        /* Enable 32-bit burst output */
        adisSet32bitBurstConfig();

        /* 设置输出频率, output_rate=sample_rate/(n+1) */
        adisSetSampleRate(9);

        /* Hard filter setting in sensor */
        adisHardwareFilterSelect(1);
    }

    /* 读取某个地址的寄存器的值 */
    uint16_t adisReadReg(uint16_t addr) {
        if (addr == BURST_CMD)
            return false;

        uint16_t val = adisBlockingRegRead(addr);
        return val;
    }

    bool adisSet32bitBurstConfig() {
        // spiEnableNss();
        bcm2835_gpio_write(CS_PIN, LOW);
        uint16_t tmp = adisBlockingRegRead(MSC_CTRL);
        // spiDisableNss();
        bcm2835_gpio_write(CS_PIN, HIGH);
        tmp |= 1 << 9;
	    // adisRegWrite16bit(MSC_CTRL, tmp);
        adisWriteReg(MSC_CTRL, tmp);
	    // tmp = adisBlockingRegRead(MSC_CTRL);
        bcm2835_gpio_write(CS_PIN, LOW);
        tmp = adisBlockingRegRead(MSC_CTRL);
        bcm2835_gpio_write(CS_PIN, HIGH);
	    bcm2835_delay(1); //This delay allows the setting to take hold
	    return true;
    }

    /* 非blocking(阻塞)模式下读取对应地址的数据 */
    uint16_t adisNoBlockingRegRead(uint8_t addr) {
        	if(addr > 0x7F)
                return 0;

	        uint16_t rb=0;
	        spiEnableNss();
	        spiNonBlockingWriteWord(addr << 8, &rb);
	        spiDisableNss();

	        bcm2835_delay(tSTALL);

	        return rb;
    }

    /* 读取对应地址的数据 */
    uint16_t adisBlockingRegRead(uint8_t addr) {
        /* 超出寄存器的地址范围 */
        if (addr > 0x7F)
            return 0;

        uint8_t rdat[2] = {0, 0};
        // bcm2835_gpio_write(CS_PIN, LOW);
        uint8_t wd[2] = {addr & 0xFF, (addr >> 8) & 0xFF};

        bcm2835_spi_transfernb(reinterpret_cast<char*>(wd), reinterpret_cast<char*>(rdat), 2);
        // bcm2835_gpio_write(CS_PIN, HIGH);

        uint16_t rBuf = (rdat[1] << 8) | rdat[0];
        return rBuf;      
    }

    /* 向寄存器写8bit的数据 */
    int adisRegWrite8bit(uint8_t regAddr, int8_t regData){
	    uint16_t txBuf;
	    /* 将寄存器地址和寄存器数据组合成一个 16 位的数据字 */ 
	    txBuf = ((regAddr | 0x80) << 8) | (regData & 0xFF);

	    /* 向外部设备写入数据，将之前组合好的数据字 txBuf 发送到设备 */ 
	    spiEnableNss();
	    spiWriteWord(txBuf);
	    spiDisableNss();

	    bcm2835_delay(tSTALL);

	    return 1;
    }

    /* 向寄存器写16bit的数据 */
    int adisRegWrite16bit(uint8_t regAddr, int16_t regData){
        /* 低8位 */
	    adisRegWrite8bit(regAddr, regData & 0xFF);
        /* 高8位 */
        adisRegWrite8bit(regAddr + 1, ((regData >> 8) & 0xFF));

	    return 1;
    }

    bool adisHardwareFilterSelect(int dat) {
	    // adisRegWrite16bit(FILT_CTRL, dat); // Set digital filter
        adisWriteReg(FILT_CTRL, dat);
	    return true;
    }

    /* 更新IMU的零偏bias */
    void adisBiasCorrectionUpdate(void) {
	    adisRegWrite16bit(GLOB_CMD, 0x0001);
    }

    /* 连续读取所有寄存器的值 */
    int adisPageDump(uint16_t *vals) {
	    uint16_t len=0;
	    for(int i = 0; i <= 0x7E; i += 2){
		    if(i == BURST_CMD){ // Burst command can not be read
			    vals[len] =0;
		    }else{
			    vals[len] = adisBlockingRegRead(i);
		    }
		    len++;
	    }

	    return len;
    }

    /* 读加速度、角速度和温度 */
    void adisRead32bit(uint16_t *rb) {
        int j = 0;
        for(int i=0x04;i<=0x1C;i+=2){
            /* 循环遍历，从X_GYRO_LOW到TEMP_OUT */
		    rb[j++] = adisNoBlockingRegRead(i);
	    }
	    rb[j++] = adisNoBlockingRegRead(0x00);
    }

    /* burst-read读取数据(我的理解就是连续读取) */
    void adisBurstRead32bit(uint16_t *rb) {
        uint8_t rxBuf [ADIS_Burst_Packet_Size];

        spiEnableNss();
        /* Packet size is half of burst message because SPI is configured for 16-bit transactions */
        spiNonBlockingWriteArray(ADIS_Burst_Packet, rxBuf, ADIS_Burst_Packet_Size / 2);
        spiDisableNss();

        /* 只要加速度、陀螺和温度 */
        volatile int cnt = 0;
	    for (int i = 0; i < 26; i+=2) {
		    rb[cnt] = ((rxBuf[i + 5] << 8) | rxBuf[i + 4]);
		    cnt++;
	    }
    }

    /* 获取通过burst-read读取时的加速度、角速度 */
    int adisIMUConvert32bit(uint16_t *buf, double *gyro, double *acc) {
	    //Save raw data of angular velocity and acceleration

	    /* Process XG */
	    gyro_hex[0] = ((buf[1] << 16) | buf[0]);
	    gyro[0] = (int32_t)gyro_hex[0] / GYRO_SENSITIVITY * M_PI /180.0;
	    gyro_raw[0] = gyro[0];

	    /* Process YG */
	    gyro_hex[1] = ((buf[3] << 16) | buf[2]);
	    gyro[1] = (int32_t)gyro_hex[1] / GYRO_SENSITIVITY * M_PI / 180.0;
	    gyro_raw[1] = gyro[1];

	    /* Process ZG */
	    gyro_hex[2] = ((buf[5] << 16) | buf[4]);
	    gyro[2] = (int32_t)gyro_hex[2] / GYRO_SENSITIVITY * M_PI / 180.0;
	    gyro_raw[2] = gyro[2];

	    /* Process XA */
	    acc_hex[0] = ((buf[7] << 16) | buf[6]);
	    acc[0] = (int32_t)acc_hex[0] / ACCL_SENSITIVITY;
	    acc_raw[0] = acc[0] / G_ACCL;

	    /* Process YA */
	    acc_hex[1] = ((buf[9] << 16) | buf[8]);
	    acc[1] = (int32_t)acc_hex[1] / ACCL_SENSITIVITY;
	    acc_raw[1] = acc[1] / G_ACCL;

	    /* Process ZA */
	    acc_hex[2] = ((buf[11] << 16) | buf[10]);
	    acc[2] = (int32_t)acc_hex[2] / ACCL_SENSITIVITY;
	    acc_raw[2] = acc[2] / G_ACCL;

	    double n = sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
	    if(n>0.0001){
		    acc[0]/=n;
		    acc[1]/=n;
		    acc[2]/=n;
	    }else{
		    acc[0]=0;
		    acc[1]=0;
		    acc[2]=0;
	    }
	    return(1);
    }

    /* 返回观测值 */
    void adisGetHexData(uint32_t *p_gyro, uint32_t *p_acc) {
        for (int i = 0; i < 3; i++) {
            p_gyro[i] = gyro_hex[i];
            p_acc[i] = acc_hex[i];
        }
    }

    /* 返回观测值(Raw) */
    void adisGetRawData(uint32_t *p_gyro, uint32_t *p_acc) {
        for (int i = 0; i < 3; i++) {
            p_gyro[i] = gyro_raw[i];
            p_acc[i] = acc_raw[i];
        }
    }

    /* 返回温度观测值 */
    double adisTempConvert(uint16_t *buf) {
        return (double)((int16_t)buf[12]) * 0.1;
    }

    /* 设置采样频率 */
    bool adisSetSampleRate(int16_t n) {
        /* 设置DEC_RATE，也就是控制采样频率 */
        adisWriteReg(DEC_RATE, n);
        // adisRegWrite16bit(DEC_RATE, n);

        /* 输出频率 */
        frequency = 2000 / (1 + n);

        return true;
    }

    /* TODO 陀螺(和加速度计)零偏校准 */
    void adisSelfCalibration() {
        /* 储存二进制当前的误差值 */
        int32_t RawBiasData[6] = {0};

        /* 待写入内存的首地址 和addr一起递增 */
        uint8_t* writeData = (uint8_t*) RawBiasData;

        /* 先执行一次清零 */
        for (uint8_t i = 0; i < 24; i++) {
            /* 每个占2*2=4个字节，6*4就是24 */
            adisRegWrite8bit(XG_BIAS_LOW + i, writeData[i]);
        }

        /* 10秒的采样数 */
        int sum = 10 * frequency;
        uint16_t count = 0;
        uint16_t delay = 1000 / frequency;

        while(count < sum) {
            /* TODO 执行单次的数据读取 */


            bcm2835_delay(delay);
        }

        double gyroData[3];
        gyroData[0] = gyro_raw[0];
        gyroData[1] = gyro_raw[1];
        gyroData[2] = gyro_raw[2];
        float* pointf = (float*)&gyroData;
    }

    /* 向寄存器写值 */
    int8_t adisWriteReg(uint8_t addr,uint8_t value) {
        addr|=0x80;//写数据的掩码
	    uint16_t Tx_tmp=(addr<<8) | value;
	    adisFlameTandR(Tx_tmp);
	    return 0;
    }

    uint16_t adisFlameTandR(uint16_t trans) {
        bcm2835_gpio_write(CS_PIN, LOW);

        uint16_t result = 0;

        bcm2835_spi_transfernb(reinterpret_cast<char*>(&trans), reinterpret_cast<char*>(&result), 2);

        bcm2835_gpio_write(CS_PIN, LOW);
        bcm2835_delay(tSTALL);

        return result;
    }
};

#endif