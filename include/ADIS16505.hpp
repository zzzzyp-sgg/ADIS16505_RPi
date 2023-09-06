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

/* 信号拉低拉高 */
inline void CS_LOW() {
    bcm2835_gpio_write(CS_PIN, LOW);
}

inline void CS_HIGH() {
    bcm2835_gpio_write(CS_PIN, HIGH);
}

/* 类型转换 */
template<typename D>
inline char* to_char(D x) {
    return reinterpret_cast<char*>(x);
}

class ADIS16505 {
public:
    /* 角速度和加速度变量 */
    volatile uint32_t gyro_hex[3], acc_hex[3];
    double gyro_raw[3], acc_raw[3];
    double frequency;

public:
    bool setUp() {
        if(!bcm2835_spi_begin()) {
            printf("bcm2835_ispi_begin failed!\n");
            return false;
        }

        /* 初始化设备 */
        bool isInit = initADIS16505();

        /* 判断是否连接到了设备 */
        if (isInit) {
            std::cout << "Connected to SPI device." << std::endl;
        } else {
            std::cout << "Not connected to SPI device." << std::endl;
            return false;
        }

        return true;
    }

    /* 单次处理读取数据 */
    void adisSingleRead() {
        /* 先读取 */
        // adisBurstRead32bit(rb);
        auto filt_ctrl = adisReadReg(X_GYRO_LOW);
        auto x_g_l = adisReadReg(X_GYRO_OUT);
        auto x_g_o = adisReadReg(Y_GYRO_LOW);
        auto y_g_l = adisReadReg(Y_GYRO_OUT);
        auto y_g_o = adisReadReg(Z_GYRO_LOW);
        auto z_g_l = adisReadReg(Z_GYRO_OUT);
        auto z_g_o = adisReadReg(X_ACCL_LOW);
        auto x_a_l = adisReadReg(X_ACCL_OUT);
        auto x_a_o = adisReadReg(Y_ACCL_LOW);
        auto y_a_l = adisReadReg(Y_ACCL_OUT);
        auto y_a_o = adisReadReg(Z_ACCL_LOW);
        auto z_a_l = adisReadReg(Z_ACCL_OUT);
        auto z_a_o = adisReadReg(TEMP_OUT);

        /* 再转换 */
        gyro_raw[0] = ((int32_t(x_g_o) << 16) + int32_t(x_g_l)) * M_PI / 180.0 / GYRO_SENSITIVITY;
        gyro_raw[1] = ((int32_t(y_g_o) << 16) + int32_t(y_g_l)) * M_PI / 180.0 / GYRO_SENSITIVITY;
        gyro_raw[2] = ((int32_t(z_g_o) << 16) + int32_t(z_g_l)) * M_PI / 180.0 / GYRO_SENSITIVITY;
        acc_raw[0] = ((int32_t(x_a_o) << 16) + int32_t(x_a_l)) / ACCL_SENSITIVITY;
        acc_raw[1] = ((int32_t(y_a_o) << 16) + int32_t(y_a_l)) / ACCL_SENSITIVITY;
        acc_raw[2] = ((int32_t(z_a_o) << 16) + int32_t(z_a_l)) / ACCL_SENSITIVITY;
    }
private:
    /* 初始化设备 */
    bool initADIS16505() {
        /* 高位优先 */
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
        /* 模式2,CPOL = 1 (polarity), CPHA = 1 (phase) */
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
        /* 时钟分频 */
        bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);

        /* 选中设备，将这个引脚设置为输出模式 */
        bcm2835_gpio_fsel(CS_PIN, BCM2835_GPIO_FSEL_OUTP);
        // bcm2835_gpio_set(CS_PIN); 这里是设备未选中，我感觉是可以不用的

        /* 设备重启 */
        CS_LOW();
        bcm2835_delay(500);
        CS_HIGH();
        bcm2835_delay(500);

        // 启动后先读PROD_ID
        uint8_t rdat[2] = {0, 0};
        uint8_t wd[2] = {PROD_ID, 0x00};
        CS_LOW();
        bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd)); // 发送和接收数据
        CS_HIGH();
        bcm2835_delayMicroseconds(tSTALL);
        
        bool isConnected = adisIsConnected();
        if (!isConnected) return false;

        /* 设置DEC_RATE(频率) */
        adisSetSampleRate(3);
        
        /* Enable 32-bit burst output */
        // adisSet32bitBurstConfig();

        /* Hard filter setting in sensor */
        adisHardwareFilterSelect(1);

        return true;
    }

    /* 向寄存器写值 */
    void adisWriteReg(uint16_t addr, uint16_t val) {
        uint16_t txBuf1 = ((addr | 0x80) << 8) | (val & 0xFF);
        uint8_t tx_data1[2] = {txBuf1 >> 8, txBuf1 & 0xFF};
        uint16_t txBuf2 = ((addr + 1 | 0x80) << 8) | (0 & 0xFF);
        uint8_t tx_data2[2] = {txBuf2 >> 8, txBuf2 & 0xFF};

        uint8_t rdat[2] = {0, 0};
        CS_LOW();
        bcm2835_spi_transfernb(to_char(tx_data1),to_char(rdat), sizeof(tx_data1));
        CS_HIGH();
        bcm2835_delayMicroseconds(tSTALL);  
        CS_LOW();
        bcm2835_spi_transfernb(to_char(tx_data2),to_char(rdat), sizeof(tx_data2));
        CS_HIGH();
        bcm2835_delayMicroseconds(tSTALL);
    }
    
    /* 通过检查PROD_ID, 判断是否连接成功 */
    bool adisIsConnected() {
        uint16_t prodId = adisReadReg(PROD_ID);
        auto val = (prodId == 16505);
        return val;
    }
    
    /* 读取某个地址的寄存器的值 */
    uint16_t adisReadReg(uint16_t addr) {
        if (addr == BURST_CMD)
            return false;

        uint8_t wd[2] = {addr, 0x00};
        uint8_t rdat[2] = {0, 0};
        CS_LOW();
        bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd));
        CS_HIGH();
        bcm2835_delayMicroseconds(tSTALL);
        uint16_t val = (rdat[0] << 8) | rdat[1];
        return val;
    }

    /* 这里设置的是连续读取得到的是增量还是加速度形式 */
    bool adisSet32bitBurstConfig() {
        uint8_t wd[2] = {MSC_CTRL, 0x00};
        uint8_t rdat[2] = {0, 0};
        CS_LOW();
        bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd));
        CS_HIGH();
        bcm2835_delayMicroseconds(tSTALL);
        uint16_t tmp = (rdat[0] << 8) | rdat[1];
        tmp |= 1 << 9;
	    adisWriteReg(MSC_CTRL, tmp);
        CS_LOW();
        bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd));
        CS_HIGH();
        bcm2835_delayMicroseconds(tSTALL);
        uint16_t tmp_ = adisReadReg(MSC_CTRL);
        // This delay allows the setting to take hold
	    // bcm2835_delay(1);
	    bool res = (tmp_ == tmp);
        return res;
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
        uint8_t wd[2] = {(addr >> 8) & 0xFF, addr & 0xFF};

        CS_LOW();
        bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd));
        CS_HIGH();

        uint16_t rBuf = (rdat[0] << 8) | rdat[1];
        return rBuf;      
    }

    /* 向寄存器写8bit的数据 */
    int adisRegWrite8bit(uint8_t regAddr, int8_t regData){
	    uint16_t txBuf;
	    /* 将寄存器地址和寄存器数据组合成一个 16 位的数据字 */ 
	    txBuf = ((regAddr | 0x80) << 8) | (regData & 0xFF);

	    /* 向外部设备写入数据，将之前组合好的数据字 txBuf 发送到设备 */ 
	    CS_LOW();
	    spiWriteWord(txBuf);
	    CS_HIGH();

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

    bool adisHardwareFilterSelect(uint16_t dat) {
	    // adisRegWrite16bit(FILT_CTRL, dat); // Set digital filter
        adisWriteReg(FILT_CTRL, dat);
        uint8_t wd[2] = {FILT_CTRL, 0x00};
        uint8_t rdat[2] = {0, 0};
        CS_LOW();
        bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd));
        CS_HIGH(); 
        bcm2835_delayMicroseconds(tSTALL);
        auto tmp = adisReadReg(FILT_CTRL);
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
    bool adisSetSampleRate(uint16_t n) {
        /* 设置DEC_RATE，也就是控制采样频率 */
        adisWriteReg(DEC_RATE, n);
        // adisRegWrite16bit(DEC_RATE, n);
        uint8_t wd[2] = {DEC_RATE, 0x00};
        uint8_t rdat[2] = {0, 0};
        CS_LOW();
        bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd));
        CS_HIGH();
        bcm2835_delayMicroseconds(tSTALL);
        auto tmp = adisReadReg(DEC_RATE);

        /* 输出频率 */
        frequency = 2000 / (1 + tmp);

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
};

#endif