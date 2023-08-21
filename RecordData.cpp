/**
 * @brief   记录IMU原始的观测数据
 * @author  zyp
 * @date    2023-08-08
*/

#include "ADIS16505.h"
#include <fstream>
#include <cstring>
#include <iostream>

int main(int argc, char **argv) {
    ADIS16505 adi;
    if (!bcm2835_init()) {
        std::cout << "bcm2835 init failded, please check whether you are running as root!" << std::endl;
        return 0;
    }

    if(!adi.setUp()) {
        printf("ADI connection failed.\n");
        bcm2835_delay(5000);
    } else {
        uint16_t rbuf[18];
	    memset(rbuf,0,sizeof(rbuf));

        adi.adisSingleRead(rbuf);

        std::cout << "test: x_acc = " << adi.acc_raw[0] << std::endl;

        bcm2835_gpio_set(CS_PIN);
        /* 清除工作区 */
        bcm2835_spi_end();
        bcm2835_close();
    }

    return 0;
}