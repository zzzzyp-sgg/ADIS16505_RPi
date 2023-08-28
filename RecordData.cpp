/**
 * @brief   记录IMU原始的观测数据
 * @author  zyp
 * @date    2023-08-08
*/

#include "ADIS16505.hpp"
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
        bcm2835_delay(500);
    } else {
        adi.adisSingleRead();
        bcm2835_gpio_set(CS_PIN);
        /* 清除工作区 */
        bcm2835_spi_end();
        bcm2835_close();
    }

    return 0;
}