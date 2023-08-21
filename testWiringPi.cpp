#include <cstdio>
#include <cstring>
#include <cstdint>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define CS_PIN      11
#define PORD_ID     0x72

#define SPI_CHANNEL 0
#define SPI_SPEED   500000
#define SPI_MODE    3

int main(int argc, char **argv) {  
    wiringPiSetup();

    // pinMode(CS_PIN, OUTPUT);
    // digitalWrite(CS_PIN, HIGH);

    if (wiringPiSPISetupMode(SPI_CHANNEL, SPI_SPEED, SPI_MODE) == -1) {
        printf("SPI initialize failed.\n");
    }

    pinMode(CS_PIN, OUTPUT);
    
    /* 重启设备 */
    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(1000);
    digitalWrite(CS_PIN, HIGH);
    delayMicroseconds(1000);

    digitalWrite(CS_PIN, LOW);
    // Read PROD_ID register
    uint8_t rdat[2] = {0, 0};
    uint16_t addr = 0x72;
    addr = addr << 8;
    uint8_t wd[2] = {static_cast<uint8_t>(addr & 0xFF), static_cast<uint8_t>((addr >> 8) & 0xFF)};

    int status = wiringPiSPIDataRW(SPI_CHANNEL, wd, 2);  // 发送和接收数据

    digitalWrite(CS_PIN, HIGH);

    int pord_id = (rdat[1] << 8) | rdat[0];
    printf("Value is: %d\n", pord_id);

    return 0;
}