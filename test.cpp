/**
 * @brief   测试bcm2835库使用
 * @author  zyp
 * @date    2023-08-08
*/

#include <iostream>
#include <unistd.h>
#include <bcm2835.h>
 
using namespace std;
 
int main(int argc, char **argv){
    if (!bcm2835_init()){
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }
 
    if (!bcm2835_spi_begin()){
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      return 1;
    }
 
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // the default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // the default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // the default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // the default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
    
    // the sending data and reading data
    uint8_t send_data = 0x23;                                      
    uint8_t read_data = bcm2835_spi_transfer(send_data);
 
    //print the MOSI MISO message
    printf("Sent to SPI: 0x%02X. Read back from SPI: 0x%02X.\n", send_data, read_data);
 
    //if you don't short the MOSI and MISO
    if (send_data != read_data)    
      printf("Do you have the loopback from MOSI to MISO connected?\n");
 
    //colse the device
    bcm2835_spi_end();
    bcm2835_close();
 
    return 0;
}