#include <wiringPiSPI.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#include "spi.h"
#include "constants.h"

using namespace std;

#define SPI_CHANNEL 1

int main() {
    wiringPiSPISetup(SPI_CHANNEL, 500000);

    spi_packet_t test_packet;
    test_packet.message[0] = 0x11;
    test_packet.message[1] = 0x12;
    test_packet.message[2] = 0x13;
    test_packet.message[3] = 0x14;
    test_packet.message[4] = 0x15;
    test_packet.message[5] = 0x16;
    test_packet.message[6] = 0x17;
    test_packet.message[7] = 0x18;

    uint8_t counter = 0;

    while(1) {
        // Write to the EUI register
        writeRegister(SPI_CHANNEL, DWM_REG_EUI, 8, test_packet);

        // Read the EUI back from the device
        sleep(1);
        readRegister(SPI_CHANNEL, DWM_REG_EUI, 8, test_packet);

        printf("Extended ID: ");
        for (int i = 0; i < 8; i++) {
            printf("%2x", test_packet.message[i]);
        }
        printf("\n");

        test_packet.message[0] = 0x11 + counter;
        test_packet.message[1] = 0x12;
        test_packet.message[2] = 0x13;
        test_packet.message[3] = 0x14;
        test_packet.message[4] = 0x15;
        test_packet.message[5] = 0x16;
        test_packet.message[6] = 0x17;
        test_packet.message[7] = 0x18;
        counter++;

        sleep(1);
    }

}