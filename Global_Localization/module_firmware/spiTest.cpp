#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <thread>

#include "spi.h"
#include "constants.h"

using namespace std;

#define DWM_CHANNEL 0
#define DWM_INT_PIN 4

void DWM_ISR() {
    printf("Interrupt received\n");
}

void txWaitThread() {
    spi_packet_t status_packet;

    while(1) {
        readRegister(DWM_CHANNEL, DWM_REG_SYS_STATUS, 5, &status_packet);
        if (status_packet.message[0] & DWM_STATUS_TXFRS) {
            printf("Sent message\n");
            status_packet.message[0] |= DWM_STATUS_TXFRS; // Clear bit
            writeRegister(DWM_CHANNEL, DWM_REG_SYS_STATUS, 5, &status_packet);
        }
    }
}

int main() {
    // Initialization
    wiringPiSetup();
    wiringPiSPISetup(DWM_CHANNEL, 500000);

    // Create thread to check status of transmitted message
    std::thread txThread(txWaitThread);

    uint8_t buf[6] = {0};
    buf[0] = 'h';
    buf[1] = 'e';
    buf[2] = 'l';
    buf[3] = 'l';
    buf[4] = 'o';
    buf[5] = '\n';

    while(1) {
        transmitMessage(DWM_CHANNEL, 6, buf);
        sleep(1);
    }

}
