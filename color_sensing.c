/*
 * Author: Arnab Dey
 * Description: Basic code to interface flora color sensor
 * with PIC24FJ64GA002
 * I2C1 is used for flora sensor
 */


#include "xc.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "TCS34725.h"

// Basic configuration
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL 

// Global variables
// TODO: Remove later
int gammatable[256];
int baseRed[3] = {255, 0, 0};
int baseGreen[3] = {0, 255, 0};
int baseBlue[3] = {0, 0, 255};

// Creates Gamma table
void setupGammatable(void){
    int i;
    for (i=0; i<256; i++) {
        int x = i;
        x /= 255;
        x = pow(x, 2.5);
        x *= 255;
        gammatable[i] = 255 - x;
  }
}
// Sets up PIC
void setup(){
  CLKDIVbits.RCDIV = 0;
  AD1PCFG = 0xffff;
}
// Initializes I2C1
void i2c1_init() {
    I2C1CONbits.I2CEN = 0;
    I2C1BRG = 157;
    I2C1CONbits.I2CEN = 1;
    IFS1bits.MI2C1IF = 0;
    delay_ms(50);
}

int main(void){
    bool isSensorInitialized = 0;
    setup();
    i2c1_init();
    delay_ms(100);
    isSensorInitialized = sensor_init();
    delay_ms(100);
//    setupGammatable();
    float red, green, blue;
    while(1){
        getRGB(&red, &green, &blue);
        delay_ms(100);
    }
    return 0;
}
    
