/*
 * File:   TCD34725.c
 * Author: Arnab Dey
 * Description: Flora color sensor library
 * I2C1 is used to interface the sensor
 * Created on April 30, 2022, 3:22 AM
 */
#include "xc.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "TCS34725.h"

static uint8_t is_rep_start_state = 0;

void write(const uint8_t *buffer, size_t len, bool stop){
    // Send start
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN == 1);
    IFS1bits.MI2C1IF = 0;
    // Select slave
    I2C1TRN = (TCS34725_ADDRESS << 1) | I2C_WRITE;
    while(IFS1bits.MI2C1IF == 0);
    IFS1bits.MI2C1IF = 0;
    // Write prefix data: TODO: Check len and send 
//    I2C1TRN = *prefix_buffer | TCS34725_COMMAND_BIT;
//    while(IFS1bits.MI2C1IF == 0);
//    IFS1bits.MI2C1IF = 0;
    size_t i;
    for(i = 0; i < len; ++i){
        // Write data itself
        I2C1TRN = buffer[i];
        while(IFS1bits.MI2C1IF == 0);
        IFS1bits.MI2C1IF = 0;
    }
    
    if (stop == true) {
        is_rep_start_state = 0;
        // End transmission
        I2C1CONbits.PEN = 1;
        while(I2C1CONbits.PEN == 1);
    } else {
        is_rep_start_state = 1;
    }
}

void read(uint8_t *buffer, size_t len, bool stop) {
    IFS1bits.MI2C1IF = 0;
    // Check if we are in repeated start state
    if (is_rep_start_state == 1) {
        // Send repeated start state
        I2C1CONbits.RSEN = 1;
        while(I2C1CONbits.RSEN == 1);
        IFS1bits.MI2C1IF = 0;
        is_rep_start_state = 0;
    } else {
        // Send start
        I2C1CONbits.SEN = 1;
        while(I2C1CONbits.SEN == 1);
        IFS1bits.MI2C1IF = 0;
    }
    // Select slave with read
    I2C1TRN = (TCS34725_ADDRESS << 1) | I2C_READ;
    while(IFS1bits.MI2C1IF == 0);
    IFS1bits.MI2C1IF = 0;
    
    // Receive data
    size_t i;
    for(i = 0; i < len; ++i){
        I2C1CONbits.RCEN = 1;
        while(I2C1CONbits.RCEN == 1);
        buffer[i] = I2C1RCV;
        while(IFS1bits.MI2C1IF == 0);
        IFS1bits.MI2C1IF = 0;
        if (i == len-1) {
            I2C1CONbits.ACKDT = 1;
        } else {
            I2C1CONbits.ACKDT = 0;
        }
        I2C1CONbits.ACKEN = 1;
        while(IFS1bits.MI2C1IF == 0);
        IFS1bits.MI2C1IF = 0;
    }
    
    if (stop == true) {
        // Send stop
        I2C1CONbits.PEN = 1;
        while(I2C1CONbits.PEN == 1);
        is_rep_start_state = 0;
    } else {
        is_rep_start_state = 1;
    }
}

void write_then_read(const uint8_t *write_buffer, size_t write_len,
        uint8_t *read_buffer, size_t read_len, bool stop) {
    write(write_buffer, write_len, stop);
    read(read_buffer, read_len, true);
}

void write8(uint8_t reg, uint32_t value) {
    uint8_t buffer[2] = {TCS34725_COMMAND_BIT | reg, value & 0xFF};
    write(buffer, 2, true);
}

uint8_t read8(uint8_t reg) {
  uint8_t buffer[1] = {TCS34725_COMMAND_BIT | reg};
  write_then_read(buffer, 1, buffer, 1, false);
  return buffer[0];
}

uint16_t read16(uint8_t reg) {
  uint8_t buffer[2] = {TCS34725_COMMAND_BIT | reg, 0};
  write_then_read(buffer, 1, buffer, 2, false);
  return ((uint16_t)(buffer[1]) << 8) | ((uint16_t)(buffer[0]) & 0xFF);
}

void setIntegrationTime(uint8_t it) {
  /* Update the timing register */
  write8(TCS34725_ATIME, it);
}

void setGain(uint8_t gain) {
  /* Update the timing register */
  write8(TCS34725_CONTROL, gain);
}

void sensor_enable() {
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    delay_ms(3);
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    /* Set a delay for the integration time.
    This is only necessary in the case where enabling and then
    immediately trying to read values back. This is because setting
    AEN triggers an automatic integration, so if a read RGBC is
    performed too quickly, the data is not yet valid and all 0's are
    returned */
    /* 12/5 = 2.4, add 1 to account for integer truncation */
    delay_ms((256 - TCS34725_INTEGRATIONTIME_50MS) * 12 / 5 + 1);
}

void sensor_disable() {
    /* Turn the device off to save power */
    uint8_t reg = 0;
    reg = read8(TCS34725_ENABLE);
    write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}


bool sensor_init() {
    // First read the device ID
    uint8_t id = read8(TCS34725_ID);
    if (id != 0x44) {
        return false;
    }
    // Set integration time and gain
    setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
    setGain(TCS34725_GAIN_4X);
    // Enable device
    sensor_enable();
    return true;
}

void getRawData(uint16_t *r, uint16_t *g, uint16_t *b,
                                   uint16_t *c) {

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);

  /* Set a delay for the integration time */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
  delay_ms((256 - TCS34725_INTEGRATIONTIME_50MS) * 12 / 5 + 1);
}
void getRGB(float *r, float *g, float *b) {
  uint16_t red, green, blue, clear;
  getRawData(&red, &green, &blue, &clear);
  uint32_t sum = clear;

  // Avoid divide by zero errors ... if clear = 0 return black
  if (clear == 0) {
    *r = *g = *b = 0;
    return;
  }

  *r = (float)red / sum * 255.0;
  *g = (float)green / sum * 255.0;
  *b = (float)blue / sum * 255.0;
}

uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b) {
  float X, Y, Z; /* RGB to XYZ correlation      */
  float xc, yc;  /* Chromaticity co-ordinates   */
  float n;       /* McCamy's formula            */
  float cct;

  if (r == 0 && g == 0 && b == 0) {
    return 0;
  }

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct =
      (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/*!
 *  @brief  Converts the raw R/G/B values to color temperature in degrees
 *          Kelvin using the algorithm described in DN40 from Taos (now AMS).
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @param  c
 *          Clear channel value
 *  @return Color temperature in degrees Kelvin
 */
uint16_t calculateColorTemperature_dn40(uint16_t r, uint16_t g,
        uint16_t b, uint16_t c) {
  uint16_t r2, b2; /* RGB values minus IR component */
  uint16_t sat;    /* Digital saturation level */
  uint16_t ir;     /* Inferred IR content */

  if (c == 0) {
    return 0;
  }

  /* Analog/Digital saturation:
   *
   * (a) As light becomes brighter, the clear channel will tend to
   *     saturate first since R+G+B is approximately equal to C.
   * (b) The TCS34725 accumulates 1024 counts per 2.4ms of integration
   *     time, up to a maximum values of 65535. This means analog
   *     saturation can occur up to an integration time of 153.6ms
   *     (64*2.4ms=153.6ms).
   * (c) If the integration time is > 153.6ms, digital saturation will
   *     occur before analog saturation. Digital saturation occurs when
   *     the count reaches 65535.
   */
  if ((256 - TCS34725_INTEGRATIONTIME_50MS) > 63) {
    /* Track digital saturation */
    sat = 65535;
  } else {
    /* Track analog saturation */
    sat = 1024 * (256 - TCS34725_INTEGRATIONTIME_50MS);
  }

  /* Ripple rejection:
   *
   * (a) An integration time of 50ms or multiples of 50ms are required to
   *     reject both 50Hz and 60Hz ripple.
   * (b) If an integration time faster than 50ms is required, you may need
   *     to average a number of samples over a 50ms period to reject ripple
   *     from fluorescent and incandescent light sources.
   *
   * Ripple saturation notes:
   *
   * (a) If there is ripple in the received signal, the value read from C
   *     will be less than the max, but still have some effects of being
   *     saturated. This means that you can be below the 'sat' value, but
   *     still be saturating. At integration times >150ms this can be
   *     ignored, but <= 150ms you should calculate the 75% saturation
   *     level to avoid this problem.
   */
  if ((256 - TCS34725_INTEGRATIONTIME_50MS) <= 63) {
    /* Adjust sat to 75% to avoid analog saturation if atime < 153.6ms */
    sat -= sat / 4;
  }

  /* Check for saturation and mark the sample as invalid if true */
  if (c >= sat) {
    return 0;
  }

  /* AMS RGB sensors have no IR channel, so the IR content must be */
  /* calculated indirectly. */
  ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;

  /* Remove the IR component from the raw RGB values */
  r2 = r - ir;
  b2 = b - ir;

  if (r2 == 0) {
    return 0;
  }

  /* A simple method of measuring color temp is to use the ratio of blue */
  /* to red light, taking IR cancellation into account. */
  uint16_t cct = (3810 * (uint32_t)b2) / /** Color temp coefficient. */
                     (uint32_t)r2 +
                 1391; /** Color temp offset. */

  return cct;
}

/*!
 *  @brief  Converts the raw R/G/B values to lux
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return Lux value
 */
uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}

/*!
 *  @brief  milliseconds delay
 *  @param  ms
 *          Amount of delay in ms
 */
void delay_ms(uint16_t ms){
    while(ms > 0){
    asm("repeat #15993");
    asm("nop");
    ms--;
    }
}
