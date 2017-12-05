/*
 * i2c.h
 * Author: jonathanwingfield
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdbool.h>
#include <stdint.h>

void i2c_init(void);

int i2c_write(uint8_t slave_addr, uint8_t* data, uint8_t n, bool stop);
int i2c_read(uint8_t slave_addr, uint8_t* data, uint8_t n);
bool i2c_busy(void);

void I2C_IRQHandler(void);

#endif /* I2C_H_ */
