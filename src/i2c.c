/*
 * i2c.c
 *
 *  Created on: Oct 27, 2017
 *      Author: jonathanwingfield
 */

#include "../include/i2c.h"

#include <stdbool.h>
#include <stdint.h>

#include "../include/LPC11xx.h"
#include "../include/scheduler.h"
#include "../include/utils.h"

#define I2C_BUS_FREQ (100000)
#define I2C_TIMEOUT_MS (5)

#define AA_FLAG BIT(2)
#define SI_FLAG BIT(3)
#define STOP_FLAG BIT(4)
#define START_FLAG BIT(5)
#define I2EN_FLAG BIT(6)

#define W_BIT (0x00)
#define R_BIT (0x01)

typedef enum {
	TRANSACTION_PENDING = 1, TRANSACTION_ERROR = -1, TRANSACTION_SUCCESS = 0
} i2c_transaction_status_t;

typedef enum {
	I2C_READ, I2C_WRITE
} i2c_direction_t;

static volatile uint8_t* buf_;
static volatile uint8_t byte_cntr_;
static volatile uint8_t slave_addr_;
static volatile bool transmit_end_stop_ = false;
static volatile uint8_t i2c_transaction_state_;
static volatile i2c_transaction_status_t i2c_transaction_status_;

static void i2c_reset(void) {
	// reset i2c module
	LPC_SYSCON->PRESETCTRL &= ~BIT(1);
	LPC_SYSCON->PRESETCTRL |= BIT(1);
	LPC_I2C->CONCLR = 0xff;
	const float pclk_freq = (float) SystemCoreClock;
	LPC_I2C->SCLL = (uint32_t) (pclk_freq / I2C_BUS_FREQ / 2.0);
	LPC_I2C->SCLH = (uint32_t) (pclk_freq / I2C_BUS_FREQ / 2.0);
	LPC_I2C->CONSET = I2EN_FLAG;
}

void i2c_init(void) {
	// clear reset signal to i2c module
	LPC_SYSCON->PRESETCTRL |= BIT(1);
	// enable clock to i2c.
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(5);
	// ioconfig pin muxing
	LPC_IOCON->PIO0_4 = BIT(0);
	LPC_IOCON->PIO0_5 = BIT(0);
	// clear any existing state from control register.
	LPC_I2C->CONCLR = 0xff;
	const float pclk_freq = (float) SystemCoreClock;
	LPC_I2C->SCLL = (uint32_t) (pclk_freq / I2C_BUS_FREQ / 2.0);
	LPC_I2C->SCLH = (uint32_t) (pclk_freq / I2C_BUS_FREQ / 2.0);
	NVIC_EnableIRQ(I2C_IRQn);
	NVIC_SetPriority(I2C_IRQn, 0);
	LPC_I2C->CONSET = I2EN_FLAG;
}

int i2c_write(uint8_t slave_addr, uint8_t* data, uint8_t n, bool stop) {
	i2c_transaction_status_ = TRANSACTION_PENDING;
	buf_ = data;
	byte_cntr_ = n;
	transmit_end_stop_ = stop;
	slave_addr_ = (slave_addr << 1) | W_BIT;
	LPC_I2C->CONSET = START_FLAG;
	const uint32_t start_time = scheduler_get_system_time();
	uint32_t current_time = start_time;
	while ((current_time - start_time) < I2C_TIMEOUT_MS) {
		if (i2c_transaction_status_ != TRANSACTION_PENDING) {
			return i2c_transaction_status_;
		} else {
			current_time = scheduler_get_system_time();
		}
	}
	i2c_reset();
	return TRANSACTION_ERROR;
}

int i2c_read(uint8_t slave_addr, uint8_t* buf, uint8_t n) {
	i2c_transaction_status_ = TRANSACTION_PENDING;
	buf_ = buf;
	byte_cntr_ = n;
	slave_addr_ = (slave_addr << 1) | R_BIT;
	LPC_I2C->CONSET = START_FLAG;
	const uint32_t start_time = scheduler_get_system_time();
	uint32_t current_time = start_time;
	while ((current_time - start_time) < I2C_TIMEOUT_MS) {
		if (i2c_transaction_status_ != TRANSACTION_PENDING) {
			return i2c_transaction_status_;
		} else {
			current_time = scheduler_get_system_time();
		}
	}
	i2c_reset();
	return TRANSACTION_ERROR;
}

bool i2c_busy(void) {
	return (byte_cntr_ != 0);
}

void I2C_IRQHandler(void) {
	volatile uint8_t scratch; // byte to read junk data.
	const volatile uint8_t status = LPC_I2C->STAT >> 3;
	switch (status) {
	case 0: // i2c bus error.
		LPC_I2C->CONSET = (STOP_FLAG | AA_FLAG);
		i2c_transaction_status_ = TRANSACTION_ERROR;
		break;
	case 1: // START condition has just been transmitted
		// happens before read or write so must know which one is intended.
		LPC_I2C->CONCLR = START_FLAG;
		LPC_I2C->DAT = slave_addr_;
		break;
	case 2: // Repeated START condition has been sent.
		LPC_I2C->CONCLR = START_FLAG;
		LPC_I2C->DAT = slave_addr_;
		break;
	case 3: // slave_addr | W_BIT has been transmitted. ACK recvd
		LPC_I2C->DAT = *buf_;
		--byte_cntr_;
		++buf_;
		break;
	case 4: // slave_addr | W_BIT has been transmitted. NACK recvd.
		LPC_I2C->CONSET = (AA_FLAG | STOP_FLAG);
		i2c_transaction_status_ = TRANSACTION_ERROR;
		break;
	case 5: // data_byte has been transmitted. ACK has been recvd.
		if (byte_cntr_ == 0) {
			i2c_transaction_status_ = TRANSACTION_SUCCESS;
			if (transmit_end_stop_) {
				LPC_I2C->CONSET = STOP_FLAG;
			}
		} else {
			LPC_I2C->DAT = *buf_;
			--byte_cntr_;
			++buf_;
		}
		break;
	case 6: // data_byte has been transmitted. NACK has been recvd.
		LPC_I2C->CONSET = (AA_FLAG | STOP_FLAG);
		i2c_transaction_status_ = TRANSACTION_ERROR;
		break;
	case 7: // arbitration lost during transmission.
		LPC_I2C->CONSET = (AA_FLAG | START_FLAG);
		i2c_transaction_status_ = TRANSACTION_ERROR;
		break;
	case 8: // slave address | R_BIT has just been transmitted, ACK has been recvd
		LPC_I2C->CONSET = AA_FLAG;
		break;
	case 9: // slave address | R_BIT has just been transmitted, NACK has been recvd
		LPC_I2C->CONSET = (AA_FLAG | STOP_FLAG);
		i2c_transaction_status_ = TRANSACTION_ERROR;
		break;
	case 10: // data byte has just been recvd and an ACK has been returned.
		*buf_ = LPC_I2C->DAT;
		--byte_cntr_;
		++buf_;
		if (byte_cntr_ == 0) {
			LPC_I2C->CONCLR = AA_FLAG;
			//i2c_transaction_status_ = TRANSACTION_SUCCESS;
		} else {
			LPC_I2C->CONSET = AA_FLAG;
		}
		break;
	case 11: // data byte has been recvd and a NACK has been returned.
		scratch = LPC_I2C->DAT;
		LPC_I2C->CONSET = (STOP_FLAG | AA_FLAG);
		i2c_transaction_status_ = TRANSACTION_SUCCESS;
		break;
	default: // unknown state.
		break;
	}
	i2c_transaction_state_ = status;
	LPC_I2C->CONCLR = SI_FLAG;
}

/*	while (true) {
 for (volatile int i = 0; i < 1000000; ++i) {
 if (i2c_transaction_status_ != TRANSACTION_PENDING) {
 return i2c_transaction_status_;
 }
 }
 i2c_reset();
 return TRANSACTION_ERROR;
 }*/

//	while (i2c_transaction_status_ == TRANSACTION_PENDING) {
//	}
//	byte_cntr_ = 0;
//	return i2c_transaction_status_;
