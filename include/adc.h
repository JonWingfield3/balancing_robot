/*
 * adc.h
 *
 *  Created on: Oct 14, 2017
 *      Author: jonathanwingfield
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdbool.h>
#include <stdint.h>

#define ADC_REF (3.3)
#define ADC_RES (10)
#define ADC_RES_SCALE (BIT(ADC_RES) - 1)

void adc_init(void);
uint16_t adc_read(void);

#endif /* ADC_H_ */
