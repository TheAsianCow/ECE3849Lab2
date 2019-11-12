/*
 * sampling.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Jeffrey Huang
 *              Ravi Kirschner
 */

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <stdint.h>

#define ADC1_INT_PRIORITY 0


#define ADC_SAMPLING_RATE 1000000   // [samples/sec] desired ADC sampling rate
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates
#define ADC_OFFSET 2048
#define VIN_RANGE 3.3
#define PIXELS_PER_DIV 20
#define ADC_BITS 12


void ADC1_Init(void);
int getTriggerIndex(int triggerDirection);
int voltageScale(uint16_t voltage, float div);
#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer


#endif /* SAMPLING_H_ */
