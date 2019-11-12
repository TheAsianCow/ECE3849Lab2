/*
 * sampling.c
 *
 *  Created on: Oct 29, 2019
 *      Author: Jeffrey Huang
 *              Ravi Kirschner
 */
#include "inc/tm4c1294ncpdt.h"
#include "sampling.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "Crystalfontz128x128_ST7735.h"
#include "sysctl_pll.h"
#include "math.h"

volatile uint32_t gADCErrors; // number of missed ADC deadlines
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1; // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];

void ADC1_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; //round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCSequenceDisable(ADC1_BASE, 0); // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // specify the "Always" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);// in the 0th step, sample channel 3 (AIN3)
     // enable interrupt, and make it the end of sequence
    ADCSequenceEnable(ADC1_BASE, 0); // enable the sequence. it is now sampling
    ADCIntEnable(ADC1_BASE, 0); // enable sequence 0 interrupt in the ADC1 peripheral
    IntPrioritySet(INT_ADC1SS0, ADC1_INT_PRIORITY); // set ADC1 sequence 0 interrupt priority
    IntEnable(INT_ADC1SS0); // enable ADC1 sequence 0 interrupt in int. controller
}

void ADC_ISR(void){
    ADC1_ISC_R = ADC_ISC_IN0; // clear ADC1 sequence0 interrupt flag in the ADCISC register
     if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
         gADCErrors++; // count errors
         ADC1_OSTAT_R = ADC_OSTAT_OV0; // clear overflow condition
     }
     gADCBuffer[gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)] = ADC1_SSFIFO0_R & ADC_SSFIFO0_DATA_M; // read sample from the ADC1 sequence 0 FIFO
}

int getTriggerIndex(int triggerDirection) {
    int i;
    int tolerence = 25;

    for(i = 64; i < ADC_BUFFER_SIZE/2; i++) {
        int index = ADC_BUFFER_WRAP(gADCBufferIndex-i);
        if(gADCBuffer[index] >= ADC_OFFSET-tolerence && gADCBuffer[index] <= ADC_OFFSET+tolerence) {
            bool dir = gADCBuffer[ADC_BUFFER_WRAP(index+2)] > gADCBuffer[ADC_BUFFER_WRAP(index-2)];
            if ((triggerDirection && dir) || (!triggerDirection && !dir)) return index;
        }
    }
    return -1;
}

int voltageScale(uint16_t voltage, float div) {
    float x = VIN_RANGE * PIXELS_PER_DIV;
    float fScale = x/((1 << ADC_BITS) * div);
    return LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)voltage - ADC_OFFSET));
}
