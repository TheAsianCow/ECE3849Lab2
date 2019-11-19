/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"
#include "sampling.h"

#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"
#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency

int trigger = 0;
int divNumber = 1;
uint16_t triggerDir = 1;
uint16_t mode = 0;
const char* slope[] = {"DOWN","UP"};
const float divArray[] = {0.1,0.2, 0.5, 1};
tContext sContext;
uint16_t ADC_local[128];
uint16_t scaledWave[128];
uint16_t fft_local[NFFT];

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // hardware initialization goes here

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    ADC1_Init();
    ButtonInit();

    IntMasterEnable();
    /* Start BIOS */
    BIOS_start();

    return (0);
}

void Clock(UArg arg){
    Semaphore_post(ButtonSem);
}

void UI_Task(UArg arg0, UArg arg1){
    char data = 'A';
    while(1){
        Mailbox_pend(ButtonBox, &data, BIOS_WAIT_FOREVER);
        switch(data) {
        case 'D':
            divNumber = divNumber > 0 ? divNumber-1 : divNumber;
            break;
        case 'U':
            divNumber = divNumber < 3 ? divNumber+1 : divNumber;
            break;
        case 'T':
            triggerDir = triggerDir == 1 ? 0 : 1;
            break;
        case 'F':
            mode = mode == 1 ? 0 : 1;
            break;
        }
        Semaphore_post(DisplaySem);
    }
}

void Display_Task(UArg arg0, UArg arg1) {
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};
    char div_str[10];
    int i = 0;
    while(1) {
        Semaphore_pend(DisplaySem, BIOS_WAIT_FOREVER);
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
        GrContextForegroundSet(&sContext, ClrBlue); // blue lines
        for(i = -3; i < 4; i++){
            GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX-1,LCD_VERTICAL_MAX/2+i*PIXELS_PER_DIV);
            GrLineDrawV(&sContext, LCD_HORIZONTAL_MAX/2+i*PIXELS_PER_DIV, 0,LCD_VERTICAL_MAX-1);
        }
        if(!mode) {
            if(divNumber == 3) {
                snprintf(div_str, sizeof(div_str), " 1 V");
            }
            else {
                snprintf(div_str, sizeof(div_str), "%03d mV", (int)(divArray[divNumber]*1000));
            }


            GrContextForegroundSet(&sContext, ClrWhite); // white text
            GrStringDraw(&sContext, "20 us", -1, 4, 0, /*opaque*/ false);
            GrStringDraw(&sContext, div_str, -1, 45, 0, /*opaque*/ false);
            GrStringDraw(&sContext, slope[triggerDir], -1, 100, 0, false);
        }
        else {
            GrContextForegroundSet(&sContext, ClrWhite); // white text
            GrStringDraw(&sContext, "20 kHz", -1, 0, 0, false);
            GrStringDraw(&sContext, "20 dB", -1, 45, 0, false);
        }

        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
        for(i = 0; i < 127; i++) GrLineDraw(&sContext, i, scaledWave[i], i+1, scaledWave[i+1]);
        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}

void Waveform_Task(UArg arg0, UArg arg1) {
    int i = 0;
    while(1) {
        Semaphore_pend(WaveformSem, BIOS_WAIT_FOREVER);
        if(!mode) {
            trigger = getTriggerIndex(triggerDir);
            for(i = -64; i < 64; i++) {
                ADC_local[i+64] = gADCBuffer[ADC_BUFFER_WRAP(trigger+i)];
            }
        }
        else {
            for(i = 0; i < NFFT; i++) {
                fft_local[i] = gADCBuffer[ADC_BUFFER_WRAP(gADCBufferIndex-NFFT+i)];
            }
        }
        Semaphore_post(ProcessingSem);
    }
}

void Processing_Task(UArg arg0, UArg arg1) {
    int i = 0;
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    while(1) {
        Semaphore_pend(ProcessingSem, BIOS_WAIT_FOREVER);
        if(!mode) {
            for(i = 0; i < 128; i++) {
                scaledWave[i] = voltageScale(ADC_local[i], divArray[divNumber]);
            }
        }
        else {
            for (i = 0; i < NFFT; i++) { // generate an input waveform
                in[i].r = fft_local[i]*3.3f/4096.f; // real part of waveform
                in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT
            // convert first 128 bins of out[] to dB for display
            for(i = 1; i < 129; i++) {
                scaledWave[i-1] = 84-(int)roundf(log10f(sqrtf(out[i].r*out[i].r+out[i].i*out[i].i))*20.f);
            }
        }
        Semaphore_post(DisplaySem);
        Semaphore_post(WaveformSem);
    }
}
