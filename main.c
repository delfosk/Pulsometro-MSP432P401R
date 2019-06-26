/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//***************************************************************************************
//  Blink the LED Demo - Software Toggle P1.0
//
//  Description; Toggle P1.0 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP432P4xx
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.0|-->LED
//
//  E. Chen
//  Texas Instruments, Inc
//  March 2015
//  Built with Code Composer Studio v6
//***************************************************************************************
#include <math.h>
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/driverlib/adc14.h>
#include <stdint.h>
#include <arm_math.h>
#include <uart.h>

#include <arm_const_structs.h>
#define segundos 2
#define frecuenciaMuestreo 100
#define muestras 60
#define muestrasdinamicas 600
#define muestrasfft 1024
uint16_t estado=0;
volatile arm_status status;
float hann[muestrasfft];


int16_t data_input[muestrasfft];
int16_t data_array1[muestrasfft*2];
int16_t data_output[muestrasfft];

void InitTimer() {
    Timer32_initModule(TIMER32_0_BASE,
                       TIMER32_PRESCALER_1,
                       TIMER32_32BIT,
                       TIMER32_PERIODIC_MODE);
}
void Timer1msStartOneShot() {
    Timer32_setCount(TIMER32_0_BASE, 3000000/frecuenciaMuestreo);   // 20 Hz(150000) (100 microsecond period) Fclock 3MHZ
    Timer32_startTimer(TIMER32_0_BASE, false);
    NVIC_EnableIRQ(T32_INT1_IRQn);

}

void dinamicpulse(unsigned vx){
    int i;
    int64_t sum = 0;
    int media=0;
    static int booleanFirstValue =0;
    static unsigned count=0;
    static unsigned buffetTmp[muestrasdinamicas];
    static int firstTime=0;
    buffetTmp[count++] = vx;
    if(firstTime==0 ){
        if (count>100){
            for(i = 0; i < count; i++)
                sum += buffetTmp[i] *buffetTmp[i];
            media=sqrt(sum / count);
            if(vx> media*1.1)
            {
                if (booleanFirstValue==0)
                {
                    booleanFirstValue=1;
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                }
            }
            else{
                booleanFirstValue=0;
                GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
            }
            if (count==muestrasdinamicas){
                count =0;
                firstTime=1;
            }
        }
    }else{
        sum = 0;
        for(i = 0; i < muestrasdinamicas; i++)
            sum += buffetTmp[i] * buffetTmp[i];
        media=sqrt(sum / muestrasdinamicas);
        if(vx> media*1.1)
        {
            if (booleanFirstValue==0)
            {
                booleanFirstValue=1;
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }
        }
        else{
            booleanFirstValue=0;
            GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        }
        if (count==muestrasdinamicas){
            count =0;
        }
    }
}
int rms(unsigned *v )
{
    int i;
    int64_t sum = 0;
    for(i = 0; i < muestras; i++)
        sum += v[i] * v[i];
    return sqrt(sum / muestras);
}


int contarpicos(unsigned *v,int rms )
{
    int i;
    int picos=0;
    int booleanFirstValue =0;

    for(i = 0; i < muestras; i++)
    {
        if(v[i]>rms*1.1)
        {
            if (booleanFirstValue==0)
            {
                booleanFirstValue=1;
                picos++;
            }

        }else if(v[i]<rms)
        {
            booleanFirstValue=0;
        }

    }
    return picos;
}


void CountPulse(unsigned vx) {
    static unsigned count=0;
    static unsigned buffetTmp[muestras];
    char indexString[3];
    if (count==muestras)
    {
        int mediarms=rms(buffetTmp);
        int picos =contarpicos(buffetTmp, mediarms);
        count =0;
        itoa(picos, indexString, 10);
        UART_putchar_n("El pulso con analisis temporal es : " ,36);
        UART_putchar_n(indexString, strlen(indexString));
        UART_putchar_n(" BPM \r\n" ,8);

    }
    buffetTmp[count++] = vx;

}

int dofft(unsigned *v )
{
    int i;
    int32_t sum=0;
    for(i = 0; i < muestrasfft; i++){
        sum += (int32_t)v[i];
    }
    int16_t media=sum/muestrasfft;
    for(i = 0; i < muestrasfft; i++)
    {
        data_input[i]=(int16_t)v[i]-media;
    }
    for(i = 0; i < muestrasfft; i++)
    {
        data_input[i] = (int16_t)(hann[i] * (int16_t)data_input[i]);
    }
    arm_rfft_instance_q15 instance;
    status = arm_rfft_init_q15(&instance, muestrasfft, 0,1);
    arm_rfft_q15(&instance, data_input, data_array1);
    /* Calculate magnitude of FFT complex output */
    for(i = 0; i < muestrasfft; i += 2)
    {
        data_output[i/2] = (int32_t)(sqrtf((data_array1[i] * data_array1[i]) + (data_array1[i + 1] * data_array1[i + 1])));
    }
    int out=0;
    int outi=0;
    for(i = 0; i < 41; i ++)
    {
        if (data_output[i]>out){
            out=data_output[i];
            outi=i;
        }
    }
    return outi*frecuenciaMuestreo*60/muestrasfft;


}
int FftPulse(unsigned vx) {
    static unsigned count=0;
    int pulso=0;
    static unsigned bufferfft[muestrasfft];
    char indexString[3];
    if (count==muestrasfft)
    {
        pulso= dofft(bufferfft);
        count =0;
        itoa(pulso, indexString, 10);
        UART_putchar_n("El pulso con analisis frecucial es : " ,37);
        UART_putchar_n(indexString, strlen(indexString));
        UART_putchar_n(" BPM \r\n" ,8);
    }
    bufferfft[count++] = vx;
    return pulso;

}





void initADC() {
    ADC14_enableModule();

    // This sets the conversion clock to 3MHz
    ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC,
                     ADC_PREDIVIDER_1,
                     ADC_DIVIDER_1,
                     0
    );

    ADC14_setResolution(ADC_14BIT);

    // This configures the ADC to store output results
    // in ADC_MEM0 (single-channel conversion, repeat mode)
    ADC14_configureSingleSampleMode(ADC_MEM0, true);

    // This configures the ADC in automatic conversion mode
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
}


void startADC() {
    // The ADC is in continuous sampling mode, so after calling this once
    // the ADC will continuously update
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();
}


void initSensor() {
    // This configures ADC_MEM0 to store the result from
    // input channel A10 (Microphone), in non-differential input mode
    // (non-differential means: only a single input pin)
    // The reference for Vref- and Vref+ are VSS and VCC respectively
    ADC14_configureConversionMemory(ADC_MEM0,
                                    ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                    ADC_INPUT_A10,
                                    ADC_NONDIFFERENTIAL_INPUTS);

    // This selects the GPIO as analog input
    // A9 is multiplexed on GPIO port P4 pin PIN4
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                                               GPIO_PIN3,
                                               GPIO_TERTIARY_MODULE_FUNCTION);
}
unsigned getSampleSensor() {
    return ADC14_getResult(ADC_MEM0);
}
void InitHannWindow(){
    //Init Hann Window for fft
    int n;
    for(n = 0; n < muestrasfft; n++)
    {
        hann[n] = 0.5f - 0.5f * cosf((2 * PI * n) / (muestrasfft - 1));
    }

}
void InitButton1(){
    P1->SEL0 &= ~BIT1;    //
    P1->SEL1 &= ~BIT1;    //General I/O is enable for P1.1
    P1->OUT |= BIT1;      //enable the pull up register
    P1->DIR &= ~BIT1;     //p1.1 to be input
    P1->REN |= BIT1;      //Enable pull-up resistor for P1.1
    P1 ->IFG &= ~BIT1;
    P1->IE |= BIT1;       //Enable interrupt for P1.1
    P1->IES |= BIT1;      //Interrupt on high to low transition
    NVIC_EnableIRQ(PORT1_IRQn);
}

int main(void)
{    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    InitButton1();
    initADC();
    initSensor();
    UART_config();
    GPIO_setAsOutputPin(
            GPIO_PORT_P1,
            GPIO_PIN0);
    InitTimer();
    InitHannWindow();
    Timer1msStartOneShot();
    startADC();
    __enable_interrupts();
    //ADC_MEM0

    UART_putchar_n("Modo Temporal \r\n",17);
    while (1)
    {

    }
}

void PORT1_IRQHandler(){


    if (P1->IFG & BIT1  ){
        P1 -> IFG &=~ BIT1;
        estado= (estado+1)%3;
        if(estado==0){
            UART_putchar_n("Modo Temporal \r\n",17);
        }
        else if(estado==1){
            UART_putchar_n("Modo Frecuencial \r\n",20);
        }
        else{
            UART_putchar_n("Modo Live \r\n",13);
        }
    }

}


