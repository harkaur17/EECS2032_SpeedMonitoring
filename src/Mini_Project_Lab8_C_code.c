/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MKL43Z4_Project_2032_Exercise2.c
 * @brief   Application entry point.
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"

// global variables
volatile uint32_t g_hundredsCounter = 0;
volatile uint8_t  g_timerRunning    = 0;

volatile uint32_t g_speedLimit_kmh  = 0;
volatile uint32_t g_vehicleSpeed_kmh = 0;
volatile uint32_t g_vehicleId = 0;

#define POT_ADC_CHANNEL          (8U)
#define ADC_MAX_COUNTS           (4095.0f)
#define SPEED_LIMIT_MIN_KMH      (30.0f)
#define SPEED_LIMIT_MAX_KMH      (120.0f)

#define SENSOR_DISTANCE_M        (8.0f)
#define HUNDRED_MS_INTERVAL_SEC  (0.1f)

int main(void) {

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    __disable_irq();

    NVIC_DisableIRQ(PORTA_IRQn);
    NVIC_DisableIRQ(PORTC_PORTD_IRQn);

    SIM->SCGC5 |= (1<<9)|(1<<10)|(1<<12)|(1<<13);
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    PORTD->PCR[3] = 0x0103;
    PORTD->PCR[3] &= ~0xF0000;
    PORTD->PCR[3] |=  0xA0000;

    PORTA->PCR[4] = 0x0103;
    PORTA->PCR[4] &= ~0xF0000;
    PORTA->PCR[4] |=  0xA0000;

    PORTD->PCR[5] = 0x0100;
    PORTE->PCR[31] = 0x0100;

    PORTB->PCR[0] = 0x0000;

    PTD->PDDR &= ~(1<<3);
    PTA->PDDR &= ~(1<<4);

    PTD->PDDR |= (1<<5);
    PTE->PDDR |= (1u<<31);

    PTD->PDOR |= (1<<5);
    PTE->PDOR |= (1u<<31);

    SysTick->LOAD = 0x493E00 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;

    ADC0->CFG1 = ADC_CFG1_MODE(1)|ADC_CFG1_ADIV(0)|ADC_CFG1_ADICLK(0);
    ADC0->CFG2 = 0x00;
    ADC0->SC2  = ADC_SC2_REFSEL(1);
    ADC0->SC3  = 0x00;

    PORTD->ISFR = 0xFFFFFFFF;
    PORTA->ISFR = 0xFFFFFFFF;

    NVIC_SetPriority(PORTA_IRQn, 192);
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);

    NVIC_SetPriority(PORTC_PORTD_IRQn, 192);
    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);

    __enable_irq();

    while (1) {

        ADC0->SC1[0] = ADC_SC1_ADCH(POT_ADC_CHANNEL);
        while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {}

        uint16_t adcValue = ADC0->R[0] & 0x0FFF;

        float limitF = SPEED_LIMIT_MIN_KMH +
                       ((float)adcValue / ADC_MAX_COUNTS) *
                       (SPEED_LIMIT_MAX_KMH - SPEED_LIMIT_MIN_KMH);

        g_speedLimit_kmh = (uint32_t)(limitF + 0.5f);

        PRINTF("ADC: %u SpeedLimit: %u km/h\r\n",
               (unsigned int)adcValue,
               (unsigned int)g_speedLimit_kmh);


        for (volatile uint32_t d = 0; d < 12000000; d++) { __NOP(); }
    }
}

// start sensor
void PORTC_PORTD_DriverIRQHandler(void)
{
    if (PORTD->ISFR & (1 << 3))
    {
        PORTD->ISFR |= (1 << 3);

        PTD->PTOR |= (1 << 5);

        g_hundredsCounter = 0;
        g_timerRunning = 1;

        SysTick->VAL = 0;
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk |
                         SysTick_CTRL_ENABLE_Msk;

        PTE->PSOR |= (1u<<31);
    }
}

// stop sensor
void PORTA_DriverIRQHandler(void)
{
    if (PORTA->ISFR & (1 << 4))
    {
        PORTA->ISFR |= (1 << 4);

        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        g_timerRunning = 0;

        uint32_t counts = g_hundredsCounter;
        g_hundredsCounter = 0;

        if (counts > 0)
        {
            float t = counts * HUNDRED_MS_INTERVAL_SEC;
            float speedMS = SENSOR_DISTANCE_M / t;
            float speedKMH = speedMS * 3.6f;
            g_vehicleSpeed_kmh = (uint32_t)(speedKMH + 0.5f);
        }
        else {
            g_vehicleSpeed_kmh = 0;
        }

        g_vehicleId++;

        // fine calculations
        int32_t dv = (int32_t)g_vehicleSpeed_kmh - (int32_t)g_speedLimit_kmh;
        uint32_t fine = 0;

        if (dv < 10)
            fine = 0;
        else if (dv < 20)
            fine = 50;
        else if (dv < 30)
            fine = 100;
        else
            fine = 100 + 10 * (dv - 30);

       // led colour based on the fine
        if (fine > 0)
            PTE->PCOR |= (1u<<31);    // red ON
        else
            PTE->PSOR |= (1u<<31);    // red OFF

       // updated output
        PRINTF("v%u speed:%u limit:%u fine:%u\r\n",
               (unsigned int)g_vehicleId,
               (unsigned int)g_vehicleSpeed_kmh,
               (unsigned int)g_speedLimit_kmh,
               (unsigned int)fine);

        PTD->PDOR |= (1<<5);
    }
}

// SysTick timer
void SysTick_Handler(void)
{
    if (g_timerRunning)
    {
        PTD->PTOR |= (1<<5);
        g_hundredsCounter++;
    }
}
