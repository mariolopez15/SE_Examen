/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"

#include "MKL46Z4.h"
#include "lcd.h"
#include <stdbool.h>
#include <string.h>


/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */


void irclk_ini()
{
    MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
    MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

void delay(void)
{
    volatile int i;

    for (i = 0; i < 500000; i++);
}

// RIGHT_SWITCH (SW1) = PTC3
void sw1_ini()
{
    SIM->COPC = 0; //desactiva el watchdog
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; //se activa el puerto C
    PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_IRQC(0xA);
    GPIOC->PDDR &= ~(1 << 3); //se  configura como pin de entrada
    NVIC_SetPriority(31, 0); //prioridad max
    NVIC_EnableIRQ(31); //habilitamos la interrupcion
}

// LEFT_SWITCH (SW2) = PTC12
void sw2_ini()
{
    SIM->COPC = 0; //desactiva el watchdog
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; //se activa el puerto C
    PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_IRQC(0xA);
    GPIOC->PDDR &= ~(1 << 12); //se configura como pin de entrada
    NVIC_SetPriority(31, 0); //prioridad max
    NVIC_EnableIRQ(31); //habilitamos la interrupcion
}


int sw1_check()
{
    return( !(GPIOC->PDIR & (1 << 3)) );
}

int sw2_check()
{
    return( !(GPIOC->PDIR & (1 << 12)) );
}

// RIGHT_SWITCH (SW1) = PTC3
// LEFT_SWITCH (SW2) = PTC12
void sws_ini()
{
    SIM->COPC = 0;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_IRQC(0xA) ;
    PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_IRQC(0xA) ;
    GPIOC->PDDR &= ~(1 << 3 | 1 << 12);

    NVIC_SetPriority(31, 0); //prioridad max
    NVIC_EnableIRQ(31); //habilitamos la interrupcion
}

// LED_GREEN = PTD5
void led_green_ini()
{
    SIM->COPC = 0;
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[5] = PORT_PCR_MUX(1); //se configura el pin como GPIO
    GPIOD->PDDR |= (1 << 5); //se configura como pin de salida
    GPIOD->PSOR = (1 << 5); //se pone a 1 el pin de salida
}

void led_green_toggle()
{
    GPIOD->PTOR = (1 << 5);
}

void led_green_clear()
{
    GPIOD->PSOR = (1 << 5);
}

void led_green_set()
{
    GPIOD->PCOR = (1 << 5);
}


// LED_RED = PTE29
void led_red_ini()
{
    SIM->COPC = 0;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[29] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= (1 << 29);
    GPIOE->PSOR = (1 << 29);
}

void led_red_toggle(void)
{
    GPIOE->PTOR = (1 << 29);
}
void led_red_clear(void)
{
    GPIOE->PSOR = (1 << 29);
}

void led_red_set(void)
{
    GPIOE->PCOR = (1 << 29);
}





// LED_RED = PTE29
// LED_GREEN = PTD5
void leds_ini()
{
    SIM->COPC = 0;
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
    PORTD->PCR[5] = PORT_PCR_MUX(1);
    PORTE->PCR[29] = PORT_PCR_MUX(1);
    GPIOD->PDDR |= (1 << 5);
    GPIOE->PDDR |= (1 << 29);
    // both LEDS off after init
    GPIOD->PSOR = (1 << 5);
    GPIOE->PSOR = (1 << 29);
}

void led1(){
    led_green_set();
    led_red_clear();
}

void led2(){
    led_red_set();
    led_green_clear();
}

void off(){
    led_red_clear();
    led_green_clear();
}


void PORTD_Int_Handler(void) {
    PORTC->PCR[3] |=PORT_PCR_ISF(1);
    PORTC->PCR[12] |=PORT_PCR_ISF(1);


    if(sw1_check()){
        led1();
    }else if(sw2_check()){
        led2();
    }



}


int main(void)
{
  //char ch;
  char palabra[15];
  //int pos=0; //posicion del char en la palabra

    irclk_ini();
    sw1_ini();
    sw2_ini();
    led_green_ini();
    led_red_ini();
    led_green_clear();
    led_red_clear();
    lcd_ini();

  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();





  PRINTF("\r\nReinicio!\r\n");
  PRINTF("Introduce los comandos\r\n");
  PUTCHAR('$');
  PUTCHAR(' ');
  while (1){

      /*
      ch = GETCHAR();
      palabra[pos]=ch;
      PUTCHAR(ch);
      if(ch == (char)'\r'){
          //SCANF("%s",palabra);
          //PRINTF("\r\n");
          PRINTF("\r\n%s\r\n", palabra);
          pos=0;


      }else{
          pos++;
      }
       */

      SCANF("%s",palabra);
      PRINTF("\r\n%s\r\n", palabra);

      if(!strcmp(palabra, "led1")){
          led1();
      }else if(!strcmp(palabra, "led2")){
          led2();
      }else if(!strcmp(palabra, "off")){
          off();
      }else if (atoi(palabra)!=0){
          lcd_display_dec(atoi(palabra));
      }


  }

}
