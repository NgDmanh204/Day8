/*******************************************************************************
 *
 * Copyright (c) 2020
 * Lumi, JSC.
 * All Rights Reserved
 *
 * Description: Lab2.1
 *
 * Author: Developer embedded team
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 1.0 $
 * Last Changed:     $Date: 10/7/2020 $
 *
 ******************************************************************************/

#include <stdint.h>
#include "stm32f401re_gpio.h"
#include "stm32f401re_rcc.h"

#define GPIO_PIN_SET        1
#define GPIO_PIN_RESET      0
#define BTN_PRESS           0  // LOW

#define LED_ONBOARD_GPIO_PORT			GPIOA
#define LED_ONBOARD_GPIO_PIN			GPIO_Pin_5  // Onboard LED (assumed green on STM32F401RE)
#define LED_ONBOARD_RCC_CLK			RCC_AHB1Periph_GPIOA

// LED RGB - Green
#define LED1_GPIO_PORT      GPIOA
#define LED1_GPIO_PIN       GPIO_Pin_0
#define LED2_GPIO_PORT      GPIOA
#define LED2_GPIO_PIN       GPIO_Pin_11

// LED_3 - BLUE
#define LED3_GPIO_PORT      GPIOA
#define LED3_GPIO_PIN       GPIO_Pin_10  // Assigned for BLUE LED

// Button B2 (press once) - PA6
#define BUTTON_B2_PORT      GPIOB
#define BUTTON_B2_PIN       GPIO_Pin_3

// Button B5 (hold 500ms) - PB4
#define BUTTON_B5_PORT      GPIOB
#define BUTTON_B5_PIN       GPIO_Pin_4

// Buzzer - PC9
#define BUZZER_PORT         GPIOC
#define BUZZER_PIN          GPIO_Pin_9

// Basic delay (simple but with some error)
void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 5000; j++) {;}
    }
}

// Flash onboard LED (PA5)
void Flash_Onboard_LED(uint8_t times) {
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(LED_ONBOARD_RCC_CLK, ENABLE);

    gpio.GPIO_Pin = LED_ONBOARD_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(LED_ONBOARD_GPIO_PORT, &gpio);

    for (uint8_t i = 0; i < times; i++) {
        Set_Pin(LED_ONBOARD_GPIO_PORT, LED_ONBOARD_GPIO_PIN, GPIO_PIN_SET);
        delay_ms(300);
        Set_Pin(LED_ONBOARD_GPIO_PORT, LED_ONBOARD_GPIO_PIN, GPIO_PIN_RESET);
        delay_ms(300);
    }
}

// GPIO Initialization
void Led_Init(void) {
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = LED1_GPIO_PIN | LED2_GPIO_PIN | LED3_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &gpio);
}

void Button_Init(void) {
    GPIO_InitTypeDef gpio;

    // B2 - PA6
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = BUTTON_B2_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(BUTTON_B2_PORT, &gpio);

    // B5 - PB4
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = BUTTON_B5_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(BUTTON_B5_PORT, &gpio);
}

void Buzzer_Init(void) {
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin = BUZZER_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(BUZZER_PORT, &gpio);
}
uint8_t Read_Button(GPIO_TypeDef *port, uint16_t pin) {
    return (port->IDR & pin) ? 1 : 0;
}

void Set_Pin(GPIO_TypeDef *port, uint16_t pin, uint8_t state) {
    if (state == GPIO_PIN_SET) {
        port->BSRRL = pin;
    } else {
        port->BSRRH = pin;
    }
}

void Flash_RGB_and_Buzzer(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        Set_Pin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET);
        Set_Pin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_SET);
        Set_Pin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
        delay_ms(300);
        Set_Pin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET);
        Set_Pin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_RESET);
        Set_Pin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
        delay_ms(300);
    }
}

int main(void) {
    Led_Init();
    Button_Init();
    Buzzer_Init();

    // 1. Flash onboard LED to indicate power-on
    Flash_Onboard_LED(1);

    uint8_t B2_pressed_flag = 0;

    while (1) {
        // ----------- Handle button B2: press once to flash RGB GREEN LEDs and buzzer --------
        if (Read_Button(BUTTON_B2_PORT, BUTTON_B2_PIN) == BTN_PRESS) {
            if (B2_pressed_flag == 0) {
                delay_ms(50);  // Debounce
                Flash_RGB_and_Buzzer(1);  // Flash LEDs and buzzer once
                B2_pressed_flag = 1;
            }
        } else {
            B2_pressed_flag = 0;
        }

        // ----------- Handle button B5: hold > 500ms to turn on LED_3 (BLUE) ----------
        if (Read_Button(BUTTON_B5_PORT, BUTTON_B5_PIN) == BTN_PRESS) {
            delay_ms(500);  // Check if held for 500ms
            if (Read_Button(BUTTON_B5_PORT, BUTTON_B5_PIN) == BTN_PRESS) {
                Set_Pin(LED3_GPIO_PORT, LED3_GPIO_PIN, GPIO_PIN_SET);  // Turn on LED_3
            }else
            {
            	Set_Pin(LED3_GPIO_PORT, LED3_GPIO_PIN, GPIO_PIN_RESET);
            }
        }
    }
}
