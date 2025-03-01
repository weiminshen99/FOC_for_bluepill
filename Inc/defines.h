/*
*
* 2022-11-3: Wei-Min Shen: Simplification: Deleted all LEFT_*
*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H

#include "stm32f1xx_hal.h"
#include "config.h"

// Hall sensor inputs for Bluepill: PB5,6,7 (no PC10,11,12)
#define HALL_U_PORT GPIOB
#define HALL_U_PIN GPIO_PIN_6
#define HALL_V_PORT GPIOB
#define HALL_V_PIN GPIO_PIN_7
#define HALL_W_PORT GPIOB
#define HALL_W_PIN GPIO_PIN_8

// From SimpleFOC examples for Bluepill:
// BLDCDriver6PWM(IN1_H,  IN1_L, IN2_H,  IN2_L,  IN3_H, IN3_L, enable(optional))
// BLDCDriver6PWM(phA_h,  phA_l, phB_h,  phB_l,  phC_h, phC_l, enable)
// BLDCDriver6PWM(  PA8,  PB13,    PA9,   PB14,   PA10,  PB15, PB12);
// U = A; V = B, W = C
#define MOTOR_TIM TIM1
#define MOTOR_TIM_U CCR1
#define MOTOR_TIM_UH_PIN GPIO_PIN_8
#define MOTOR_TIM_UH_PORT GPIOA
#define MOTOR_TIM_UL_PIN GPIO_PIN_13
#define MOTOR_TIM_UL_PORT GPIOB
#define MOTOR_TIM_V CCR2
#define MOTOR_TIM_VH_PIN GPIO_PIN_9
#define MOTOR_TIM_VH_PORT GPIOA
#define MOTOR_TIM_VL_PIN GPIO_PIN_14
#define MOTOR_TIM_VL_PORT GPIOB
#define MOTOR_TIM_W CCR3
#define MOTOR_TIM_WH_PIN GPIO_PIN_10
#define MOTOR_TIM_WH_PORT GPIOA
#define MOTOR_TIM_WL_PIN GPIO_PIN_15
#define MOTOR_TIM_WL_PORT GPIOB

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

#define BUZZER_PIN GPIO_PIN_13
#define BUZZER_PORT GPIOC

#define OFF_PIN GPIO_PIN_15
#define OFF_PORT GPIOC

#define BUTTON_PIN GPIO_PIN_9
#define BUTTON_PORT GPIOB

#define CHARGER_PIN GPIO_PIN_11
#define CHARGER_PORT GPIOA


/*
// PINS for ADC1
// ==> On Bluepill, ADC ports are PA0-7, PB0-1, so we need remap these
// #define MOTOR_DC_CUR_ADC ADC1
#define MOTOR_DC_CUR_PORT GPIOC
#define MOTOR_DC_CUR_PIN GPIO_PIN_0
// #define MOTOR_U_CUR_ADC ADC1
#define MOTOR_U_CUR_PORT GPIOA
#define MOTOR_U_CUR_PIN GPIO_PIN_0
// #define MOTOR_V_CUR_ADC ADC1
#define MOTOR_V_CUR_PORT GPIOC
#define MOTOR_V_CUR_PIN GPIO_PIN_3

// #define DCLINK_ADC ADC3
// #define DCLINK_CHANNEL

#define DCLINK_PIN GPIO_PIN_1
#define DCLINK_PORT GPIOA

// #define DCLINK_PULLUP 30000
// #define DCLINK_PULLDOWN 1000

// UNUSED/REDUNDANT
//#define SWITCH_PIN GPIO_PIN_1
//#define SWITCH_PORT GPIOA

#if defined(CONTROL_PPM_RIGHT)
#define PPM_PIN             GPIO_PIN_11
#define PPM_PORT            GPIOB
#endif

#if defined(CONTROL_PWM_RIGHT)
#define PWM_PIN_CH1         GPIO_PIN_10
#define PWM_PORT_CH1        GPIOB
#define PWM_PIN_CH2         GPIO_PIN_11
#define PWM_PORT_CH2        GPIOB
#endif

#if defined(SUPPORT_BUTTONS_RIGHT)
#define BUTTON1_PIN         GPIO_PIN_10
#define BUTTON1_PORT        GPIOB
#define BUTTON2_PIN         GPIO_PIN_11
#define BUTTON2_PORT        GPIOB
#endif
*/

#define DELAY_TIM_FREQUENCY_US 1000000

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0f) : (((x) < (-lowhigh)) ? (-1.0f) : (0.0f)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0f) : (((x) < (low)) ? (-1.0f) : (0.0f)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0f)
#define RAD(a) ((a)*180.0f / M_PI)
#define SIGN(a) (((a) < 0) ? (-1) : (((a) > 0) ? (1) : (0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define IN_RANGE(x, low, high) (((x) >= (low)) && ((x) <= (high)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0f), 1.0f)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
#define ARRAY_LEN(x) (uint32_t)(sizeof(x) / sizeof(*(x)))
#define MAP(x, in_min, in_max, out_min, out_max) (((((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min))) + (out_min))

#if defined(PRINTF_FLOAT_SUPPORT) && (defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)) && defined(__GNUC__)
    asm(".global _printf_float");     // this is the magic trick for printf to support float. Warning: It will increase code considerably! Better to avoid!
#endif

typedef struct {
  uint16_t dcr;
  uint16_t dcl;
  uint16_t rlA;
  uint16_t rlB;
  uint16_t rrB;
  uint16_t rrC;
  uint16_t batt1;
  uint16_t l_tx2;
  uint16_t temp;
  uint16_t l_rx2;
} adc_buf_t;

typedef enum {
  NUNCHUK_CONNECTING,
  NUNCHUK_DISCONNECTED,
  NUNCHUK_RECONNECTING,
  NUNCHUK_CONNECTED
} nunchuk_state;

// Define I2C, Nunchuk, PPM, PWM functions
void I2C_Init(void);
void PPM_Init(void);
void PPM_ISR_Callback(void);
void PWM_Init(void);
void PWM_ISR_CH1_Callback(void);
void PWM_ISR_CH2_Callback(void);
nunchuk_state Nunchuk_Read(void);

// Sideboard definitions
#define LED1_SET            (0x01)
#define LED2_SET            (0x02)
#define LED3_SET            (0x04)
#define LED4_SET            (0x08)
#define LED5_SET            (0x10)
#define SENSOR1_SET         (0x01)
#define SENSOR2_SET         (0x02)
#define SENSOR_MPU          (0x04)

// RC iBUS switch definitions. Flysky FS-i6S has [SWA, SWB, SWC, SWD] = [2, 3, 3, 2] positions switch
#define SWA_SET             (0x0100)   //  0000 0001 0000 0000
#define SWB_SET             (0x0600)   //  0000 0110 0000 0000
#define SWC_SET             (0x1800)   //  0001 1000 0000 0000
#define SWD_SET             (0x2000)   //  0010 0000 0000 0000

#endif // DEFINES_H

