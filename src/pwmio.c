/*
    EvvGC-PLUS - Copyright (C) 2013-2015 Sarunas Vaitekonis

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

#include "pwmio.h"
#include "misc.h"

/* C libraries: */
#include <math.h>
#include <string.h>

/**
 * DeadTime range (us) = (0..127) * 1 / 72;
 */
#define BDTR_DTG_MUL1   0x00
#define BDTR_DTG_MSK1   0x7F
/**
 * DeadTime range (us) = (64 + 0..63) * 2 / 72;
 */
#define BDTR_DTG_MUL2   0x80
#define BDTR_DTG_MSK2   0x3F
/**
 * DeadTime range (us) = (32 + 0..15) * 8 / 72;
 */
#define BDTR_DTG_MUL8   0xC0
#define BDTR_DTG_MSK8   0x1F
/**
 * DeadTime range (us) = (32 + 0..15) * 16 / 72;
 */
#define BDTR_DTG_MUL16  0xE0
#define BDTR_DTG_MSK16  0x1F

/* DT = 27 * 2 / 72 = 750ns */
#define PWM_OUT_TIM4_5_DT_US7   0x0000001B
/* DT = 36 * 2 / 72 = 1us */
#define PWM_OUT_TIM4_5_DT_1US   0x00000024
/* DT = 72 * 2 / 72 = 2us */
#define PWM_OUT_TIM4_5_DT_2US   0x00000048
/* DT = 108 * 2 / 72 = 3us */
#define PWM_OUT_TIM4_5_DT_3US   0x0000006C
/* DT = 144 * 2 / 72 = 4us */
#define PWM_OUT_TIM4_5_DT_4US   0x00000090
/* DT = 180 * 2 / 72 = 5us */
#define PWM_OUT_TIM4_5_DT_5US   0x000000B4
/* DT = 216 * 2 / 72 = 6us */
#define PWM_OUT_TIM4_5_DT_6US   0x000000D8
/* DT = 252 * 2 / 72 = 7us */
#define PWM_OUT_TIM4_5_DT_7US   0x000000FC

/**
 * PWM value for the 50 percent of the total power, given:
 * - PWM clock frequency = 72 MHz;
 * - PWM period = 1/18000 s;
 */
#define PWM_OUT_POWER_50PCT     0x000003E8
/**
 * PWM value for the half percent of the total power, given:
 * - PWM clock frequency = 72 MHz;
 * - PWM period = 1/18000 s;
 */
#define PWM_OUT_POWER_1PCT2     0x0000000A

/**
 * ADC related constants.
 */
#define ADC_GRP_NUM_CHANNELS    0x02
#define ADC_GRP_BUF_DEPTH       0x20

/**
 * Separation angle between phases.
 */
#ifndef M_2PI_3
#define M_2PI_3         (2.0f * M_PI / 3.0f)
#endif

/**
 * Amplitude scaling factor for third harmonic injection PWM.
 */
#define THI_PWM_K       (2.0f / sqrtf(3.0f))

/**
 * Local macros for dead time calculation.
 */
#define constrainLeft(val,left)     ((val)<(left)?(left):(val))
#define constrainRight(val,right)   ((val)>(right)?(right):(val))

/**
 * Default settings for PWM outputs.
 */
PWMOutputStruct g_pwmOutput[3] = {
  {0,                      /* Motor power;     */
   14,                     /* Number of poles; */
   0x00,                   /* Flags;           */
   PWM_OUT_CMD_DISABLED |
   PWM_OUT_DT5000NS},      /* DTime-Cmd ID;    */
  {0,                      /* Motor power;     */
   14,                     /* Number of poles; */
   0x00,                   /* Flags;           */
   PWM_OUT_CMD_DISABLED |
   PWM_OUT_DT5000NS},      /* DTime-Cmd ID;    */
  {0,                      /* Motor power;     */
   14,                     /* Number of poles; */
   0x00,                   /* Flags;           */
   PWM_OUT_CMD_DISABLED |
   PWM_OUT_DT5000NS}       /* DTime-Cmd ID;    */
};

/**
 * Default settings for generic inputs.
 */
MixedInputStruct g_mixedInput[3] = {
  {1000,                   /* Min value;      */
   1500,                   /* Mid value;      */
   2000,                   /* Max value;      */
   INPUT_CHANNEL_DISABLED},/* Input channel#; */
  {1000,                   /* Min value;      */
   1500,                   /* Mid value;      */
   2000,                   /* Max value;      */
   INPUT_CHANNEL_DISABLED},/* Input channel#; */
  {1000,                   /* Min value;      */
   1500,                   /* Mid value;      */
   2000,                   /* Max value;      */
   INPUT_CHANNEL_DISABLED} /* Input channel#; */
};

/**
 * Values of the input channels.
 */
int16_t g_inputValues[5] = {0};

/**
 * Local PWM output values for three phase BLDC motor driver.
 */
static uint32_t pwm3PhaseDrv[3];

/**
 * @brief  Disables PWM on roll driver.
 * @return none.
 */
static void pwmOutputDisableRoll(void) {
}

/**
 * @brief  Disables PWM on pitch driver.
 * @return none.
 */
static void pwmOutputDisablePitch(void) {
}

/**
 * @brief  Disables PWM on yaw driver.
 * @return none.
 */
static void pwmOutputDisableYaw(void) {
}

/**
 * @brief  Converts motor command to biased three phase motor driving PWM signal.
 * @param  cmd - new position of the motor.
 * @param  power - power of the motor in percent.
 * @param  thi - third harmonic injection enable flag.
 * @return none.
 */
static void pwmOutputCmdTo3PhasePWM(float cmd, uint8_t power, uint8_t thi) {
  float halfPower = power * PWM_OUT_POWER_1PCT2;
  if (thi) {
    halfPower *= THI_PWM_K;
    float thirdHarmonic = sinf(cmd * 3.0f) / 6.0f;
    pwm3PhaseDrv[0] = PWM_OUT_POWER_50PCT + (sinf(cmd) + thirdHarmonic)*halfPower;
    pwm3PhaseDrv[1] = PWM_OUT_POWER_50PCT + (sinf(cmd + M_2PI_3) + thirdHarmonic)*halfPower;
    pwm3PhaseDrv[2] = PWM_OUT_POWER_50PCT + (sinf(cmd - M_2PI_3) + thirdHarmonic)*halfPower;
  } else {
    pwm3PhaseDrv[0] = PWM_OUT_POWER_50PCT + sinf(cmd)*halfPower;
    pwm3PhaseDrv[1] = PWM_OUT_POWER_50PCT + sinf(cmd + M_2PI_3)*halfPower;
    pwm3PhaseDrv[2] = PWM_OUT_POWER_50PCT + sinf(cmd - M_2PI_3)*halfPower;
  }
}

/**
 *
 */
static void pwmOutputUpdateRoll(void) {
}

/**
 *
 */
static void pwmOutputUpdatePitch(void) {
}

/**
 *
 */
static void pwmOutputUpdateYaw(void) {
}

/**
 * @brief  Starts the PWM output.
 * @note   The pwmStart() function used in this code is not
 *         the original ChibiOS HAL function, but modified
 *         one with STM32_TIM_CR1_CEN flag removed.
 * @return none.
 */
void pwmOutputStart(void) {
}

/**
 * @brief  Stops the PWM output.
 * @return none.
 */
void pwmOutputStop(void) {
  pwmOutputDisableAll();
}

/**
 * @brief  Updates specified PWM output channel driver state
 *         according to the given command.
 * @param  channel_id - PWM output channel ID.
 * @param  cmd - new command to the motor driver.
 * @return none.
 */
void pwmOutputUpdate(const uint8_t channel_id, float cmd) {
  switch (channel_id) {
  case PWM_OUT_PITCH:
    if ((g_pwmOutput[PWM_OUT_PITCH].dt_cmd_id & PWM_OUT_CMD_ID_MASK) == PWM_OUT_CMD_DISABLED) {
      pwmOutputDisablePitch();
    } else {
      pwmOutputCmdTo3PhasePWM(cmd, g_pwmOutput[PWM_OUT_PITCH].power,
        g_pwmOutput[PWM_OUT_PITCH].flags & PWM_OUT_THI_FLAG);
      pwmOutputUpdatePitch();
    }
    break;
  case PWM_OUT_ROLL:
    if ((g_pwmOutput[PWM_OUT_ROLL].dt_cmd_id & PWM_OUT_CMD_ID_MASK) == PWM_OUT_CMD_DISABLED) {
      pwmOutputDisableRoll();
    } else {
      pwmOutputCmdTo3PhasePWM(cmd, g_pwmOutput[PWM_OUT_ROLL].power,
        g_pwmOutput[PWM_OUT_ROLL].flags & PWM_OUT_THI_FLAG);
      pwmOutputUpdateRoll();
    }
    break;
  case PWM_OUT_YAW:
    if ((g_pwmOutput[PWM_OUT_YAW].dt_cmd_id & PWM_OUT_CMD_ID_MASK) == PWM_OUT_CMD_DISABLED) {
      pwmOutputDisableYaw();
    } else {
      pwmOutputCmdTo3PhasePWM(cmd, g_pwmOutput[PWM_OUT_YAW].power,
        g_pwmOutput[PWM_OUT_YAW].flags & PWM_OUT_THI_FLAG);
      pwmOutputUpdateYaw();
    }
    break;
  default:;
  }
}

/**
 * @brief  Disables all PWM output channels.
 * @return none.
 */
void pwmOutputDisableAll(void) {
  pwmOutputUpdatePitch();
  pwmOutputUpdateRoll();
  pwmOutputUpdateYaw();
}

/**
 *
 */
void pwmOutputSettingsUpdate(const PPWMOutputStruct pNewSettings) {
  memcpy((void *)&g_pwmOutput, (void *)pNewSettings, sizeof(g_pwmOutput));
}

/**
 * @brief  Starts the ADC and ICU input drivers.
 * @note   ICU drivers used in the firmware are modified ChibiOS
 *         drivers for extended input capture functionality.
 * @return none.
 */
void mixedInputStart(void) {
}

/**
 * @brief  Stops the ADC and ICU input drivers.
 * @return none.
 */
void mixedInputStop(void) {
}

/**
 *
 */
void mixedInputSettingsUpdate(const PMixedInputStruct pNewSettings) {
  memcpy((void *)&g_mixedInput, (void *)pNewSettings, sizeof(g_mixedInput));
}
