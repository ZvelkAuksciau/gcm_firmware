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

#include "usbcfg.h"
#include "mpu6050.h"
#include "attitude.h"
#include "pwmio.h"
#include "misc.h"
#include "telemetry.h"
#include "eeprom.h"
#include "node.hpp"

/* Telemetry operation time out in milliseconds. */
#define TELEMETRY_SLEEP_MS      20

#define MPU6050_LOW_DETECTED    0x00000001
#define EEPROM_24C02_DETECTED   0x00000004

#define WARM_UP_COUNTER_MAX     0x00000BB8

#define STREAM_BUFFER_SIZE      0x20

/**
 * Global variables
 */
/* Status of the board. */
uint32_t g_boardStatus = 0;
/* Main thread termination flag. */
bool g_runMain = TRUE;
/* I2C error info structure. */
I2CErrorStruct g_i2cErrorInfo = {0, 0};
/* Stream data id. */
uint8_t g_streamDataID = 0;
/* Data streaming index. */
uint8_t g_streamIdx = 0;

/**
 * Local variables
 */
/* I2C1 configuration for I2C driver 1 */
static const I2CConfig i2cfg_d1 = {
    STM32_TIMINGR_PRESC(0U) | STM32_TIMINGR_SCLL(9U) |
    STM32_TIMINGR_SCLH(2U) | STM32_TIMINGR_SDADEL(0U) |
    STM32_TIMINGR_SCLDEL(0U),
  0,
  0
};

/* Binary semaphore indicating that new data is ready to be processed. */
static binary_semaphore_t bsemIMU1DataReady;
static binary_semaphore_t bsemStreamReady;


/* Data streaming buffer. */
static float dataStream[STREAM_BUFFER_SIZE];
/* Pointer to low or high part of the data streaming buffer. */
float *pStream = NULL;

static void streamUpdateData(PIMUStruct pIMU) {
  switch (g_streamDataID) {
  case 1: /* Accel X; */
    dataStream[g_streamIdx++] = pIMU->accelData[0];
    break;
  case 2: /* Accel Y; */
    dataStream[g_streamIdx++] = pIMU->accelData[1];
    break;
  case 3: /* Accel Z; */
    dataStream[g_streamIdx++] = pIMU->accelData[2];
    break;
  case 4: /* Gyro X;  */
    dataStream[g_streamIdx++] = pIMU->gyroData[0];
    break;
  case 5: /* Gyro Y;  */
    dataStream[g_streamIdx++] = pIMU->gyroData[1];
    break;
  case 6: /* Gyro Z;  */
    dataStream[g_streamIdx++] = pIMU->gyroData[2];
    break;
  case 7: /* Atti X;  */
    dataStream[g_streamIdx++] = pIMU->rpy[0];
    break;
  case 8: /* Atti Y;  */
    dataStream[g_streamIdx++] = pIMU->rpy[1];
    break;
  case 9: /* Atti Z;  */
    dataStream[g_streamIdx++] = pIMU->rpy[2];
    break;
  default:
    g_streamIdx = 0;
    g_streamDataID = 0;
  }

  if (g_streamIdx == (STREAM_BUFFER_SIZE / 2)) {
    pStream = &dataStream[0];
    chBSemSignal(&bsemStreamReady);
  }

  if (g_streamIdx == STREAM_BUFFER_SIZE) {
    g_streamIdx = 0;
    pStream = &dataStream[STREAM_BUFFER_SIZE / 2];
    chBSemSignal(&bsemStreamReady);
  }
}

/**
 * LED blinker thread. Times are in milliseconds.
 */
static THD_WORKING_AREA(waBlinkerThread, 64);
static msg_t BlinkerThread(void *arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while (!chThdShouldTerminateX()) {
    systime_t time;
    if (g_boardStatus & IMU_CALIBRATION_MASK) {
      time = 50;
    } else {
      time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 500;
    }
    palTogglePad(GPIOB, GPIOB_LED_DATA);
    chThdSleepMilliseconds(time);
  }
  /* This point may be reached if shut down is requested. */
  return 0;
}

/**
 * MPU6050 data polling thread. Times are in milliseconds.
 * This thread requests a new data from MPU6050 every 1.5 ms (@666 Hz).
 */
static THD_WORKING_AREA(waPollMPU6050Thread, 128);
static msg_t PollMPU6050Thread(void *arg) {
  systime_t time;
  uint32_t warmUp = 0;
  (void)arg;
  chRegSetThreadName("sensor_poll");
  time = chVTGetSystemTime();
  do {
    if (!mpu6050GetNewData(&g_IMU1)) {
      /* Restart I2C2 bus in case of an error. */
      i2cStop(&I2CD1);
      i2cStart(&I2CD1, &i2cfg_d1);
    }
    /* Wait until the next 1.5 milliseconds passes. */
    chThdSleepUntil(time += US2ST(1500));
  } while (warmUp++ < WARM_UP_COUNTER_MAX);

  while (!chThdShouldTerminateX()) {
    if (mpu6050GetNewData(&g_IMU1)) {
      chBSemSignal(&bsemIMU1DataReady);
    } else {
      /* Restart I2C2 bus in case of an error. */
      i2cStop(&I2CD1);
      i2cStart(&I2CD1, &i2cfg_d1);
    }
    /* Wait until the next 1.5 milliseconds passes. */
    chThdSleepUntil(time += US2ST(1500));
  }
  /* This point may be reached if shut down is requested. */
  return 0;
}

/**
 * Attitude calculation thread.
 * - This thread works in conjunction with PollMPU6050Thread thread.
 * - This thread is synchronized by PollMPU6050Thread thread.
 * - This thread has the highest priority level.
 */
static THD_WORKING_AREA(waAttitudeThread, 2048);
static msg_t AttitudeThread(void *arg) {
  (void)arg;
  chRegSetThreadName("attitude");
  attitudeInit();
  while (!chThdShouldTerminateX()) {
    /* Process IMU1 new data ready event. */
    if (chBSemWait(&bsemIMU1DataReady) == MSG_OK) {
      if (g_boardStatus & IMU1_CALIBRATION_MASK) {
        if (imuCalibrate(&g_IMU1, g_boardStatus & IMU1_CALIBRATE_ACCEL)) {
          g_boardStatus &= ~IMU1_CALIBRATION_MASK;
        }
      } else {
        attitudeUpdate(&g_IMU1);
      }
    }

    if (g_streamDataID) {
      streamUpdateData(&g_IMU1);
    }

    if (g_boardStatus & IMU_CALIBRATION_MASK) {
      pwmOutputDisableAll();
    } else {
      cameraRotationUpdate();
      actuatorsUpdate();
    }
  }
  /* This point may be reached if shut down is requested. */
  return 0;
}

static const CRCConfig crc32_config = {
  .poly_size         = 32,
  .poly              = 0x04C11DB7,
  .initial_val       = 0xFFFFFFFF,
  .final_val         = 0xFFFFFFFF,
  .reflect_data      = 1,
  .reflect_remainder = 1
};

/**
 * @brief   Application entry point.
 * @details
 */

static Node::uavcanNodeThread node;
int main(void) {
     thread_t *tpBlinker  = NULL;
    thread_t *tpPoller   = NULL;
    thread_t *tpAttitude = NULL;

  /* System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Initializes a serial-over-USB CDC driver. */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /* Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbStop(serusbcfg.usbp);
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(500);
  usbConnectBus(serusbcfg.usbp);
  usbStart(serusbcfg.usbp, &usbcfg);

  /* Activates the serial driver 4 using the driver's default configuration. */
  sdStart(&SD3, NULL);

  /* Activates the I2C driver 2. */
  i2cStart(&I2CD1, &i2cfg_d1);

  crcStart(&CRCD1, &crc32_config);           /* Activate CRC driver              */
  crcReset(&CRCD1);

  /* Initialize IMU data structure. */
  imuStructureInit(&g_IMU1, FALSE); // IMU1 on low address.

  /* Loads settings from external EEPROM chip.
     WARNING! If MPU6050 sensor is not connected to the I2C bus, there
     aren't pull-up resistors on SDA and SCL lines, therefore it is
     impossible to communicate with EEPROM without the sensor connected. */
  if (eepromLoadSettings()) {
    g_boardStatus |= EEPROM_24C02_DETECTED;
  }

  /* Initializes the MPU6050 sensor1. */
  if (mpu6050Init(g_IMU1.addr)) {
    g_boardStatus |= MPU6050_LOW_DETECTED;
    g_boardStatus |= IMU1_CALIBRATE_GYRO;
  }

  if (g_boardStatus & MPU6050_LOW_DETECTED) {
    /* Creates a taken binary semaphore. */
      chBSemObjectInit(&bsemIMU1DataReady, TRUE);
    /* Creates a taken binary semaphore. */
      chBSemObjectInit(&bsemStreamReady, TRUE);

    /* Creates the MPU6050 polling thread and attitude calculation thread. */
    tpPoller = chThdCreateStatic(waPollMPU6050Thread, sizeof(waPollMPU6050Thread),
      NORMALPRIO + 1, (tfunc_t)PollMPU6050Thread, NULL);
    tpAttitude = chThdCreateStatic(waAttitudeThread, sizeof(waAttitudeThread),
      HIGHPRIO, (tfunc_t)AttitudeThread, NULL);

    /* Starts motor drivers. */
    pwmOutputStart();

    /* Starts ADC and ICU input drivers. */
    mixedInputStart();
  }

  /* Creates the blinker thread. */
  tpBlinker = chThdCreateStatic(waBlinkerThread, sizeof(waBlinkerThread),
    LOWPRIO, (tfunc_t)BlinkerThread, NULL);

  node.start(NORMALPRIO-10);

  /* Normal main() thread activity. */
  while (g_runMain) {
    if ((g_boardStatus & EEPROM_24C02_DETECTED) && eepromIsDataLeft()) {
      eepromContinueSaving();
    }
    g_chnp = serusbcfg.usbp->state == USB_ACTIVE ? (BaseChannel *)&SDU1 : (BaseChannel *)&SD3;
    telemetryReadSerialData();
    /* Process data stream if ready. */
    if ((g_chnp == (BaseChannel *)&SDU1) && /* USB only; */
        (chBSemWaitTimeout(&bsemStreamReady, TIME_IMMEDIATE) == MSG_OK)) {
      telemetryWriteStream(pStream, sizeof(float) * STREAM_BUFFER_SIZE / 2);
    }
    chThdSleepMilliseconds(TELEMETRY_SLEEP_MS);
  }

  /* Starting the shut-down sequence.*/
  if (tpAttitude != NULL) {
    chThdTerminate(tpAttitude); /* Requesting termination.                  */
    chThdWait(tpAttitude);      /* Waiting for the actual termination.      */
  }
  if (tpPoller != NULL) {
    chThdTerminate(tpPoller);   /* Requesting termination.                  */
    chThdWait(tpPoller);        /* Waiting for the actual termination.      */
  }
  if (tpBlinker != NULL) {
    chThdTerminate(tpBlinker);  /* Requesting termination.                  */
    chThdWait(tpBlinker);       /* Waiting for the actual termination.      */
  }

  mixedInputStop();             /* Stopping mixed input devices.            */
  pwmOutputStop();              /* Stopping pwm output devices.             */
  i2cStop(&I2CD1);              /* Stopping I2C2 device.                    */
  sdStop(&SD3);                 /* Stopping serial port 4.                  */
  usbStop(serusbcfg.usbp);      /* Stopping USB port.                       */
  usbDisconnectBus(serusbcfg.usbp);
  sduStop(&SDU1);               /* Stopping serial-over-USB CDC driver.     */

  chSysDisable();

  /* Reset of all peripherals. */
  rccResetAPB1(0xFFFFFFFF);
  rccResetAPB2(0xFFFFFFFF);

  NVIC_SystemReset();

  return 0;
}
