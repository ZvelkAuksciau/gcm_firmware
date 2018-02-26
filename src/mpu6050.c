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

/**
 * This is device realize "read through write" paradigm. This is not
 * standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include "ch.h"
#include "hal.h"

#include "misc.h"
#include "telemetry.h"
#include "mpu6050.h"

/* C libraries: */
#include <string.h>

//=====================Register definitions=====================================
#define MPU9250_READ(n) n | 0x80

#define MPU9250_PWR_MGMNT_1             0x6A
#define MPU9250_PWR_MGMNT_1_RESET       (1 << 7)
#define MPU9250_PWR_MGMNT_1_SLEEP       (1 << 6)
#define MPU9250_PWR_MGMNT_1_CYCLE       (1 << 5)
#define MPU9250_PWR_MGMNT_1_GYRO_SBY    (1 << 4)
#define MPU9250_PWR_MGMNT_1_PD_PAT      (1 << 3)
#define MPU9250_PWR_MGMNT_1_CLKSEL_2    (1 << 2)
#define MPU9250_PWR_MGMNT_1_CLKSEL_1    (1 << 1)
#define MPU9250_PWR_MGMNT_1_CLKSEL_0    (1)

#define MPU9250_SMPLRT_DIV              0x19

#define MPU9250_CONFIG                  0x1A
#define MPU9250_CONFIG_FIFO_MODE        (1 << 6)

#define MPU9250_GYRO_CONFIG             0x1B
#define MPU9250_GYRO_CONFIG_XGYRO_Cten          (1 << 7)
#define MPU9250_GYRO_CONFIG_YGYRO_Cten          (1 << 6)
#define MPU9250_GYRO_CONFIG_ZGYRO_Cten          (1 << 5)
#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_1       (1 << 4)
#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_0       (1 << 3)
#define MPU9250_GYRO_CONFIG_GYRO_FCHOISE_B_1    (1 << 1)
#define MPU9250_GYRO_CONFIG_GYRO_FCHOISE_B_0    (1)

#define MPU9250_ACCEL_CONFIG            0x1C
#define MPU9250_ACCEL_CONFIG_ax_st_en           (1 << 7)
#define MPU9250_ACCEL_CONFIG_ay_st_en           (1 << 6)
#define MPU9250_ACCEL_CONFIG_az_st_en           (1 << 5)
#define MPU9250_ACCEL_CONFIG_FS_SEL_1           (1 << 4)
#define MPU9250_ACCEL_CONFIG_FS_SEL_0           (1 << 3)

#define MPU9250_ACCEL_CONFIG2           0x1D

#define MPU9250_ACCEL_XOUT_H            0x3B
//=========================End register definitions=============================

/* Sensor scales */
//#define MPU6050_GYRO_SCALE        (1.0f / 131.0f) //  250 deg/s
//#define MPU6050_GYRO_SCALE        (1.0f /  65.5f) //  500 deg/s
#define MPU6050_GYRO_SCALE        (1.0f /  32.8f) // 1000 deg/s
//#define MPU6050_GYRO_SCALE        (1.0f /  16.4f) // 2000 deg/s

#define GRAV                      9.81
//#define MPU6050_ACCEL_SCALE       (GRAV / 16384.0f) //  2G
//#define MPU6050_ACCEL_SCALE       (GRAV /  8192.0f) //  4G
#define MPU6050_ACCEL_SCALE       (GRAV /  4096.0f) //  8G
//#define MPU6050_ACCEL_SCALE       (GRAV /  2048.0f) // 16G

#define MPU6050_RX_BUF_SIZE       0x0E
#define MPU6050_TX_BUF_SIZE       0x05

#define IMU_AXIS_DIR_POS          0x08
#define IMU_AXIS_ID_MASK          0x07

#define IMU1_AXIS_DIR_POS         0x08
#define IMU1_AXIS_ID_MASK         0x07
#define IMU1_CONF_MASK            0x0F

#define IMU2_AXIS_DIR_POS         0x80
#define IMU2_AXIS_ID_MASK         0x70
#define IMU2_CONF_MASK            0xF0

/* I2C read transaction time-out in milliseconds. */
#define MPU6050_READ_TIMEOUT_MS   0x01
/* I2C write transaction time-out in milliseconds. */
#define MPU6050_WRITE_TIMEOUT_MS  0x01

#define CALIBRATION_COUNTER_MAX   2000

/**
 * Global variables
 */
/* Default packed sensor settings.
 * Structure of the sensor settings is:
 * D2|2I2|1I2|0I2|D1|2I1|1I1|0I1
 * where Dx  - axis direction of the sensor x;
 *       nIx - n-th bit of axis ID of the sensor x.
 */
uint8_t g_sensorSettings[3] = {
  0x00,              /* Pitch (X) */
  0x11,              /* Roll  (Y) */
  0x22 |
  IMU1_AXIS_DIR_POS |
  IMU2_AXIS_DIR_POS  /* Yaw   (Z) */
};

static const SPIConfig hs_spicfg = {
  NULL,
  GPIOA,
  GPIOA_MPU_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_0, //fpclk/64 SPI_CLK 555kHz
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

/* IMU data structure. */
IMUStruct g_IMU1;

/* I2C error info structure. */
extern I2CErrorStruct g_i2cErrorInfo;
extern uint32_t g_boardStatus;

/**
 * Local variables
 */
/* Data buffers */
static uint8_t mpu6050RXData[MPU6050_RX_BUF_SIZE];
static uint8_t mpu6050TXData[MPU6050_TX_BUF_SIZE];

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fAddrLow - IMU address pin A0 is pulled low flag.
 */
void imuStructureInit(PIMUStruct pIMU, uint8_t fAddrHigh) {
  uint8_t i;
  /* Initialize to zero. */
  memset((void *)pIMU, 0, sizeof(IMUStruct));
  /* Initialize to unity quaternion. */
  pIMU->qIMU[0] = 1.0f;

  if (fAddrHigh) {
    pIMU->addr = 0x01;//MPU6050_I2C_ADDR_A0_HIGH;
    for (i = 0; i < 3; i++) {
      pIMU->axes_conf[i] = g_sensorSettings[i] >> 4;
    }
  } else {
    pIMU->addr = 0x00;//MPU6050_I2C_ADDR_A0_LOW;
    for (i = 0; i < 3; i++) {
      pIMU->axes_conf[i] = g_sensorSettings[i] & IMU1_CONF_MASK;
    }
  }
}

/**
 * @brief
 */
void imuCalibrationSet(uint8_t flags) {
  g_boardStatus |= flags & IMU_CALIBRATION_MASK;
}

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fCalibrateAcc - accelerometer calibration flag.
 * @return 0 - if calibration is not finished;
 *         1 - if calibration is finished.
 */
uint8_t imuCalibrate(PIMUStruct pIMU, uint8_t fCalibrateAcc) {
  if (fCalibrateAcc) {
    if (pIMU->clbrCounter == 0) {
      /* Reset accelerometer bias. */
      pIMU->accelBias[0] = pIMU->accelData[0];
      pIMU->accelBias[1] = pIMU->accelData[1];
      pIMU->accelBias[2] = pIMU->accelData[2] - GRAV;
      pIMU->clbrCounter++;
      return 0;
    } else if (pIMU->clbrCounter < CALIBRATION_COUNTER_MAX) {
      /* Accumulate accelerometer bias. */
      pIMU->accelBias[0] += pIMU->accelData[0];
      pIMU->accelBias[1] += pIMU->accelData[1];
      pIMU->accelBias[2] += pIMU->accelData[2] - GRAV;
      pIMU->clbrCounter++;
      return 0;
    } else {
      /* Update accelerometer bias. */
      pIMU->accelBias[0] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->accelBias[1] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->accelBias[2] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->clbrCounter = 0;
    }
  } else {
    if (pIMU->clbrCounter == 0) {
      /* Reset gyroscope bias. */
      pIMU->gyroBias[0] = pIMU->gyroData[0];
      pIMU->gyroBias[1] = pIMU->gyroData[1];
      pIMU->gyroBias[2] = pIMU->gyroData[2];
      pIMU->clbrCounter++;
      return 0;
    } else if (pIMU->clbrCounter < CALIBRATION_COUNTER_MAX) {
      /* Accumulate gyroscope bias. */
      pIMU->gyroBias[0] += pIMU->gyroData[0];
      pIMU->gyroBias[1] += pIMU->gyroData[1];
      pIMU->gyroBias[2] += pIMU->gyroData[2];
      pIMU->clbrCounter++;
      return 0;
    } else {
      /* Update gyroscope bias. */
      pIMU->gyroBias[0] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->gyroBias[1] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->gyroBias[2] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->clbrCounter = 0;
    }
  }
  return 1;
}

void writeRegister(uint8_t regAddr, uint8_t value) {
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &hs_spicfg);
    spiSelect(&SPID1);

    uint8_t tx_data[2];
    tx_data[0] = regAddr;
    tx_data[1] = value; //Reset IMU

    spiSend(&SPID1, 2, tx_data);

    spiUnselect(&SPID1);
    spiReleaseBus(&SPID1);
}

uint8_t readRegister(uint8_t regValue) {
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &hs_spicfg);
    spiSelect(&SPID1);

    uint8_t tx_data = MPU9250_READ(regValue);
    uint8_t rx_data = 0;
    spiSend(&SPID1, 1, &tx_data);
    spiReceive(&SPID1, 1, &rx_data);

    spiUnselect(&SPID1);
    spiReleaseBus(&SPID1);
    return rx_data;
}

/**
 * @brief  Initialization function for the MPU6050 sensor.
 * @param  addr - I2C address of MPU6050 chip.
 * @return 1 - if initialization was successful;
 *         0 - if initialization failed.
 */
uint8_t mpu6050Init(uint8_t addr) {
    writeRegister(MPU9250_PWR_MGMNT_1, MPU9250_PWR_MGMNT_1_RESET);

    chThdSleepMilliseconds(100); //Wait for IMU to reset

    writeRegister(MPU9250_PWR_MGMNT_1, MPU9250_PWR_MGMNT_1_CLKSEL_0); //Clock PLL if avialable
    writeRegister(MPU9250_SMPLRT_DIV, 7); //Sample rate 1kHz
    writeRegister(MPU9250_CONFIG, 0);
    writeRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_CONFIG_GYRO_FS_SEL_1); //Fchoise = 2b11 -> Gyro sample rate = 8kHz BW 3600Hz
    writeRegister(MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_CONFIG_FS_SEL_1); //Full scale +-8g
    return 1;
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t mpu6050GetNewData(PIMUStruct pIMU) {
  msg_t status = I2C_NO_ERROR;
  uint8_t id;
  int16_t mpu6050Data[6];

  spiAcquireBus(&SPID1);
   spiStart(&SPID1, &hs_spicfg);
   spiSelect(&SPID1);

   uint8_t tx_data = MPU9250_READ(MPU9250_ACCEL_XOUT_H);
   uint8_t rx_data[14];
   spiSend(&SPID1, 1, &tx_data);
   spiReceive(&SPID1, 14, mpu6050RXData);

   spiUnselect(&SPID1);
   spiReleaseBus(&SPID1);

  mpu6050Data[0] = (int16_t)((mpu6050RXData[ 0]<<8) | mpu6050RXData[ 1]); /* Accel X */
  mpu6050Data[1] = (int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]); /* Accel Y */
  mpu6050Data[2] = (int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]); /* Accel Z */
  mpu6050Data[3] = (int16_t)((mpu6050RXData[ 8]<<8) | mpu6050RXData[ 9]); /* Gyro X  */
  mpu6050Data[4] = (int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]); /* Gyro Y  */
  mpu6050Data[5] = (int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]); /* Gyro Z  */

  /* Pitch: */
  id = pIMU->axes_conf[0] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[0] & IMU_AXIS_DIR_POS) {
    pIMU->accelData[0] = mpu6050Data[id + 0]*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[0]  = mpu6050Data[id + 3]*MPU6050_GYRO_SCALE;
  } else {
    pIMU->accelData[0] = (-1 - mpu6050Data[id + 0])*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[0]  = (-1 - mpu6050Data[id + 3])*MPU6050_GYRO_SCALE;
  }

  /* Roll: */
  id = pIMU->axes_conf[1] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[1] & IMU_AXIS_DIR_POS) {
    pIMU->accelData[1] = mpu6050Data[id + 0]*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[1]  = mpu6050Data[id + 3]*MPU6050_GYRO_SCALE;
  } else {
    pIMU->accelData[1] = (-1 - mpu6050Data[id + 0])*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[1]  = (-1 - mpu6050Data[id + 3])*MPU6050_GYRO_SCALE;
  }

  /* Yaw: */
  id = pIMU->axes_conf[2] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[2] & IMU_AXIS_DIR_POS) {
    pIMU->accelData[2] = mpu6050Data[id + 0]*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[2]  = mpu6050Data[id + 3]*MPU6050_GYRO_SCALE;
  } else {
    pIMU->accelData[2] = (-1 - mpu6050Data[id + 0])*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[2]  = (-1 - mpu6050Data[id + 3])*MPU6050_GYRO_SCALE;
  }

  return 1;
}

/**
 * @brief
 */
void sensorSettingsUpdate(const uint8_t *pNewSettings) {
  uint8_t i;
  memcpy((void *)g_sensorSettings, (void *)pNewSettings, sizeof(g_sensorSettings));
  for (i = 0; i < 3; i++) {
    g_IMU1.axes_conf[i] = g_sensorSettings[i] & IMU1_CONF_MASK;
  }
}

/**
 * @brief
 */
void accelBiasUpdate(PIMUStruct pIMU, const float *pNewSettings) {
  memcpy((void *)pIMU->accelBias, (void *)pNewSettings, sizeof(pIMU->accelBias));
}

/**
 * @brief
 */
void gyroBiasUpdate(PIMUStruct pIMU, const float *pNewSettings) {
	memcpy((void *)pIMU->gyroBias, (void *)pNewSettings, sizeof(pIMU->gyroBias));
}
