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

#ifndef _MISC_H_
#define _MISC_H_

#include <math.h>

#define M_PI        3.14159f
#define M_TWOPI     M_PI*2.0f //TODO: refactor to use M_2PI
#define M_2PI       M_TWOPI
#define RAD2DEG     ( 180.0f / M_PI )
#define DEG2RAD     ( M_PI / 180.0f )

#define constrain(val,min,max)  ((val)<(min)?(min):((val)>(max)?(max):(val)))
#define circadjust(val,lim)     ((val)<-(lim)?(val)+2*(lim):((val)>(lim)?(val)-2*(lim):(val)))

typedef struct tagI2CErrorStruct {
  i2cflags_t last_i2c_error;
  uint32_t i2c_error_counter;
} __attribute__((packed)) I2CErrorStruct, *PI2CErrorStruct;

#ifdef __cplusplus
extern "C" {
#endif

static float inline wrap_2PI(float radian)
{
    float res = fmodf(radian, M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

static float inline wrap_PI(float radian)
{
    float res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}


/**
 * Fast Inverse Square Root approximation from www.gamedev.net
 * with magic number proposed by Chris Lomont:
 * - Lomont, Chris (February 2003).
 */
static inline float QInvSqrtf(float x) {
  float xhalf = 0.5f * x;
  union {
    uint32_t i;
    float    f;
  } y;
  y.f = x;
  y.i = 0x5f375a86 - (y.i >> 1);      // gives initial guess y0
  y.f *= 1.5f - xhalf * y.f * y.f;    // First Newton step, repeating increases accuracy
  //y.f *= 1.5f - xhalf * y.f * y.f;    // Second Newton step
  // and so on...
  return y.f;
}

/**
 * res = v1 x v2;
 */
static inline void CrossProduct(const float v1[3], const float v2[3], float res[3]) {
  res[0] = v1[1]*v2[2] - v2[1]*v1[2];
  res[1] = v2[0]*v1[2] - v1[0]*v2[2];
  res[2] = v1[0]*v2[1] - v2[0]*v1[1];
}

/**
 * @brief Find quaternion from roll, pitch and yaw.
 * @note  The order of rotations is:
 *        1. pitch (X);
 *        2. roll (Y);
 *        3. yaw (Z).
 */
static inline void RPY2Quaternion (const float rpy[3], float q[4]) {
  float phi, theta, psi;
  float cphi, sphi, ctheta, stheta, cpsi, spsi;

  phi    = rpy[0]*0.5f;
  theta  = rpy[1]*0.5f;
  psi    = rpy[2]*0.5f;

  cphi   = cosf(phi);
  sphi   = sinf(phi);
  ctheta = cosf(theta);
  stheta = sinf(theta);
  cpsi   = cosf(psi);
  spsi   = sinf(psi);

  q[0] = cphi*ctheta*cpsi + sphi*stheta*spsi;
  q[1] = sphi*ctheta*cpsi - cphi*stheta*spsi;
  q[2] = cphi*stheta*cpsi + sphi*ctheta*spsi;
  q[3] = cphi*ctheta*spsi - sphi*stheta*cpsi;
}

/**
 * @brief Find roll, pitch and yaw from quaternion.
 * @note  The order of rotations is:
 *        1. pitch (X);
 *        2. roll (Y);
 *        3. yaw (Z).
 */
static inline void Quaternion2RPY(const float q[4], float rpy[3]) {
  float R13, R11, R12, R23, R33;
  float q2s = q[2]*q[2];

  R11 = 1.0f - 2.0f * (q2s + q[3]*q[3]);
  R12 = 2.0f * (q[0]*q[3] + q[1]*q[2]);
  R13 = 2.0f * (q[0]*q[2] - q[1]*q[3]);
  R23 = 2.0f * (q[0]*q[1] + q[2]*q[3]);
  R33 = 1.0f - 2.0f * (q2s + q[1]*q[1]);

  rpy[1] = asinf (R13);   // roll always between -pi/2 to pi/2
  rpy[2] = atan2f(R12, R11);
  rpy[0] = atan2f(R23, R33);

  //TODO: consider the cases where |R13| ~= 1, |roll| ~= pi/2
}

/**
 * @brief  Resets the CRC Data register (DR).
 * @param  None
 * @retval None
 */
static inline void crcResetDR(void) {
  /* Resets CRC generator. */
  CRC->CR = CRC_CR_RESET;
}

/**
 * @brief  Computes the 32-bit CRC of a given buffer of data word(32-bit).
 * @param  pBuf: pointer to the buffer containing the data to be computed
 * @param  length: length of the buffer to be computed
 * @retval 32-bit CRC
 */
//static inline uint32_t crcCRC32(const uint32_t pBuf[], uint32_t length) {
//  /* Resets CRC generator. */
//  //crcReset(&CRCD1);
//  /* Calculates CRC32 checksum. */
//  return crcCalc(&CRCD1, length, pBuf);
//}
/* Nibble lookup table for 0x04C11DB7 polynomial. */
static const uint32_t crc_tab[16] = {
    0x00000000,0x04C11DB7,0x09823B6E,0x0D4326D9,
    0x130476DC,0x17C56B6B,0x1A864DB2,0x1E475005,
    0x2608EDB8,0x22C9F00F,0x2F8AD6D6,0x2B4BCB61,
    0x350C9B64,0x31CD86D3,0x3C8EA00A,0x384FBDBD
};

/**
 * @brief crc32 - calculates CRC32 checksum of the data buffer.
 * @param pBuf - address of the data buffer.
 * @param length - length of the buffer.
 * @return CRC32 checksum.
 */
static inline uint32_t crcCRC32(const uint32_t pBuf[], uint32_t length)
{
    uint32_t i;
    /* Initial XOR value. */
    uint32_t crc = 0xFFFFFFFF;

    for (i = 0; i < length; i++) {
        /* Apply all 32-bits: */
        crc ^= pBuf[i];

        /* Process 32-bits, 4 at a time, or 8 rounds.
         * - Assumes 32-bit reg, masking index to 4-bits;
         * - 0x04C11DB7 Polynomial used in STM32.
         */
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
        crc = (crc << 4) ^ crc_tab[crc >> 28];
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif /* _MISC_H_ */
