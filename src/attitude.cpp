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

/*
  References:
  [1] Mahony, R.; Hamel, T.; Pflimlin, Jean-Michel, "Nonlinear Complementary
      Filters on the Special Orthogonal Group", Automatic Control,
      IEEE Transactions on, vol.53, no.5, pp.1203,1218, June 2008
  [2] Euston, M.; Coote, P.; Mahony, R.; Jonghyuk Kim; Hamel, T.,
      "A complementary filter for attitude estimation of a fixed-wing UAV",
      Intelligent Robots and Systems, 2008. IROS 2008. IEEE/RSJ International
      Conference on, vol., no., pp.340,345, 22-26 Sept. 2008
*/

/*
  Note:
  - the order of rotation of the standard 3D-gimbal system is:
    Pitch (X) then Roll (Y) and then Yaw (Z).
*/

#include "ch.h"
#include "hal.h"

#include "mpu6050.h"
#include "pwmio.h"
#include "misc.h"
#include "attitude.h"

#include "misc.hpp"

/* C libraries: */
#include <string.h>

#define FIXED_DT_STEP             0.0015f

#define MOTOR_STEP_LIMIT_MAX      M_PI / 45.0f
#define MOTOR_STEP_LIMIT_MIN      -MOTOR_STEP_LIMIT_MAX

#define ACCEL_TAU                 0.1f
#define INPUT_SIGNAL_ALPHA        100.0f
#define MODE_FOLLOW_DEAD_BAND     0.01f



/* PID controller structure. */
typedef struct tagPIDStruct {
  float P;
  float I;
  float D;
  float prevDist;
  float prevSpeed;
  float prevCmd;
  float errSum;
} __attribute__((packed)) PIDStruct, *PPIDStruct;

/**
 * Global variables.
 */
/* Mechanical offset of the motors. */
float g_motorOffset[3] = {0.0f, 0.0f, 0.0f};

CanInputStruct g_canInput[3] = { 0 };
CanInputStruct g_controlInput[3] = { 0 };

Quaterion g_autopilot_attitude;
Location g_location;
Location g_target_loc;

/**
 * Default PID settings.
 */
PIDSettings g_pidSettings[3] = {
/* P, I, D */
  {0, 0, 0}, /* Pitch PID */
  {0, 0, 0}, /* Roll  PID */
  {0, 0, 0}, /* Yaw   PLocationID */
};

/**
 * Default input mode settings.
 */
InputModeStruct g_modeSettings[3] = {
  {-60,               /* Min angle */
   60,                /* Max angle */
   0,                 /* Offset    */
   20,                /* Speed     */
   INPUT_MODE_ANGLE}, /* Mode ID   */
  {-60,               /* Min angle */
   60,                /* Max angle */
   0,                 /* Offset    */
   20,                /* Speed     */
   INPUT_MODE_ANGLE}, /* Mode ID   */
  {-90,               /* Min angle */
   90,                /* Max angle */
   0,                 /* Offset    */
   20,                /* Speed     */
   INPUT_MODE_SPEED}, /* Mode ID   */
};

/**
 * Default complementary filter settings.
 */
uint16_t g_cfSettings[2] = {
  300, /* 2Kp */
  100  /* 2Ki */
};

/**
 * Local variables
 */
static float camAtti[3] = {0.0f};
static float camRot[3] = {0.0f};
static float camRotSpeedPrev[3] = {0.0f};

static float accel2Kp = 30.0f;
static float accel2Ki = 0.001f;

/* Accelerometer filter variables. */
static uint8_t fAccelFilterEnabled = TRUE;
static float accel_alpha = 0.0f;

/* PID controller parameters. */
static PIDStruct PID[3] = {
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};
/**
 * @brief  Implements basic PID stabilization of the motor speed.
 * @param  cmd_id - command id to apply PID action to.
 * @param  err - process error value.
 * @param  rot - rotation command value.
 * @return weighted sum of P, I and D actions.
 */
static float pidControllerApply(uint8_t cmd_id, float err, float rot) {
  float poles2 = (float)(g_pwmOutput[cmd_id].num_poles >> 1);
  /* Distance for the motor to travel: */
  float dist = circadjust(err, M_PI);
  /* Convert mechanical distance to electrical distance: */
  //dist *= poles2;
  /* Convert mechanical rotation to electrical rotation: */
  //rot *= poles2;
  /* If there is a distance to travel then rotate the motor in small steps: */
  //float step = constrain(dist*PID[cmd_id].I, MOTOR_STEP_LIMIT_MIN, MOTOR_STEP_LIMIT_MAX);
  PID[cmd_id].errSum += PID[cmd_id].I * dist/10.0f;
  if(PID[cmd_id].errSum > 1.0f) {
    PID[cmd_id].errSum = 1.0f;
  } else if(PID[cmd_id].errSum < -1.0f) {
    PID[cmd_id].errSum = -1.0f;
  }
  float step = PID[cmd_id].errSum;
  /* Calculate proportional speed of the motor: */
  float speed = dist - PID[cmd_id].prevDist;
  step += dist*PID[cmd_id].P;
  /* Account for the acceleration of the motor: */
  step += speed*PID[cmd_id].D;
  /* Add rotation command. */
  //step += rot;
  /* Update offset of the motor: */
  //g_motorOffset[cmd_id] += step / poles2;
  /* Wind-up guard limits motor offset range to one mechanical rotation: */
  //g_motorOffset[cmd_id] = circadjust(g_motorOffset[cmd_id], M_PI);
  /* Update motor position: */
  float cmd = step;//PID[cmd_id].prevCmd + step;
  /* Normalize command to -1..1 range: */
  if (cmd < -1.0f) {
    cmd = -1.0f;
  } else if(cmd > 1.0f) {
    cmd = 1.0f;
  }
  /* Save values for the next iteration: */
  PID[cmd_id].prevDist = dist;
  PID[cmd_id].prevSpeed = speed;
  PID[cmd_id].prevCmd = cmd;
  return cmd;
}

/**
 * @brief
 */
static void pidUpdateStruct(void) {
  uint8_t i;
  for (i = 0; i < 3; i++) {
    PID[i].P = (float)g_pidSettings[i].P*0.1f;
    PID[i].I = (float)g_pidSettings[i].I*0.01f;
    PID[i].D = (float)g_pidSettings[i].D*1.0f;
    if (!g_pidSettings[i].I) {
      g_motorOffset[i] = 0.0f;
    }
  }
}

/**
 * @brief
 */
static void cfUpdateSettings(void) {
  accel2Kp = g_cfSettings[0] * 0.1f;
  accel2Ki = g_cfSettings[1] * 0.00001f;
}

/**
 * @brief  First order low-pass filter.
 * @param  raw - pointer to raw data array;
 * @param  filtered - pointer to filtered data array;
 */
static void accelFilterApply(const float raw[], float filtered[]) {
  if (fAccelFilterEnabled) {
    filtered[0] = (filtered[0] - raw[0])*accel_alpha + raw[0];
    filtered[1] = (filtered[1] - raw[1])*accel_alpha + raw[1];
    filtered[2] = (filtered[2] - raw[2])*accel_alpha + raw[2];
  } else {
    memcpy((void *)filtered, (void *)raw, sizeof(float)*3);
  }
}

/**
 * @brief
 */
void attitudeInit(void) {
  memset((void *)PID, 0, sizeof(PID));
  pidUpdateStruct();
  cfUpdateSettings();
  accel_alpha = expf(-FIXED_DT_STEP / ACCEL_TAU);
}

/**
 * @brief
 */
__attribute__((optimize("O3"))) void attitudeUpdate(PIMUStruct pIMU) {
  float accelErr[3] = {0.0f};
  float mag;
  float dq[4];

  pIMU->accelData[0] -= pIMU->accelBias[0];
  pIMU->accelData[1] -= pIMU->accelBias[1];
  pIMU->accelData[2] -= pIMU->accelBias[2];

  // Account for accel's magnitude.
  mag = QInvSqrtf(pIMU->accelData[0]*pIMU->accelData[0] + pIMU->accelData[1]*pIMU->accelData[1] + pIMU->accelData[2]*pIMU->accelData[2]);

  if ((mag > 0.0724f) && (mag < 0.1724f)) {
    float v2[3];

    /* Compute estimated direction of gravity halved.
     *
     * Rotated gravity vector v2 is calculated by multiplying gravity
     * vector v={0,0,1} by conjugate (q`) of attitude quaternion (q) (v2=q`vq),
     * because MPU6050 senses gravity in opposite direction.
     */
    v2[0] = (pIMU->qIMU[1]*pIMU->qIMU[3] - pIMU->qIMU[0]*pIMU->qIMU[2]);
    v2[1] = (pIMU->qIMU[2]*pIMU->qIMU[3] + pIMU->qIMU[0]*pIMU->qIMU[1]);
    v2[2] = (pIMU->qIMU[0]*pIMU->qIMU[0] + pIMU->qIMU[3]*pIMU->qIMU[3]) - 0.5f;

    // Apply smoothing to accel values, to reduce vibration noise before main calculations.
    accelFilterApply(pIMU->accelData, pIMU->accelFiltered);
    // Apply the same filtering to the estimated direction of gravity to match phase shift.
    accelFilterApply(v2, pIMU->v2Filtered);
    // Compute the error between the predicted direction of gravity and smoothed acceleration.
    CrossProduct(pIMU->accelFiltered, pIMU->v2Filtered, accelErr);

    // Normalize accel_error.
    accelErr[0] *= mag;
    accelErr[1] *= mag;
    accelErr[2] *= mag;
  }

  // Correct rates based on error.
  pIMU->gyroBias[0] -= accelErr[0]*accel2Ki;
  pIMU->gyroBias[1] -= accelErr[1]*accel2Ki;
  pIMU->gyroBias[2] -= accelErr[2]*accel2Ki;

  pIMU->gyroData[0] -= pIMU->gyroBias[0];
  pIMU->gyroData[1] -= pIMU->gyroBias[1];
  pIMU->gyroData[2] -= pIMU->gyroBias[2];

  // Correct rates based on error.
  pIMU->gyroData[0] += accelErr[0]*accel2Kp;
  pIMU->gyroData[1] += accelErr[1]*accel2Kp;
  pIMU->gyroData[2] += accelErr[2]*accel2Kp;

  dq[0] = (-pIMU->qIMU[1]*pIMU->gyroData[0] - pIMU->qIMU[2]*pIMU->gyroData[1] - pIMU->qIMU[3]*pIMU->gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);
  dq[1] = ( pIMU->qIMU[0]*pIMU->gyroData[0] - pIMU->qIMU[3]*pIMU->gyroData[1] + pIMU->qIMU[2]*pIMU->gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);
  dq[2] = ( pIMU->qIMU[3]*pIMU->gyroData[0] + pIMU->qIMU[0]*pIMU->gyroData[1] - pIMU->qIMU[1]*pIMU->gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);
  dq[3] = (-pIMU->qIMU[2]*pIMU->gyroData[0] + pIMU->qIMU[1]*pIMU->gyroData[1] + pIMU->qIMU[0]*pIMU->gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);

  pIMU->qIMU[0] += dq[0];
  pIMU->qIMU[1] += dq[1];
  pIMU->qIMU[2] += dq[2];
  pIMU->qIMU[3] += dq[3];

  // Normalize attitude quaternion.
  mag = QInvSqrtf(pIMU->qIMU[0]*pIMU->qIMU[0] + pIMU->qIMU[1]*pIMU->qIMU[1] + pIMU->qIMU[2]*pIMU->qIMU[2] + pIMU->qIMU[3]*pIMU->qIMU[3]);
  pIMU->qIMU[0] *= mag;
  pIMU->qIMU[1] *= mag;
  pIMU->qIMU[2] *= mag;
  pIMU->qIMU[3] *= mag;

  // Convert attitude into Euler angles;
  Quaternion2RPY(pIMU->qIMU, pIMU->rpy);
}

/**
 * @brief
 */
__attribute__((optimize("O0")))
void cameraRotationUpdate(void) {
    uint8_t i;
    float coef;
    float speedLimit;

    float target_angles[3];

    if(g_canInput[0].mode == INPUT_MODE_GPS_COORD) {
        if (!g_target_loc.is_zero()) {
            calculate_angle_to_location(g_location, g_target_loc, target_angles);
            g_controlInput[0].mode = INPUT_MODE_ANGLE;
            g_controlInput[2].mode = INPUT_MODE_BODYFRAME;

            g_controlInput[0].cmd = target_angles[0];
            float att[3];
            g_autopilot_attitude.GetEulerAngles(att[0], att[1], att[2]);
            g_controlInput[2].cmd = wrap_PI(target_angles[2] - att[2]);
        } else {
            for(uint8_t i = 0; i < 3; i++) {
                g_controlInput[i].mode = INPUT_MODE_SPEED;
                g_controlInput[i].cmd = 0.0f;
            }
        }
    } else { //if not GPS mode just copy
        for(uint8_t i = 0; i < 3; i++) {
            g_controlInput[i].mode = g_canInput[i].mode;
            g_controlInput[i].cmd = g_canInput[i].cmd;
        }
    }

    for (i = 0; i < 3; i++) {
        speedLimit = ((float) g_modeSettings[i].speed) * DEG2RAD;

        if (g_controlInput[i].mode & INPUT_MODE_BODYFRAME) {
            /* Calculate offset of the gimbal: */
            coef = g_motorOffset[i] - g_controlInput[i].cmd;

            coef = wrap_PI(coef);

            if(PID[i].prevDist > 0.1f || PID[i].prevDist < -0.1f) {
                coef = 0.0f;
            }
            if (coef > MODE_FOLLOW_DEAD_BAND) {
                coef -= MODE_FOLLOW_DEAD_BAND;
                /* Convert to speed: */
                coef /= INPUT_SIGNAL_ALPHA * FIXED_DT_STEP;
            } else if (coef < -MODE_FOLLOW_DEAD_BAND) {
                coef += MODE_FOLLOW_DEAD_BAND;
                /* Convert to speed: */
                coef /= INPUT_SIGNAL_ALPHA * FIXED_DT_STEP;
            } else {
                coef = 0.0f;
            }
        } else if (g_controlInput[i].mode == INPUT_MODE_NONE) { //Unknow mode
            camRot[i] = 0.0f;
            continue;
        } else if (g_controlInput[i].mode == INPUT_MODE_GPS_COORD) {
            coef = g_motorOffset[i];
            if(PID[i].prevDist > 0.1f || PID[i].prevDist < -0.1f) {
                coef = 0.0f;
            }
            if (coef > MODE_FOLLOW_DEAD_BAND) {
                coef -= MODE_FOLLOW_DEAD_BAND;
                /* Convert to speed: */
                coef /= INPUT_SIGNAL_ALPHA * FIXED_DT_STEP;
            } else if (coef < -MODE_FOLLOW_DEAD_BAND) {
                coef += MODE_FOLLOW_DEAD_BAND;
                /* Convert to speed: */
                coef /= INPUT_SIGNAL_ALPHA * FIXED_DT_STEP;
            } else {
                coef = 0.0f;
            }
        } else {
            coef = g_controlInput[i].cmd;

            if (g_controlInput[i].mode & INPUT_MODE_SPEED) {
                coef = constrain(coef, -speedLimit, speedLimit);
                camRotSpeedPrev[i] += (coef - camRotSpeedPrev[i])
                        / INPUT_SIGNAL_ALPHA;
                coef = camRotSpeedPrev[i];
            } else { /* INPUT_MODE_ANGLE */
                /* Calculate angle from input data: */
                coef = constrain(coef,
                        (float)g_modeSettings[i].min_angle*DEG2RAD,
                        (float)g_modeSettings[i].max_angle*DEG2RAD);
                /* Convert angle difference to speed: */
                coef = (coef - camAtti[i]) / INPUT_SIGNAL_ALPHA / FIXED_DT_STEP;
            }
        }
        coef = constrain(coef, -speedLimit, speedLimit);
        camRot[i] = coef * FIXED_DT_STEP;
    }
}

/**
 * @brief
 */
void actuatorsUpdate(void) {
  float err;

  //Quaterion g_imu(g_IMU1.qIMU[0], g_IMU1.qIMU[1], g_IMU1.qIMU[2], g_IMU1.qIMU[3]);
  Quaterion g_imu = Quaterion::FromEulerAngles(g_IMU1.rpy[0], 0.0f, 0.0f);
  Quaterion g_motors = Quaterion::FromEulerAngles(-g_motorOffset[0], 0.0f, 0.0f);//g_motorOffset[1], g_motorOffset[2]);
  Quaterion q = g_imu * g_motors.inverse();
  float cmd[3] = { 0.0f };
  /* Pitch: */
  uint8_t cmd_id = g_pwmOutput[PWM_OUT_PITCH].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
  if (cmd_id != PWM_OUT_CMD_DISABLED) {
    err = camAtti[cmd_id] - g_IMU1.rpy[cmd_id];
    camAtti[cmd_id] += camRot[cmd_id];
    camAtti[cmd_id] = circadjust(camAtti[cmd_id], M_PI);
    cmd[0] = pidControllerApply(cmd_id, err, camRot[cmd_id]);
  }
  /* Roll: */
  cmd_id = g_pwmOutput[PWM_OUT_ROLL].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
  if (cmd_id != PWM_OUT_CMD_DISABLED) {
    err = camAtti[cmd_id] - g_IMU1.rpy[cmd_id];
    camAtti[cmd_id] += camRot[cmd_id];
    camAtti[cmd_id] = circadjust(camAtti[cmd_id], M_PI);
    cmd[1] = pidControllerApply(cmd_id, err, camRot[cmd_id]);
  }
  /* Yaw: */
  cmd_id = g_pwmOutput[PWM_OUT_YAW].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
  if (cmd_id != PWM_OUT_CMD_DISABLED) {
    err = camAtti[cmd_id] - g_IMU1.rpy[cmd_id];
    camAtti[cmd_id] += camRot[cmd_id];
    camAtti[cmd_id] = circadjust(camAtti[cmd_id], M_PI);
    cmd[2] = pidControllerApply(cmd_id, err, camRot[cmd_id]);
  }
  Quaterion cmd_vector(0.0f, cmd[0], cmd[1], cmd[2]);
  cmd_vector = q.conjugate() * cmd_vector * q;
  pwmOutputUpdate(PWM_OUT_PITCH, cmd_vector.get_x());
  pwmOutputUpdate(PWM_OUT_ROLL, cmd_vector.get_y());
  pwmOutputUpdate(PWM_OUT_YAW, cmd_vector.get_z());
}

/**
 * @brief
 */
void pidSettingsUpdate(const PPIDSettings pNewSettings) {
  memcpy((void *)&g_pidSettings, (void *)pNewSettings, sizeof(g_pidSettings));
  pidUpdateStruct();
}

/**
 * @brief
 */
void inputModeSettingsUpdate(const PInputModeStruct pNewSettings) {
  memcpy((void *)&g_modeSettings, (void *)pNewSettings, sizeof(g_modeSettings));
}

/**
 * @brief
 */
void cfSettingsUpdate(const uint16_t *pNewSettings) {
  memcpy((void *)&g_cfSettings, (void *)pNewSettings, sizeof(g_cfSettings));
  cfUpdateSettings();
}


void calculate_angle_to_location(Location current_loc, Location target, float target_angles[3]) {
    int32_t target_lng = target.lng * 1e7;
    int32_t target_lat = target.lat * 1e7;
    int32_t current_lng = current_loc.lng * 1e7;
    int32_t current_lat = current_loc.lat * 1e7;
    int32_t target_alt = target.alt * 100;
    int32_t current_alt = current_loc.alt * 100;

    float GPS_vector_x = (target_lng - current_lng)
            * cosf(DEG2RAD*((current_lat + target_lat) * 0.00000005f))
            * 0.01113195f;
    float GPS_vector_y = (target_lat - current_lat) * 0.01113195f;
    float GPS_vector_z = (target_alt -current_alt);

    float target_distance = 100.0f*sqrt(GPS_vector_x*GPS_vector_x+ GPS_vector_y*GPS_vector_y);

    target_angles[0] = atan2f(GPS_vector_z, target_distance); //Pitch
    target_angles[1] = 0.0f; //Roll
    target_angles[2] = atan2f(GPS_vector_x, GPS_vector_y); //Absolute  yaw angle


}
