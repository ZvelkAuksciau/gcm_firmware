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

#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "mpu6050.h"
#include "misc.hpp"

/* Input modes: */
#define INPUT_MODE_ANGLE          0x00 //Earth frame
#define INPUT_MODE_SPEED          0x01
#define INPUT_MODE_BODYFRAME      0x02 //keep constant with regards to body
#define INPUT_MODE_NONE           0x03
#define INPUT_MODE_GPS_COORD      0x04

typedef struct tagPIDSettings {
  uint8_t P;
  uint8_t I;
  uint8_t D;
} __attribute__((packed)) PIDSettings, *PPIDSettings;

typedef struct tagInputModeStruct {
  int16_t min_angle;
  int16_t max_angle;
  int16_t offset;
  uint8_t speed;
  uint8_t mode_id;
} __attribute__((packed)) InputModeStruct, *PInputModeStruct;

typedef struct tagCanInputStruct {
    float cmd; //RAD/S or RAD depending on mode
    uint8_t mode;
} CanInputStruct;

extern float g_motorOffset[3];
extern PIDSettings g_pidSettings[3];
extern InputModeStruct g_modeSettings[3];
extern uint16_t g_cfSettings[2];
extern CanInputStruct g_canInput[3];
extern Quaterion g_autopilot_attitude;
extern Location g_location;
extern Location g_target_loc;

#ifdef __cplusplus
extern "C" {
#endif
  void attitudeInit(void);
  void attitudeUpdate(PIMUStruct pIMU);
  void cameraRotationUpdate(void);
  void actuatorsUpdate(void);
  void pidSettingsUpdate(const PPIDSettings pNewSettings);
  void inputModeSettingsUpdate(const PInputModeStruct pNewSettings);
  void cfSettingsUpdate(const uint16_t *pNewSettings);
  void calculate_angle_to_location(Location current_loc, Location target, float target_angles[3]);
#ifdef __cplusplus
}
#endif

#endif /* _ATTITUDE_H_ */
