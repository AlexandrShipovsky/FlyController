/**
  ******************************************************************************
  * @file           : telemetry.h
  * @brief          : 
  *                
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

  typedef struct
  {
    int16_t Pitch;
    int16_t Roll;
    int16_t MinPitch;
    int16_t MaxPitch;
    int16_t MinRoll;
    int16_t MaxRoll;
    float VBAT;
    float PitchForce;
  } ElMotorUnitParametersTypeDef;

  typedef struct
  {
    uint8_t RPM; // Real value/1000
    uint8_t FuelCapacity; // 0 - 100%
    uint8_t ThrottlePosition; // 0 - 100%
  } PropultionParametersTypeDef;

  typedef struct 
  {
    float altitude;
    float yaw;
    float pitch;
    float roll;
  } IMUTelemetryTypeDef;

  typedef struct 
  {
    int16_t PitchJoystick;
    int16_t RollJoystick;
    uint8_t MoveType;
    uint8_t GasPedalJoystick;
  } JoystickTypeDef;
  

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H*/