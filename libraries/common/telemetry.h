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

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H*/