/**
  ******************************************************************************
  * @file           : prothawk.h
  * @brief          : 
  *                
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PROTHAWK_H
#define __PROTHAWK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
    typedef enum
    {
        PreFlightTestRequest = 1,
        PreFlightTestResponse = 2,
        WingCalibrationRequest = 3,
        WingCalibrationResponse = 4,
        PilotCommand = 5,
        PilotCommandResponse = 6,
        PING = 7,
        TelemetryRequest = 8,
        TelemetryResponse = 9,
        ENDCOMMAND
    } HawkCommandsTypeDef;

    typedef enum
    {
        PitchRollCommand = 5,
        VBATCommand = 6
    } ElMotorCommandsTypeDef;

    typedef struct
    {
        int16_t Pitch;
        int16_t Roll;
        int16_t Accel;
    } PitchRollAccelTypeDef;

    const uint8_t CommandSize[ENDCOMMAND] = {
        /*zero*/0,
        /*commands[PreFlightTestRequest] = */ 0,
        /*commands[PreFlightTestResponse] = */ 0,
        /*commands[WingCalibrationRequest] = */ 9,
        /*commands[WingCalibrationResponse] = */ 2,
        /*commands[PilotCommand] = */ 9,
        /*commands[PilotCommandResponse] = */ 9,
        /*commands[PING] = */ 10,
        /*commands[TelemetryRequest] = */ 1,
        /*commands[TelemetryResponse] = */ 15
        };

#ifdef __cplusplus
}
#endif

#endif /* __PROTHAWK_H */