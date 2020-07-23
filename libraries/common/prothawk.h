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
        VBATCommand = 6,
        TestMode = 7,
        CalibComplied = 8,
        PitchMinMax = 9,
        RollMinMax = 10,
        PitchForceCommand = 11
    } ElMotorCommandsTypeDef;

    typedef enum
    {
        HeaderPropultionCommand = 55

    } PropultionCommandsTypeDef;

    const uint8_t CommandSize[ENDCOMMAND] = {
        /*zero*/ 0,
        /*commands[PreFlightTestRequest] = */ 9,
        /*commands[PreFlightTestResponse] = */ 9,
        /*commands[WingCalibrationRequest] = */ 9,
        /*commands[WingCalibrationResponse] = */ 9,
        /*commands[PilotCommand] = */ 9,
        /*commands[PilotCommandResponse] = */ 23,
        /*commands[PING] = */ 9,
        /*commands[TelemetryRequest] = */ 1,
        /*commands[TelemetryResponse] = */ 21};

#ifdef __cplusplus
}
#endif

#endif /* __PROTHAWK_H */