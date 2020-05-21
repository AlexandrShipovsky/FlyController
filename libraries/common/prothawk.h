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

#define NUMCOMMANDS (uint16_t)512
    typedef enum
    {
        PreFlightTestRequest = 1,
        PreFlightTestResponse = 2,
        WingCalibrationRequest = 3,
        WingCalibrationResponse = 4,
        PilotCommand = 5,
        PilotCommandResponse = 6,
        PING = 7
    } HawkCommandsTypeDef;

    typedef struct
    {
        int16_t Pitch;
        int16_t Roll;
        int16_t Accel;
    }
    PitchRollAccelTypeDef;

    PitchRollAccelTypeDef f[5];

#ifdef __cplusplus
}
#endif

#endif /* __PROTHAWK_H */