/**
  ******************************************************************************
  * @file           : ElMotorUnit.h
  * @brief          : 
  *                
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ELMOTORUNIT_H
#define __ELMOTORUNIT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

    typedef struct
    {
        int16_t Pitch;
        int16_t Roll;
        float VBAT;
    } ElMotorUnitParametersTypeDef;

#ifdef __cplusplus
}
#endif

#endif /* __ELMOTORUNIT_H */