#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "bsp_usart.h"
#include "xbox.h"
/*-----------------------------------macro------------------------------------*/
#define MAX_ACCELERATION 0.5f
#define MAX_VELOCITY 2.7f
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void Control_Task(void *argument);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


#endif