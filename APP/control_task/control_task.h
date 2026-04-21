/**
 * @file control_task.h
 * @author 大帅将军
 * @brief
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "topics.hpp"

#define MAX_VELOCITY 3.0f // 最大速度，单位m/s或rad/s，根据实际情况调整


void controlInit();

void controlTask(void *argument);