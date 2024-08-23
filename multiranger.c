/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2019 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * log.h: Dynamic log system
 *
 * This file has been modified for the dataset collection framework of
 * PULP-Dronet v3 (https://github.com/pulp-platform/pulp-dronet) by:
 *        Lorenzo Lamberti      <lorenzo.lamberti@unibo.it>
 *        Daniel Rieben		      <riebend@student.ethz.ch>
 */

/* multiranger.c: Multiranger deck driver */
#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "MR"

#include "system.h"
#include "debug.h"
#include "log.h"
#include "pca95x4.h"
#include "vl53l1x.h"
#include "range.h"
// TEST
#include "usec_time.h"
#include "queue.h"
#include "static_mem.h"

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

#ifdef DEBUG_MULTIRANGER
#define MR_DEBUG(fmt, ...)  DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
#define MR_DEBUG(...)
#endif

static bool isInit = false;
static bool isTested = false;
static bool isPassed = false;

#define MR_PIN_UP     PCA95X4_P0
#define MR_PIN_FRONT  PCA95X4_P4
#define MR_PIN_BACK   PCA95X4_P1
#define MR_PIN_LEFT   PCA95X4_P6
#define MR_PIN_RIGHT  PCA95X4_P2

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devFront;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devBack;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devUp;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devLeft;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devRight;

struct{
    uint8_t front;
    uint8_t back;
    uint8_t right;
    uint8_t left;
    uint8_t up;
}range_states;

#define PRESET_MODE VL53L1_PRESETMODE_LITE_RANGING
#define RANGE_MODE VL53L1_DISTANCEMODE_LONG
#define TIMING_BUDGET_US 50000

static uint16_t mrGetMeasurementAndRestart(VL53L1_Dev_t *dev, uint8_t *range_status)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;

    status = VL53L1_WaitMeasurementDataReady(dev);
    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    status = VL53L1_ClearInterruptAndStartMeasurement(dev);
    status = status;

    *range_status = rangingData.RangeStatus;
    return (uint16_t) rangingData.RangeMilliMeter;
}

static void mrTask(void *param)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;

    systemWaitStart();

    // Change the distance mode and set the maximum allowed time for the sensor measurements
    status = VL53L1_SetPresetMode(&devFront, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devFront, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devFront, TIMING_BUDGET_US);

    status = VL53L1_SetPresetMode(&devBack, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devBack, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devBack, TIMING_BUDGET_US);

    status = VL53L1_SetPresetMode(&devUp, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devUp, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devUp, TIMING_BUDGET_US);

    status = VL53L1_SetPresetMode(&devLeft, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devLeft, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devLeft, TIMING_BUDGET_US);

    status = VL53L1_SetPresetMode(&devRight, PRESET_MODE);
    status = VL53L1_SetDistanceMode(&devRight, RANGE_MODE);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&devRight, TIMING_BUDGET_US);


    // Restart all sensors
    status = VL53L1_StartMeasurement(&devFront);
    status = VL53L1_StartMeasurement(&devBack);
    status = VL53L1_StartMeasurement(&devUp);
    status = VL53L1_StartMeasurement(&devLeft);
    status = VL53L1_StartMeasurement(&devRight);

    // Ignore the first interrupt and do not get data, clear interrupt
    status = VL53L1_WaitMeasurementDataReady(&devFront);
    status = VL53L1_ClearInterruptAndStartMeasurement(&devFront);

    status = VL53L1_WaitMeasurementDataReady(&devBack);
    status = VL53L1_ClearInterruptAndStartMeasurement(&devBack);

    status = VL53L1_WaitMeasurementDataReady(&devUp);
    status = VL53L1_ClearInterruptAndStartMeasurement(&devUp);

    status = VL53L1_WaitMeasurementDataReady(&devLeft);
    status = VL53L1_ClearInterruptAndStartMeasurement(&devLeft);

    status = VL53L1_WaitMeasurementDataReady(&devRight);
    status = VL53L1_ClearInterruptAndStartMeasurement(&devRight);
    status = status;

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, M2T(TIMING_BUDGET_US / 1000.0));
        rangeSet(rangeFront, mrGetMeasurementAndRestart(&devFront, &range_states.front)/1000.0f);
        rangeSet(rangeBack, mrGetMeasurementAndRestart(&devBack, &range_states.back)/1000.0f);
        rangeSet(rangeUp, mrGetMeasurementAndRestart(&devUp, &range_states.up)/1000.0f);
        rangeSet(rangeLeft, mrGetMeasurementAndRestart(&devLeft, &range_states.left)/1000.0f);
        rangeSet(rangeRight, mrGetMeasurementAndRestart(&devRight, &range_states.right)/1000.0f);
    }
}

static void mrInit()
{
    if (isInit)
    {
        return;
    }

    pca95x4Init();

    pca95x4ConfigOutput(~(MR_PIN_UP |
                          MR_PIN_RIGHT |
                          MR_PIN_LEFT |
                          MR_PIN_FRONT |
                          MR_PIN_BACK));

    pca95x4ClearOutput(MR_PIN_UP |
                       MR_PIN_RIGHT |
                       MR_PIN_LEFT |
                       MR_PIN_FRONT |
                       MR_PIN_BACK);

    isInit = true;

    xTaskCreate(mrTask, MULTIRANGER_TASK_NAME, MULTIRANGER_TASK_STACKSIZE, NULL,
        MULTIRANGER_TASK_PRI, NULL);
}

static bool mrTest()
{
    if (isTested)
    {
        return isPassed;
    }

    isPassed = isInit;

    pca95x4SetOutput(MR_PIN_FRONT);
    if (vl53l1xInit(&devFront, I2C1_DEV))
    {
        DEBUG_PRINT("Init front sensor [OK]\n");
        VL53L1_PresetModes presetMode;
        VL53L1_GetPresetMode(&devFront, &presetMode);
        switch(presetMode){
            case VL53L1_PRESETMODE_AUTONOMOUS:
                DEBUG_PRINT("Preset mode: Autonomous\n");
                break;
            case VL53L1_PRESETMODE_LITE_RANGING:
                DEBUG_PRINT("Preset mode: Lite ranging\n");
                break;
            case VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS:
                DEBUG_PRINT("Preset mode: Low power autonomous\n");
                break;
        }
        VL53L1_DistanceModes distanceMode;
        VL53L1_GetDistanceMode(&devFront, &distanceMode);
        switch(presetMode){
            case VL53L1_DISTANCEMODE_SHORT:
                DEBUG_PRINT("Distance mode: Short\n");
                break;
            case VL53L1_DISTANCEMODE_MEDIUM:
                DEBUG_PRINT("Distance mode: Medium\n");
                break;
            case VL53L1_DISTANCEMODE_LONG:
                DEBUG_PRINT("Distance mode: Long\n");
                break;
        }
        uint32_t measurementTimingBudgetMicroSeconds;
        VL53L1_GetMeasurementTimingBudgetMicroSeconds(&devFront, &measurementTimingBudgetMicroSeconds);
        DEBUG_PRINT("Measurement timing budget: %u us\n", measurementTimingBudgetMicroSeconds);
        uint32_t interMeasurementPeriodMilliSeconds;
        VL53L1_GetInterMeasurementPeriodMilliSeconds(&devFront, &interMeasurementPeriodMilliSeconds);
        DEBUG_PRINT("Inter measurement period: %u ms\n", interMeasurementPeriodMilliSeconds);
        FixPoint1616_t limitCheckValue;
        VL53L1_GetLimitCheckValue(&devFront, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, &limitCheckValue);
        DEBUG_PRINT("Sigma final range: %f\n", limitCheckValue / ((float) (1<<16)));
        VL53L1_GetLimitCheckValue(&devFront, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, &limitCheckValue);
        DEBUG_PRINT("Signal rate final range: %f\n", limitCheckValue/ ((float) (1<<16)));

    }
    else
    {
        DEBUG_PRINT("Init front sensor [FAIL]\n");
        isPassed = false;
    }

    pca95x4SetOutput(MR_PIN_BACK);
    if (vl53l1xInit(&devBack, I2C1_DEV))
    {
        DEBUG_PRINT("Init back sensor [OK]\n");
    }
    else
    {
        DEBUG_PRINT("Init back sensor [FAIL]\n");
        isPassed = false;
    }

    pca95x4SetOutput(MR_PIN_UP);
    if (vl53l1xInit(&devUp, I2C1_DEV))
    {
        DEBUG_PRINT("Init up sensor [OK]\n");
    }
    else
    {
        DEBUG_PRINT("Init up sensor [FAIL]\n");
        isPassed = false;
    }

    pca95x4SetOutput(MR_PIN_LEFT);
    if (vl53l1xInit(&devLeft, I2C1_DEV))
    {
        DEBUG_PRINT("Init left sensor [OK]\n");
    }
    else
    {
        DEBUG_PRINT("Init left sensor [FAIL]\n");
        isPassed = false;
    }

    pca95x4SetOutput(MR_PIN_RIGHT);
    if (vl53l1xInit(&devRight, I2C1_DEV))
    {
        DEBUG_PRINT("Init right sensor [OK]\n");
    }
    else
    {
        DEBUG_PRINT("Init right sensor [FAIL]\n");
        isPassed = false;
    }

    VL53L1_WaitDeviceBooted(&devFront);
    VL53L1_WaitDeviceBooted(&devBack);
    VL53L1_WaitDeviceBooted(&devUp);
    VL53L1_WaitDeviceBooted(&devLeft);
    VL53L1_WaitDeviceBooted(&devRight);

    isTested = true;

    return isPassed;
}

static const DeckDriver multiranger_deck = {
    .vid = 0xBC,
    .pid = 0x0C,
    .name = "bcMultiranger",

    .usedGpio = 0, // FIXME: set the used pins

    .init = mrInit,
    .test = mrTest,
};

DECK_DRIVER(multiranger_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(mRange)
LOG_ADD(LOG_UINT8, rangeStatusFront, &range_states.front)
LOG_ADD(LOG_UINT8, rangeStatusBack, &range_states.back)
LOG_ADD(LOG_UINT8, rangeStatusUp, &range_states.up)
LOG_ADD(LOG_UINT8, rangeStatusLeft, &range_states.left)
LOG_ADD(LOG_UINT8, rangeStatusRight, &range_states.right)
LOG_GROUP_STOP(mRange)
