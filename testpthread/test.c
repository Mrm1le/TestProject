/*
 * Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto. Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/**
 * @file DemoAppCom.c
 * @brief <b> Demo application to demonstrate API usage of FSI-CCPLEX generic
 * communicaton on ccplex</b>
 *
 * This file will demonstrate FSI-CCPLEX COM library APIs which are restricted
 * to be used by DriveOS user
 */

/* ==================[Includes]============================================= */

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <semaphore.h>
/* =================[Macros]================================================= */
/**
 *Configured number of FsiCom channels for DemoApp
 */
#define DA_FSI_COM_CHANNEL 1U
/**
 *Number of frames transmitted in Periodic Tx
 */
#define DA_TX_FRAME_COUNT 10U
/**
 * Periodicity in ms at which Frames are transmitted in Periodic Tx
 */
#define DA_PERIODICITY_MS 999U


#ifdef SAMPLE_APP_QNX
#define COMPATIBLE_SAMPLE_APP "nvidia,tegra-fsicom-qnx-sampleApp1"
#else

char compatible_string[32][50] = {
  "nvidia,tegra-fsicom-CAMERA_DETECTION",
  "nvidia,tegra-fsicom-LOCALIZATION",
  "nvidia,tegra-fsicom-FTP",
  "nvidia,tegra-fsicom-CP_APA_PLANNING",
  "nvidia,tegra-fsicom-CP_APA_PLANNING1",
  "nvidia,tegra-fsicom-NP_PLANNING",
  "nvidia,tegra-fsicom-CONTROL",
  "nvidia,tegra-fsicom-LIDAR_PERCEPTION",
  "nvidia,tegra-fsicom-VEHICLE_SERVICE",
  "nvidia,tegra-fsicom-SENSOR_SERVICE",
  "nvidia,tegra-fsicom-SENSOR_SERVICE1",
  "nvidia,tegra-fsicom-RESERVED1",
  "nvidia,tegra-fsicom-RESERVED2",
  "nvidia,tegra-fsicom-RESERVED3",
  "nvidia,tegra-fsicom-RESERVED4",
  "nvidia,tegra-fsicom-RESERVED5",
  "nvidia,tegra-fsicom-RESERVED6",
  "nvidia,tegra-fsicom-RESERVED7",
  "nvidia,tegra-fsicom-RESERVED8",
  "nvidia,tegra-fsicom-RESERVED9",
  "nvidia,tegra-fsicom-RESERVED10",
  "nvidia,tegra-fsicom-RESERVED11",
  "nvidia,tegra-fsicom-RESERVED12",
  "nvidia,tegra-fsicom-RESERVED13",
  "nvidia,tegra-fsicom-RESERVED14",
  "nvidia,tegra-fsicom-RESERVED15",
  "nvidia,tegra-fsicom-RESERVED16",
  "nvidia,tegra-fsicom-RESERVED17",
  "nvidia,tegra-fsicom-RESERVED18",
  "nvidia,tegra-fsicom-RESERVED19",
  "nvidia,tegra-fsicom-RESERVED20",
  "nvidia,tegra-fsicom-RESERVED21"
};

#endif

/*==================[static variables]========================================*/

/**
 * Receiver thread for DemoApp
 */
static pthread_t sgRxThreadHandle;

/**
 * FsiCom Channel Handle
 */
// static NvFsiComHandle_t sgFsiComHandle[DA_FSI_COM_CHANNEL];

/* ==================[Definition of static functions]======================== */

/**
 * @brief Handler for Rx Thread
 *
 * - <b>Description</b>\n
 *   Rx Handler
 *
 * @param arg (in)
 *
 * @return None
 *
 */
static void * lRxThread(void *arg)
{
  int32_t lRet;
  int8_t  lRxBuff[1024];
  int32_t lReadSize;
  int32_t lMessageLength=128;
  int8_t* lMsgData;
  

  while(1)
  {
    /* waiting for Receive event from Fsi-ccplex-com
       RECEIVE-blocked state until a message arrives */
    
  }
  return NULL;
}

int32_t main(int argc, char *argv[])
{
  int32_t lRet;
  lRet = pthread_create(&sgRxThreadHandle, NULL, lRxThread, NULL);

  if (0 != lRet)
  {
    printf("Failed to create Rx thread: %s\n", strerror(lRet));
    return -1;
  }
  printf("create rx threaad done\n");

  sleep(3);

return 0;
}
