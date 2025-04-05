/**
  ******************************************************************************
  * @file    cnn_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-04-05T14:25:56+0800
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef CNN_DATA_PARAMS_H
#define CNN_DATA_PARAMS_H

#include "ai_platform.h"

/*
#define AI_CNN_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_cnn_data_weights_params[1]))
*/

#define AI_CNN_DATA_CONFIG               (NULL)


#define AI_CNN_DATA_ACTIVATIONS_SIZES \
  { 360, }
#define AI_CNN_DATA_ACTIVATIONS_SIZE     (360)
#define AI_CNN_DATA_ACTIVATIONS_COUNT    (1)
#define AI_CNN_DATA_ACTIVATION_1_SIZE    (360)



#define AI_CNN_DATA_WEIGHTS_SIZES \
  { 5864, }
#define AI_CNN_DATA_WEIGHTS_SIZE         (5864)
#define AI_CNN_DATA_WEIGHTS_COUNT        (1)
#define AI_CNN_DATA_WEIGHT_1_SIZE        (5864)



#define AI_CNN_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_cnn_activations_table[1])

extern ai_handle g_cnn_activations_table[1 + 2];



#define AI_CNN_DATA_WEIGHTS_TABLE_GET() \
  (&g_cnn_weights_table[1])

extern ai_handle g_cnn_weights_table[1 + 2];


#endif    /* CNN_DATA_PARAMS_H */
