/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_common_tables.h
 * Description:  Extern declaration for common tables
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _ARM_COMMON_TABLES_H
#define _ARM_COMMON_TABLES_H

#include "arm_math.h"

  /* Tables for Fast Math Sine and Cosine */
  #if !defined(ARM_DSP_CONFIG_TABLES) || defined(ARM_ALL_FAST_TABLES) || defined(ARM_TABLE_SIN_F32)
    extern const float32_t sinTable_f32[FAST_MATH_TABLE_SIZE + 1];
  #endif /* !defined(ARM_DSP_CONFIG_TABLES) defined(ARM_ALL_FAST_TABLES) */

#endif /*  ARM_COMMON_TABLES_H */

