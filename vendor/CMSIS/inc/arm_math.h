/******************************************************************************
 * @file     arm_math.h
 * @brief    Public header file for CMSIS DSP Library
 * @version  V1.7.0
 * @date     18. March 2019
 ******************************************************************************/
/*
 * Copyright (c) 2010-2019 Arm Limited or its affiliates. All rights reserved.
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


#ifndef _ARM_MATH_H
#define _ARM_MATH_H

#ifdef   __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>

#ifndef PI
  #define PI               3.14159265358979f
#endif

#define FAST_MATH_TABLE_SIZE  512


  typedef enum
  {
    ARM_MATH_SUCCESS        =  0,        /**< No error */
    ARM_MATH_ARGUMENT_ERROR = -1,        /**< One or more arguments are incorrect */
    ARM_MATH_LENGTH_ERROR   = -2,        /**< Length of data buffer is incorrect */
    ARM_MATH_SIZE_MISMATCH  = -3,        /**< Size of matrices is not compatible with the operation */
    ARM_MATH_NANINF         = -4,        /**< Not-a-number (NaN) or infinity is generated */
    ARM_MATH_SINGULAR       = -5,        /**< Input matrix is singular and cannot be inverted */
    ARM_MATH_TEST_FAILURE   = -6         /**< Test Failed */
  } arm_status;

  /**
   * @brief 32-bit floating-point type definition.
   */
  typedef float float32_t;


  typedef struct
  {
    uint16_t numRows;     /**< number of rows of the matrix.     */
    uint16_t numCols;     /**< number of columns of the matrix.  */
    float32_t *pData;     /**< points to the data of the matrix. */
  } arm_matrix_instance_f32;


arm_status arm_mat_trans_f32(
  const arm_matrix_instance_f32 * pSrc,
        arm_matrix_instance_f32 * pDst);

  /**
   * @brief Q15 matrix transpose.
   * @param[in]  pSrc  points to the input matrix
   * @param[out] pDst  points to the output matrix
   * @return    The function returns either  <code>ARM_MATH_SIZE_MISMATCH</code>
   * or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */

  /**
   * @brief Floating-point matrix inverse.
   * @param[in]  src   points to the instance of the input floating-point matrix structure.
   * @param[out] dst   points to the instance of the output floating-point matrix structure.
   * @return The function returns ARM_MATH_SIZE_MISMATCH, if the dimensions do not match.
   * If the input matrix is singular (does not have an inverse), then the algorithm terminates and returns error status ARM_MATH_SINGULAR.
   */
arm_status arm_mat_inverse_f32(
  const arm_matrix_instance_f32 * src,
  arm_matrix_instance_f32 * dst);

  /**
   * @brief Floating-point matrix multiplication
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
arm_status arm_mat_mult_f32(
  const arm_matrix_instance_f32 * pSrcA,
  const arm_matrix_instance_f32 * pSrcB,
        arm_matrix_instance_f32 * pDst);

  /**
   * @brief Floating-point matrix scaling.
   * @param[in]  pSrc   points to the input matrix
   * @param[in]  scale  scale factor
   * @param[out] pDst   points to the output matrix
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
arm_status arm_mat_scale_f32(
  const arm_matrix_instance_f32 * pSrc,
        float32_t scale,
        arm_matrix_instance_f32 * pDst);

  /**
   * @brief  Fast approximation to the trigonometric sine function for floating-point data.
   * @param[in] x  input value in radians.
   * @return  sin(x).
   */
float32_t arm_sin_f32(
  float32_t x);


  /**
   * @brief  Fast approximation to the trigonometric cosine function for floating-point data.
   * @param[in] x  input value in radians.
   * @return  cos(x).
   */
float32_t arm_cos_f32(
  float32_t x);


/**
  @brief         Floating-point square root function.
  @param[in]     in    input value
  @param[out]    pOut  square root of input value
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : input value is positive
                   - \ref ARM_MATH_ARGUMENT_ERROR : input value is negative; *pOut is set to 0
 */
static inline arm_status arm_sqrt_f32(
  float32_t in,
  float32_t * pOut)
  {
    if (in >= 0.0f)
    {
      *pOut = sqrtf(in);

      return (ARM_MATH_SUCCESS);
    }
    else
    {
      *pOut = 0.0f;
      return (ARM_MATH_ARGUMENT_ERROR);
    }
  }

#endif
