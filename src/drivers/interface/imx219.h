/*
FILENAME: imx219.h
AUTHOR: Greg Taylor     CREATION DATE: 12 Aug 2019

DESCRIPTION:

CHANGE HISTORY:
12 Aug 2019		Greg Taylor
	Initial version

MIT License

Copyright (c) 2019 Greg Taylor <gtaylor@sonic.net>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#ifndef SRC_IMX219_H_
#define SRC_IMX219_H_

#include "xscugic.h"

#define VIDEO_COLUMNS   640
#define VIDEO_ROWS      480

#define IMX219_I2C_SLAVE_ADDR  0x10
#define IMX219_I2C_TIMEOUT_US 40000

#define IMX219_MODEL_ID_MSB_REG 0x0000
#define IMX219_MODEL_ID_LSB_REG 0x0001
#define IMX019_MODEL_ID 0x0219

/* Lens */
/* infinity, 0 current */
#define IMX219_LENS_MIN					0
/* macro, max current */
#define IMX219_LENS_MAX					255
#define IMX219_LENS_STEP				1
/* AEC */
#define IMX219_DEFAULT_EXP		                10000
#define IMX219_DEFAULT_GAIN		                UINT8P8(1.0)
#define IMX219_GAIN_MIN					UINT8P8(1.0)
#define IMX219_GAIN_MAX					UINT8P8(8.0)
#define IMX219_EXP_MIN					1
#define IMX219_ANA_GAIN_GLOBAL                          0x0157
#define IMX219_COARSE_INT_TIME_HI                       0x015A
#define IMX219_COARSE_INT_TIME_LO                       0x015B
#define IMX219_FRM_LENGTH_HI                            0x0160
#define IMX219_FRM_LENGTH_LO                            0x0161

int32_t imx219_init(uint8_t expander_channel);


#endif /* SRC_IMX219_H_ */
