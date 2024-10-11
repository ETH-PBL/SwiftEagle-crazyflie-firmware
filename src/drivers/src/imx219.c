/*
FILENAME: imx219.c
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
#include "sleep.h"
#include "platform.h"
#include "xgpiops.h"
#include "FreeRTOSConfig.h"
#include "iic_mipi.h"
#include "imx219.h"

static XGpioPs gpio;
static mipi_interface imx219;

static void iic_write(uint16_t reg, uint8_t buf);
static uint8_t iic_read(uint16_t reg);


int32_t imx219_init(uint8_t expander_channel) {
   
    imx219.expander_channel = expander_channel;
    imx219.address = IMX219_I2C_SLAVE_ADDR;
    imx219.buffer_length = 1;

    uint8_t id_msb, id_lsb;
    id_msb = iic_read(IMX219_MODEL_ID_MSB_REG);
    id_lsb = iic_read(IMX219_MODEL_ID_LSB_REG);
    xil_printf("imx219: model id expected 0x%04x got 0x%04x on mux channel 0x%02x\r\n", 
        IMX019_MODEL_ID, (id_msb<<8) + id_lsb, expander_channel);
    if ((id_msb<<8) + id_lsb != IMX019_MODEL_ID) {
        return XST_FAILURE;
    }

    // config from https://github.com/Xilinx/linux-xlnx/blob/master/drivers/media/i2c/imx219.c
    /* 640x480P48 */
    iic_write(0x30EB, 0x05);
    iic_write(0x30EB, 0x0C);
    iic_write(0x300A, 0xFF);
    iic_write(0x300B, 0xFF);
    iic_write(0x30EB, 0x05);
    iic_write(0x30EB, 0x09);
    iic_write(0x0114, 0x01); // 2-wire csi
    iic_write(0x0128, 0x00); // auto MIPI global timing
    iic_write(0x012A, 0x18); // INCK freq: 24.0Mhz
    iic_write(0x012B, 0x00);
    iic_write(0x0160, 0x04); // frame length lines = 1113
    iic_write(0x0161, 0x59);
    iic_write(0x0162, 0x0D); // line length pixels = 3448
    iic_write(0x0163, 0x78);
    iic_write(0x0164, 0x01); // x-start address = 360
    iic_write(0x0165, 0x68);
    iic_write(0x0166, 0x0b); // x-end address = 2919
    iic_write(0x0167, 0x67);
    iic_write(0x0168, 0x01); // y-start address = 272
    iic_write(0x0169, 0x10);
    iic_write(0x016A, 0x08); // y-end address = 2191
    iic_write(0x016B, 0x8F);
    iic_write(0x016C, 0x02); // x-output size = 640
    iic_write(0x016D, 0x80);
    iic_write(0x016E, 0x01); // y-output size = 480
    iic_write(0x016F, 0xE0);
    iic_write(0x0170, 0x01); //
    iic_write(0x0171, 0x01);
    iic_write(0x0174, 0x02); // binning 4x
    iic_write(0x0175, 0x02); // bininng 4x
    iic_write(0x018C, 0x0A); // csi-2 data format = 10bit
    iic_write(0x018D, 0x0A);
    iic_write(0x0301, 0x05); // video timing pixel clock divider value = 5
    iic_write(0x0303, 0x01); // video timing system clock divider value = 1
    iic_write(0x0304, 0x03); // external clock 24-27MHz
    iic_write(0x0305, 0x03); // external clock 24-27MHz
    iic_write(0x0306, 0x00); // PLL Video Timing system multiplier value = 57
    iic_write(0x0307, 0x39);
    iic_write(0x0309, 0x0A); // output pixel clock divider value = 10
    iic_write(0x030B, 0x01); // output system clock divider value = 1
    iic_write(0x030C, 0x00); // PLL output system multiplier value = 114
    iic_write(0x030D, 0x72);
    iic_write(0x455E, 0x00);
    iic_write(0x471E, 0x4B);
    iic_write(0x4767, 0x0F);
    iic_write(0x4750, 0x14);
    iic_write(0x4540, 0x00);
    iic_write(0x47B4, 0x14);
    iic_write(0x4713, 0x30);
    iic_write(0x478B, 0x10);
    iic_write(0x478F, 0x10);
    iic_write(0x4793, 0x10);
    iic_write(0x4797, 0x0E);
    iic_write(0x479B, 0x0E);
    iic_write(0x0100, 0x01);
    xil_printf("imx219 (0x%02x): wrote initial configuration\r\n", expander_channel);

    iic_write(IMX219_ANA_GAIN_GLOBAL, 232);

    // iic_write(IMX219_COARSE_INT_TIME_HI, 0x02);

    return XST_SUCCESS;
}

static void iic_write(uint16_t reg, uint8_t buf)
{
    iic_mipi_write(&imx219, reg, &buf);
}

static uint8_t iic_read(uint16_t reg)
{
    uint8_t buf[1];

    iic_mipi_read(&imx219, reg, buf);

    return buf[0];
}
