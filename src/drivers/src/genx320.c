/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

//Adapted from Metavision SDC (open source on prophesee git)

#include "sleep.h"
#include <stdio.h>

#include "iic_mipi.h"   
#include "genx320.h"
#include "genx320_definitions.h"
#include "genx320_all_pub_registers.h"

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x[0])))

static mipi_interface genx320;

static void iic_write(uint16_t reg, uint32_t buf);
static uint32_t iic_read(uint16_t reg);


int32_t genx320_init(uint8_t expander_channel)
{
    genx320.expander_channel = expander_channel;
    genx320.address = GENX320_DEV_ADDR;
    genx320.buffer_length = 4;

    return XST_SUCCESS;
}

int32_t genx320_open(uint8_t expander_channel)
{
    uint32_t read;
    genx320.expander_channel = expander_channel;

    read = iic_read(GENX320_REG_CHIPID);
    xil_printf("genx320: ID expected %08x got %08x on mux channel 0x%02x\r\n", GENX320_ID, read, expander_channel);
    if (read != GENX320_ID) {
        return XST_FAILURE;
    }

    iic_write(0x001C, 0x00000001);
    iic_write(0x9008, 0x00000320);  
    iic_write(0x9008, 0x00010320);
    iic_write(0x0200, 0x00000000);  
    iic_write(0x0214, 0x000004A0);  
    iic_write(0x0214, 0x000004A1);
    iic_write(0x0204, 0x00008E74);  
    iic_write(0x020C, 0x00000033);  
    iic_write(0x0204, 0x00000E75);  
    iic_write(0x0204, 0x00000E77);
    iic_write(0x0210, 0x00000C00); 
    iic_write(0xB00C, 0x00000002);  
    
    iic_write(0x9008, 0x00000190);  
    iic_write(0x004C, 0x00204E20); 
    iic_write(0x001C, 0x00000004);  // ??
    iic_write(0xA000, 0x00000406);  // ??
    iic_write(0xA01C, 0x0007641F);  // ??
    iic_write(0xB030, 0x00000010);  
    iic_write(0x004C, 0x00204E22);  
    usleep(100);
    iic_write(0xA01C, 0x0007E41F);  
    usleep(300);
    iic_write(0x004C, 0x00204E20);  
    iic_write(0x001C, 0x00000004);  // ??
    iic_write(0xB00C, 0x00000F02);  
    iic_write(0xB030, 0x00000013);  
    iic_write(0xB030, 0x0000001F);
    usleep(199);
    iic_write(0xB404, 0x00000068);  
    iic_write(0xB470, 0x0000000D);  // ??
    iic_write(0xB45C, 0x00000030);  // ??
    iic_write(0xB4E4, 0x00000000);  // ??
    iic_write(0xB434, 0x0000000C);  // ??
    iic_write(0xB4A0, 0x0000000C);  // ??
    iic_write(0xB010, 0x00000000);  
    iic_write(0xB000, (GENX320_CSI_PACKET_SIZE << 16) | 0xC000); 
    iic_write(0x704C, 0x00002710);  
    iic_write(0x7100, 0x0000FFFF);  
    iic_write(0x00B8, 0x00000020);  // ??
    iic_write(0x00C0, 0x000000B8);  
    iic_write(0xB000, (GENX320_CSI_PACKET_SIZE << 16) | 0xC001); 
    iic_write(0xE000, 0x00000005); 
    iic_write(0xC000, 0x00000005);  
    iic_write(0xD000, 0x00000005);  // ??
    iic_write(0x6000, 0x00000000);  
    iic_write(0x6000, 0x00000005);  
    iic_write(0x60A0, 0x00000000);  
    iic_write(0x7044, 0x00000002);  
    iic_write(0x7048, 0x00000000);  
    iic_write(0x7000, 0x00000001);  
    iic_write(0x9008, 0x00000194);
    
    iic_write(0x9000, 0x00000200);
    
    iic_write(0xB000, (GENX320_CSI_PACKET_SIZE << 16) | 0x8801);
    iic_write(0xB024, 0x80003E80);
   
    iic_write(0x001C, 0x00000004);
   
    iic_write(0x1218, 0x00000111);
    iic_write(0x1218, 0x00000111);
    iic_write(0x1218, 0x00000111);
    usleep(200);
    iic_write(0x1220, 0x00000001);
    usleep(200);
    
    iic_write(0xA008, 0x00008085);
    iic_write(0xA004, 0x00008025);
    usleep(1000);
    iic_write(0x0070, 0x0000055F);
    usleep(1000);
   
    iic_write(0x1220, 0x00000003);
    usleep(200);
   
    iic_write(0x1208, 0x00000030);
    usleep(200);
   
    iic_write(0x0000, 0x00000002);
    iic_write(0x0400, 0x01400000);
    iic_write(0x0404, 0x01400000);
    iic_write(0x0034, 0x0000000B);

    usleep(1);
    iic_write(0x1104, 0x01010018);  
    iic_write(0x1128, 0x01010018);  // ??
    iic_write(0x110C, 0x01010013); 
    iic_write(0x1130, 0x01010013);  // ??  
    iic_write(0x1000, 0x0301003D); 
    iic_write(0x1004, 0x0301001D); 
    iic_write(0x1008, 0x0101003F);  // ??
    iic_write(0x1100, 0x03010000); 
    iic_write(0x1104, 0x01010028);  
    iic_write(0x1108, 0x01010033);  
    iic_write(0x110C, 0x01010028);  
    iic_write(0x1110, 0x01010039);  // ??
    iic_write(0x1114, 0x03010052); 
    iic_write(0x1118, 0x03010042);  // ??
    iic_write(0x111C, 0x03000074);  // ??
    iic_write(0x1120, 0x010000A4);  // ??
    iic_write(0x1208, 0x00000035);  // ??
    
    
    //check PLL lock
    read = iic_read(PLL_CTRL);
    if ((read & PLL_CTRL_PL_LOCKP_Msk) > 0){
        xil_printf("genx320 (0x%02x): PLL coarse lock ok\r\n", expander_channel);
    } else {
        xil_printf("genx320 (0x%02x): PLL not locked\r\n", expander_channel);
    }
    if ((read & PLL_CTRL_PL_LOCKP_DELAYED_Msk) > 0){
        xil_printf("genx320 (0x%02x): PLL fine lock ok\r\n", expander_channel);
    } else {
        xil_printf("genx320 (0x%02x): PLL not locked\r\n", expander_channel);
    }
    usleep(1000);
   
    return XST_SUCCESS;
}

int32_t genx320_on(uint8_t expander_channel)
{
    uint32_t read;
    genx320.expander_channel = expander_channel;

    iic_write(0xB000, (GENX320_CSI_PACKET_SIZE << 16) | 0x8801);
    iic_write(0x9028, 0x00000000);
    iic_write(0x9008, 0x00000195);
    usleep(1);
    iic_write(0x002C, 0x0022C724);
    iic_write(0x0000, 0x00000C02);
    usleep(1);
    usleep(10000);
    read = iic_read(MIPI_CSI_CSI_CTRL); 
    if ((read & MIPI_CSI_CTRL_ENABLE_Msk) > 0){
        xil_printf("genx320 (0x%02x): MIPI CSI enabled\r\n", expander_channel);
    } else {
        xil_printf("genx320 (0x%02x): MIPI CSI disabled\r\n", expander_channel);
    }

    return XST_SUCCESS;
}


int32_t genx320_off(uint8_t expander_channel)
{
    uint32_t read;
    genx320.expander_channel = expander_channel;

    iic_write(0x0000, 0x00000802);
    iic_write(0x002C, 0x00200624);
    iic_write(0x9028, 0x00000002);
    usleep(1000);
    iic_write(0x9008, 0x00000194);
    iic_write(0xB000, (GENX320_CSI_PACKET_SIZE << 16) | 0x8800);

    usleep(10000);
    read = iic_read(MIPI_CSI_CSI_CTRL); 
    if ((read & MIPI_CSI_CTRL_ENABLE_Msk) > 0){
        xil_printf("genx320 (0x%02x): MIPI CSI enabled\r\n", expander_channel);
    } else {
        xil_printf("genx320 (0x%02x): MIPI CSI disabled\r\n", expander_channel);
    }

    return XST_SUCCESS;
}

static void iic_write(uint16_t reg, uint32_t buf)
{
    uint8_t writeBuffer[4];

    writeBuffer[0] = (buf >> 24) & 0xFF;
    writeBuffer[1] = (buf >> 16) & 0xFF ;
    writeBuffer[2] = (buf >>  8) & 0xFF;
    writeBuffer[3] = (buf      ) & 0xFF;

    iic_mipi_write(&genx320, reg, writeBuffer);
}

static uint32_t iic_read(uint16_t reg)
{
    uint32_t ret;
    uint8_t buf[4];

    iic_mipi_read(&genx320, reg, buf);

    ret = ((buf[3]      ) + \
           (buf[2] <<  8) + \
           (buf[1] << 16) + \
           (buf[0] << 24));

    return ret;
}
