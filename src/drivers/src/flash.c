/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

#include "xparameters.h"
#include <stdio.h>
#include "xil_printf.h"
#include "xstatus.h"
#include "ff.h"
#include "xil_cache.h"
#include "flash.h"

#define FILE_PREFIX "1:/"
#define FILE_SUFFIX ".raw"

static FIL fil;
static FATFS fatfs;
MKFS_PARM mkfs_parm;


int flash_write(uint8_t *source, uint32_t size, char name[32]) {
    FRESULT Res;
    UINT NumBytesRead;
    UINT NumBytesWritten;
    UINT NumberFiles = 0;
    char NumberFiles_str[5];
    char prefix[6] = FILE_PREFIX;
    char suffix[6] = FILE_SUFFIX;
    char FileName[32];
    char pattern[32];
    BYTE work[FF_MAX_SS];
    DIR dp;
    FILINFO fno;

    /* mount logical device */
    Res = f_mount(&fatfs, prefix, 0);
    if (Res != FR_OK) {
        xil_printf("ERROR: mounting sd card file system on `%s` failed\r\n", prefix);
        return XST_FAILURE;
    }
    xil_printf("Mounted sd card file system on `%s` \r\n", prefix);
    
    /* find all <name>*.raw files */
    strcpy(pattern, name);
    strcat(pattern, "*");
    strcat(pattern, suffix);
    Res = f_findfirst(&dp, &fno, prefix, pattern);
    while (Res == FR_OK && fno.fname[0]) {
        NumberFiles++;
        Res = f_findnext(&dp, &fno);
    }

    /* create new .raw file */
    sprintf(NumberFiles_str, "%d", NumberFiles);
    strcpy(FileName, prefix);
    strcat(FileName, name);
    strcat(FileName, NumberFiles_str);
    strcat(FileName, suffix);
    Res = f_open(&fil, FileName, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    if (Res) {
        xil_printf("ERROR: could not open file\r\n");
        return XST_FAILURE;
    }

    /* pointer to beginning of file */
    Res = f_lseek(&fil, 0);
    if (Res) {
        xil_printf("ERROR: file seek failed\r\n");
        return XST_FAILURE;
    }

    xil_printf("writing %d bytes to %s ...", size, &FileName[3]);

    /* write data to file */
    Res = f_write(&fil, (const void*)source, size,
            &NumBytesWritten);
    if (Res) {
        return XST_FAILURE;
    }

    /* pointer to beginning of file */
    Res = f_lseek(&fil, 0);
    if (Res) {
        return XST_FAILURE;
    }

    /* close file */
    Res = f_close(&fil);
    if (Res) {
        return XST_FAILURE;
    }

    xil_printf("done\r\n");

    return XST_SUCCESS;
}
