/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */


#ifndef __FLASH_H__
#define __FLASH_H__


int flash_write(uint8_t *source, uint32_t size, char name[32]);


#endif /* __FLASH_H__ */

