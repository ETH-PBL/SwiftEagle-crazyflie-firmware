/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2023 Bitcraze AB
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
 *
 * usec_time.c - microsecond-resolution timer and timestamps.
 */
#include <stdbool.h>
#include "usec_time.h"
#include "cfassert.h"

static bool isInit = false;
static uint8_t reset = 0;
static uint32_t usecTimerHighCount;

void usecTimerInit(void)
{
  if (isInit) {
    return;
  }

  // todo

  isInit = true;
}

void usecTimerReset(void)
{

  // todo
}

uint64_t usecTimestamp(void)
{

  // todo
  return 1;
}
