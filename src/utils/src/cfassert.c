/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * cfassert.c - Assert implementation
 */

#define DEBUG_MODULE "SYS"

#include <stdint.h>
#include "FreeRTOS.h"
#include "cfassert.h"
// #include "motors.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xresetps.h"

#define CORESIGHT_RPU_ROM_DSCRext_ADDR 0xFEBF0088
#define CORESIGHT_RPU_ROM_DSCRext_HDBGEN_MSK 0x4000

XResetPs ResetInstance;	


#define MAGIC_ASSERT_INDICATOR 0x2f8a001f

enum snapshotType_e
{
  SnapshotTypeNone = 0,
  SnapshotTypeFile = 1,
  SnapshotTypeHardFault = 2,
  SnapshotTypeText = 3,
};

typedef struct SNAPSHOT_DATA {
  uint32_t magicNumber;
  enum snapshotType_e type;
  union {
    struct {
      const char* fileName;
      int line;
    } file;
    struct {
      unsigned int r0;
      unsigned int r1;
      unsigned int r2;
      unsigned int r3;
      unsigned int r12;
      unsigned int lr;
      unsigned int pc;
      unsigned int psr;
    } hardfault;
    struct {
      const char* text;
    } text;
  };
} SNAPSHOT_DATA;

// The .nzds section is not cleared at startup, data here will survive a
// reset (by the watch dog for instance)
SNAPSHOT_DATA snapshot __attribute__((section(".nzds"))) = {
  .magicNumber = 0,
  .type = SnapshotTypeNone,
};

static enum snapshotType_e currentType = SnapshotTypeNone;


void assertFail(char *exp, char *file, int line)
{
  portDISABLE_INTERRUPTS();
  storeAssertFileData(file, line);
  xil_printf("Assert failed %s:%d\r\n", file, line);

  // motorsStop();

  u32 dscr = Xil_In32(CORESIGHT_RPU_ROM_DSCRext_ADDR);
  if(!(dscr & CORESIGHT_RPU_ROM_DSCRext_HDBGEN_MSK))
  {
    // Only reset if debugger is not connected
    int status;
    XResetPs_Config *configPtr;
    configPtr = XResetPs_LookupConfig(XPAR_XRESETPS_DEVICE_ID);
    status = XResetPs_CfgInitialize(&ResetInstance, configPtr);
    if (status != XST_SUCCESS) {
      xil_printf("ResetPs configuration failed\r\n");
    }
    // XResetPs_ResetPulse(&ResetInstance, XRESETPS_RSTID_RPU_LS);
  }
}

void storeAssertFileData(const char *file, int line)
{
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.type = SnapshotTypeFile;
  currentType = snapshot.type;
  snapshot.file.fileName = file;
  snapshot.file.line = line;
}

void storeAssertHardfaultData(
    unsigned int r0,
    unsigned int r1,
    unsigned int r2,
    unsigned int r3,
    unsigned int r12,
    unsigned int lr,
    unsigned int pc,
    unsigned int psr)
{
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.type = SnapshotTypeHardFault;
  currentType = snapshot.type;
  snapshot.hardfault.r0 = r0;
  snapshot.hardfault.r1 = r1;
  snapshot.hardfault.r2 = r2;
  snapshot.hardfault.r3 = r3;
  snapshot.hardfault.r12 = r12;
  snapshot.hardfault.lr = lr;
  snapshot.hardfault.pc = pc;
  snapshot.hardfault.psr = psr;
}

void storeAssertTextData(const char *text)
{
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.type = SnapshotTypeText;
  currentType = snapshot.type;
  snapshot.text.text = text;
}

static void clearAssertData() {
  snapshot.magicNumber = 0;
}

void printAssertSnapshotData()
{
  switch (currentType) {
    case SnapshotTypeNone:
      xil_printf("No assert information found\r\n");
      break;
    case SnapshotTypeFile:
      xil_printf("Assert failed at %s:%d\r\n", snapshot.file.fileName, snapshot.file.line);
      break;
    case SnapshotTypeHardFault:
      xil_printf("Hardfault. r0: %X, r1: %X, r2: %X, r3: %X, r12: %X, lr: %X, pc: %X, psr: %X\r\n",
        snapshot.hardfault.r0,
        snapshot.hardfault.r1,
        snapshot.hardfault.r2,
        snapshot.hardfault.r3,
        snapshot.hardfault.r12,
        snapshot.hardfault.lr,
        snapshot.hardfault.pc,
        snapshot.hardfault.psr);
      break;
    case SnapshotTypeText:
      xil_printf("Assert failed: %s\r\n", snapshot.text.text);
      break;
    default:
      xil_printf("Assert failed, but unknown type\r\n");
      break;
  }
}

static bool isAssertRegistered() {
  return (MAGIC_ASSERT_INDICATOR == snapshot.magicNumber) && (snapshot.type != SnapshotTypeNone);
}

bool cfAssertNormalStartTest(void) {
  bool wasNormalStart = true;

	if (isAssertRegistered()) {
		wasNormalStart = false;
    currentType = snapshot.type;
		xil_printf("The system resumed after a failed assert [WARNING]\r\n");
    clearAssertData();
		printAssertSnapshotData();
	}

	return wasNormalStart;
}
