#
# Copyright (C) 2024 ETH Zurich
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the GPL-3.0 license.  See the LICENSE file for details.
#


# variables
BSP_DIR		:= 		../bsp/drone/psu_cortexr5_0/r5_standalone/bsp
RTOS_DIR	:= 		../vendor/FreeRTOS-Kernel
CMSIS_DIR	:= 		../vendor/CMSIS
EXEC 		:= 		controller.elf
LIBS 		:= 		libxil.a libxilpm.a

C_SOURCES 	:= 		$(wildcard ${RTOS_DIR}/*.c)
C_SOURCES 	+=		$(RTOS_DIR)/portable/GCC/ARM_CR5/port.c
C_SOURCES 	+=		$(RTOS_DIR)/portable/MemMang/heap_4.c
C_SOURCES 	+=		$(wildcard ${CMSIS_DIR}/src/*.c)
C_SOURCES	+=		$(wildcard ./init/*.c)
C_SOURCES	+=		$(wildcard ./modules/src/*.c)
C_SOURCES	+=		$(wildcard ./modules/src/controller/*.c)
C_SOURCES	+=		$(wildcard ./modules/src/cpx/*.c)
C_SOURCES	+=		$(wildcard ./modules/src/estimator/*.c)
C_SOURCES	+=		$(wildcard ./modules/src/kalman_core/*.c)
C_SOURCES	+=		$(wildcard ./modules/src/lighthouse/*.c)
C_SOURCES	+=		$(wildcard ./modules/src/p2pDTR/*.c)
C_SOURCES	+=		$(wildcard ./utils/src/*.c)
C_SOURCES	+=		$(wildcard ./hal/src/*.c)
C_SOURCES	+=		$(wildcard ./drivers/src/*.c)
C_SOURCES	+=		$(wildcard ./drivers/bosch/src/*.c)

S_SOURCES	:=		$(RTOS_DIR)/portable/GCC/ARM_CR5/portASM.S
S_SOURCES	+=		$(wildcard ./init/*.S)

OBJS 		:= 		$(patsubst %.c, %.o, $(C_SOURCES))
OBJS		+=		$(patsubst %.S, %.o, $(S_SOURCES))

INCLUDEPATH	+= 		-I$(BSP_DIR)/psu_cortexr5_0/include 
INCLUDEPATH	+=		-I$(RTOS_DIR)/include
INCLUDEPATH	+=		-I$(RTOS_DIR)/portable/GCC/ARM_CR5
INCLUDEPATH	+=		-I$(CMSIS_DIR)/inc

INCLUDEPATH	+=		-I./config
INCLUDEPATH	+=		-I./deck/drivers/interface
INCLUDEPATH	+=		-I./deck/interface
INCLUDEPATH	+=		-I./drivers/bosch/interface
INCLUDEPATH	+=		-I./drivers/esp32/interface
INCLUDEPATH	+=		-I./drivers/interface
INCLUDEPATH	+=		-I./hal/interface
INCLUDEPATH	+=		-I./init
INCLUDEPATH	+=		-I./modules/interface
INCLUDEPATH	+=		-I./modules/interface/controller
INCLUDEPATH	+=		-I./modules/interface/cpx
INCLUDEPATH	+=		-I./modules/interface/estimator
INCLUDEPATH	+=		-I./modules/interface/kalman_core
INCLUDEPATH	+=		-I./modules/interface/lighthouse
INCLUDEPATH	+=		-I./modules/interface/p2pDTR
INCLUDEPATH	+=		-I./platform/interface
INCLUDEPATH	+=		-I./utils/interface
INCLUDEPATH	+=		-I./utils/interface/lighthouse
INCLUDEPATH	+=		-I./drivers/interface
INCLUDEPATH	+=		-I./drivers/bosch



CC      	:=		$(CROSS)armr5-none-eabi-gcc
AS      	:=		$(CROSS)armr5-none-eabi-gcc
LINKER  	:=		$(CROSS)armr5-none-eabi-gcc
DUMP    	:=		$(CROSS)armr5-none-eabi-objdump -xSD
CFLAGS 		:=		-fmessage-length=0
ECFLAGS 	:= 		-mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16
LSCRIPT 	:= 		-Tlscript.ld
LIBPATH 	:= 		$(BSP_DIR)/psu_cortexr5_0/lib
LDFLAGS 	:= 		-Wl,--start-group,-lxil,-lxilpm,-lmetal,-lxilffs,-lgcc,-lc,-lm,--end-group -L$(LIBPATH) -L./ -Wl,--build-id=none -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16


# targets
all: $(EXEC)

$(EXEC): $(LIBS) $(OBJS)
	$(LINKER) -o $@ $(OBJS) $(LDFLAGS) $(LSCRIPT)
	$(DUMP) $(EXEC)  > dump

$(LIBS):
	echo "Compiling bsp"
	mkdir -p $(BSP_DIR)/psu_cortexr5_0/lib
	make -C $(BSP_DIR)

bsp:
	rm -r -f ../bsp
	echo "Generating bsp"
	xsct -eval "setws ../bsp; \
	platform create -name drone -hw ../../programmable_logic_design/top/drone/drone_wrapper_hw_rev5.xsa -no-boot-bsp; \
	domain create -name r5_standalone -os standalone -proc psu_cortexr5_0; \
	domain active r5_standalone; \
	bsp setlib -name xilpm; \
	bsp setlib -name libmetal; \
	bsp setlib -name xilffs; \
	platform generate"

%.o:%.c
	$(CC) $(CFLAGS) $(ECFLAGS) -c $< -o $@ $(INCLUDEPATH)

%.o:%.S
	$(AS) $(CFLAGS) $(ECFLAGS) -c $< -o $@ $(INCLUDEPATH)

%.o:%.s
	$(AS) $(CFLAGS) $(ECFLAGS) -c $< -o $@ $(INCLUDEPATH)

clean:
	make clean -C $(BSP_DIR)
	rm -rf $(OBJS) *.elf dump

.PHONY: all clean
