PROJECT_NAME     := secure_dfu_serial_s140_pca10056_debug
TARGETS          := nrf52840_xxaa_s140
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../../../..
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf52840_xxaa_s140.out: \
  LINKER_SCRIPT  := $(PROJ_DIR)/secure_dfu_gcc_pca10056_debug.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/libraries/bsp/bsp.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp_nfc.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(PROJ_DIR)/../dfu_req_handling/dfu-cc.pb.c \
  $(PROJ_DIR)/../dfu_req_handling/dfu_public_key.c \
  $(PROJ_DIR)/../dfu_req_handling/dfu_req_handling.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc32/crc32.c \
  $(SDK_ROOT)/components/libraries/mem_manager/mem_manager.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/queue/nrf_queue.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/sha256/sha256.c \
  $(SDK_ROOT)/components/libraries/slip/slip.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/drivers_nrf/rng/nrf_drv_rng.c \
  $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(SDK_ROOT)/components/drivers_nrf/hal/nrf_nvmc.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/toolchain/system_nrf52840.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_ecdh.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_ecdsa.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_hash.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_init.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_keys.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_mem.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_rng.c \
  $(PROJ_DIR)/main.c \
  $(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/external/nano-pb/pb_common.c \
  $(SDK_ROOT)/external/nano-pb/pb_decode.c \
  $(SDK_ROOT)/components/libraries/bootloader/serial_dfu/nrf_serial_dfu.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_ecdh.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_ecdsa.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_init.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_keys.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc/micro_ecc_lib_shared.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_crypto_sw/nrf_crypto_sw_hash.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_crypto_sw/nrf_crypto_sw_rng.c \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader_app_start.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader_app_start_asm.c \
  $(SDK_ROOT)/components/libraries/bootloader/nrf_bootloader_info.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_flash.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_handling_error.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_mbr.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_settings.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_transport.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_utils.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/libraries/bootloader/serial_dfu \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/libraries/bootloader/dfu \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/drivers_nrf/uart \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/softdevice/s140/headers/nrf52 \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/drivers_nrf/rng \
  $(SDK_ROOT)/external/micro-ecc/micro-ecc \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/bootloader \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/softdevice/s140/headers \
  $(PROJ_DIR) \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/external/segger_rtt \
  ../config \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/libraries/sha256 \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_crypto_sw \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc \
  $(PROJ_DIR)/../dfu_req_handling \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler \
  $(SDK_ROOT)/components/libraries/svc \
  $(SDK_ROOT)/external/nano-pb \
  $(SDK_ROOT)/components/libraries/log/src \

# Libraries common to all targets
LIB_FILES += \
  $(SDK_ROOT)/external/micro-ecc/nrf52_armgcc/armgcc/micro_ecc_lib_nrf52.a \

# C flags common to all targets
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DNRF_DFU_DEBUG_VERSION
CFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
CFLAGS += -DNRF_SD_BLE_API_VERSION=5
CFLAGS += -DS140
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION
CFLAGS += -DSWI_DISABLE0
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror -Os -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums -flto

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_DFU_DEBUG_VERSION
ASMFLAGS += -DNRF_DFU_SETTINGS_VERSION=1
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=5
ASMFLAGS += -DS140
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -D__HEAP_SIZE=0

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys


.PHONY: $(TARGETS) default all clean help flash 

# Default target - first one defined
default: nrf52840_xxaa_s140

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf52840_xxaa_s140

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf52840_xxaa_s140.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf52 --sectorerase
	nrfjprog --reset -f nrf52

erase:
	nrfjprog --eraseall -f nrf52
