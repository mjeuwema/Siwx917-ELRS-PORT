####################################################################
# Automatically-generated file. Do not edit!                       #
####################################################################

set(SDK_PATH "C:/Users/mjeuw/.silabs/slt/installs/conan/p/simpleb526998f4a4d/p")
set(COPIED_SDK_PATH "simplicity_sdk_2025.6.2")
set(PKG_PATH "C:/Users/mjeuw/.silabs/slt/installs")

add_library(slc OBJECT
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/board/silabs/src/rsi_board.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/common/src/sl_utility.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/chip/src/iPMU_prog/iPMU_dotc/ipmu_apis.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/chip/src/iPMU_prog/iPMU_dotc/rsi_system_config_917.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/chip/src/rsi_deepsleep_soc.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/chip/src/rsi_ps_ram_func.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/chip/src/startup_si91x.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/chip/src/system_si91x.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/common/src/malloc_thread_safety.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/common/src/rsi_debug.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/common/src/sl_si91x_stack_object_declare.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/common/src/syscalls.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/config/src/rsi_nvic_priorities_config.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/cmsis_driver/UDMA.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/cmsis_driver/USART.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/aux_reference_volt_config.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/clock_update.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_adc.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_crc.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_dac.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_egpio.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_opamp.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_udma.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_udma_wrapper.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/src/rsi_usart.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/service/clock_manager/src/sl_si91x_clock_manager.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/systemlevel/src/rsi_bod.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/systemlevel/src/rsi_ipmu.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/systemlevel/src/rsi_pll.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/systemlevel/src/rsi_rtc.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/systemlevel/src/rsi_temp_sensor.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/systemlevel/src/rsi_time_period.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/systemlevel/src/rsi_ulpss_clk.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/unified_api/src/sl_si91x_adc.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/unified_api/src/sl_si91x_bjt_temperature_sensor.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/unified_api/src/sl_si91x_dma.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/ahb_interface/src/rsi_hal_mcu_m4_ram.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/ahb_interface/src/rsi_hal_mcu_m4_rom.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/ahb_interface/src/sl_platform.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/ahb_interface/src/sl_platform_wireless.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/ahb_interface/src/sl_si91x_bus.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/ahb_interface/src/sl_si91x_timer.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/ahb_interface/src/sli_siwx917_soc.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/errno/src/sl_si91x_errno.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/firmware_upgrade/firmware_upgradation.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/host_mcu/si91x/siwx917_soc_ncp_host.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/icmp/sl_net_ping.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/memory/mem_pool_buffer_quota.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/sl_net/src/sl_net_rsi_utility.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/sl_net/src/sl_net_si91x.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/sl_net/src/sl_net_si91x_callback_framework.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/sl_net/src/sl_net_si91x_integration_handler.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/sl_net/src/sl_si91x_net_internal_stack.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/sl_net/src/sli_net_si91x_utility.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/socket/src/sl_si91x_socket_utility.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/src/sl_rsi_utility.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/src/sl_si91x_driver.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/protocol/wifi/si91x/sl_wifi.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/protocol/wifi/src/sl_wifi_basic_credentials.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/protocol/wifi/src/sl_wifi_callback_framework.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/service/network_manager/src/sl_net.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/service/network_manager/src/sl_net_basic_certificate_store.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/service/network_manager/src/sl_net_basic_profiles.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/service/network_manager/src/sl_net_credentials.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/service/network_manager/src/sli_net_common_utility.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/sli_si91x_wifi_event_handler/src/sli_si91x_wifi_event_handler.c"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/sli_wifi_command_engine/src/sli_wifi_command_engine.c"
    "${SDK_PATH}/platform/CMSIS/RTOS2/Source/os_systick.c"
    "${SDK_PATH}/platform/common/src/sl_assert.c"
    "${SDK_PATH}/platform/common/src/sl_cmsis_os2_common.c"
    "${SDK_PATH}/platform/common/src/sl_core_cortexm.c"
    "${SDK_PATH}/platform/common/src/sl_slist.c"
    "${SDK_PATH}/platform/common/src/sl_string.c"
    "${SDK_PATH}/platform/common/src/sli_cmsis_os2_ext_task_register.c"
    "${SDK_PATH}/platform/service/mem_pool/src/sl_mem_pool.c"
    "${SDK_PATH}/platform/service/sl_main/src/rtos/main_retarget.c"
    "${SDK_PATH}/platform/service/sl_main/src/sl_main_init.c"
    "${SDK_PATH}/platform/service/sl_main/src/sl_main_init_memory.c"
    "${SDK_PATH}/platform/service/sl_main/src/sl_main_kernel.c"
    "${SDK_PATH}/util/third_party/freertos/cmsis/Source/cmsis_os2.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/croutine.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/event_groups.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/list.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/portable/GCC/ARM_CM4F/port.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/portable/IAR/ARM_CM4F/port.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/portable/IAR/ARM_CM4F/portasm.s"
    "${SDK_PATH}/util/third_party/freertos/kernel/portable/MemMang/heap_4.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/queue.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/stream_buffer.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/tasks.c"
    "${SDK_PATH}/util/third_party/freertos/kernel/timers.c"
    "../app.c"
    "../autogen/sl_event_handler.c"
    "../main.c"
)

target_include_directories(slc PUBLIC
   "../config"
   "../autogen"
   "../."
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/protocol/wifi/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/service/bsd_socket/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/cmsis_driver/CMSIS/Driver/Include"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/cmsis_driver"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/common/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/service/network_manager/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/rom_driver/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/chip/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/peripheral_drivers/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/board/silabs/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/core/config"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/unified_api/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/service/clock_manager/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/errno/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/firmware_upgrade"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/sl_net/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/icmp"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/socket/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/inc/mqtt/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/wireless/ahb_interface/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/sli_wifi_command_engine/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/device/silabs/si91x/mcu/drivers/systemlevel/inc"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/resources/defaults"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/resources/certificates"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/resources/html"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/resources/other"
    "${SDK_PATH}/../../wisece6a05cd369ee2/p/components/common/inc"
    "${SDK_PATH}/platform/common/inc"
    "${SDK_PATH}/platform/CMSIS/Core/Include"
    "${SDK_PATH}/platform/CMSIS/RTOS2/Include"
    "${SDK_PATH}/platform/emlib/inc"
    "${SDK_PATH}/util/third_party/freertos/kernel/include"
    "${SDK_PATH}/util/third_party/freertos/cmsis/Include"
    "${SDK_PATH}/util/third_party/freertos/kernel/portable/IAR/ARM_CM4F"
    "${SDK_PATH}/util/third_party/freertos/kernel/portable/GCC/ARM_CM4F"
    "${SDK_PATH}/platform/service/mem_pool/inc"
    "${SDK_PATH}/platform/service/sl_main/inc"
    "${SDK_PATH}/platform/service/sl_main/src"
)

target_compile_definitions(slc PUBLIC
    "SL_SI91X_PRINT_DBG_LOG=1"
    "SIWG917Y111MGABA=1"
    "SLI_SI917=1"
    "SLI_SI917B0=1"
    "SLI_SI91X_MCU_ENABLE_FLASH_BASED_EXECUTION=1"
    "SLI_SI91X_MCU_EXTERNAL_LDO_FOR_PSRAM=1"
    "SL_SI91X_ACX_MODULE=1"
    "SI917Y_DEVKIT=1"
    "SLI_SI91X_MCU_COMMON_FLASH_MODE=1"
    "SLI_SI91X_MCU_CONFIG_RADIO_BOARD_BASE_VER=1"
    "SLI_SI91X_MCU_CONFIG_RADIO_BOARD_VER2=1"
    "SL_BOARD_NAME=\"BRD2708A\""
    "SL_BOARD_REV=\"A02\""
    "__FREERTOS_OS_WISECONNECT=1"
    "SL_NET_COMPONENT_INCLUDED=1"
    "__STATIC_INLINE=static inline"
    "CLOCK_ROMDRIVER_PRESENT=1"
    "ULPSS_CLOCK_ROMDRIVER_PRESENT=1"
    "SL_SI91X_BOARD_INIT=1"
    "SRAM_BASE=0x0cUL"
    "SRAM_SIZE=0x2fc00UL"
    "SLI_CODE_CLASSIFICATION_DISABLE=1"
    "SLI_SI91X_MCU_ENABLE_IPMU_APIS=1"
    "SL_SI91X_SOC_MODE=1"
    "CRC_ROMDRIVER_PRESENT=1"
    "SI91X_32kHz_EXTERNAL_OSCILLATOR=1"
    "DEBUG_ENABLE=1"
    "DEBUG_UART=1"
    "ENABLE_DEBUG_MODULE=1"
    "SL_SI91X_SI917_RAM_MEM_CONFIG=1"
    "UDMA_ROMDRIVER_PRESENT=1"
    "SLI_SI91X_OFFLOAD_NETWORK_STACK=1"
    "SI917=1"
    "SLI_SI91X_ENABLE_OS=1"
    "SLI_SI91X_MCU_INTERFACE=1"
    "TA_DEEP_SLEEP_COMMON_FLASH=1"
    "SLI_SI91X_SOCKETS=1"
    "PLL_ROMDRIVER_PRESENT=1"
    "SL_WIFI_COMPONENT_INCLUDED=1"
    "configNUM_SDK_THREAD_LOCAL_STORAGE_POINTERS=2"
    "SL_COMPONENT_CATALOG_PRESENT=1"
    "SL_CODE_COMPONENT_FREERTOS_KERNEL=freertos_kernel"
    "SL_CODE_COMPONENT_CORE=core"
)

target_link_libraries(slc PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/../-lgcc"
    "${CMAKE_CURRENT_LIST_DIR}/../-lnosys"
    "${CMAKE_CURRENT_LIST_DIR}/../-lc"
    "${CMAKE_CURRENT_LIST_DIR}/../-lm"
)
target_compile_options(slc PUBLIC
    $<$<COMPILE_LANGUAGE:C>:--cmse>
    $<$<COMPILE_LANGUAGE:C>:--cpu=Cortex-M4.no_dsp>
    $<$<COMPILE_LANGUAGE:C>:--cpu_mode=thumb>
    $<$<COMPILE_LANGUAGE:C>:--fpu=VFPv4_sp>
    $<$<COMPILE_LANGUAGE:C>:--endian=little>
    $<$<COMPILE_LANGUAGE:C>:-Ohz>
    $<$<COMPILE_LANGUAGE:C>:--use_c++_inline>
    $<$<COMPILE_LANGUAGE:C>:--debug>
    $<$<COMPILE_LANGUAGE:C>:-e>
    $<$<COMPILE_LANGUAGE:CXX>:--cmse>
    $<$<COMPILE_LANGUAGE:CXX>:--cpu=Cortex-M4.no_dsp>
    $<$<COMPILE_LANGUAGE:CXX>:--cpu_mode=thumb>
    $<$<COMPILE_LANGUAGE:CXX>:--fpu=VFPv4_sp>
    $<$<COMPILE_LANGUAGE:CXX>:--endian=little>
    $<$<COMPILE_LANGUAGE:CXX>:-Ohz>
    $<$<COMPILE_LANGUAGE:CXX>:--use_c++_inline>
    $<$<COMPILE_LANGUAGE:CXX>:--debug>
    $<$<COMPILE_LANGUAGE:CXX>:-e>
    $<$<COMPILE_LANGUAGE:ASM>:--cpu=Cortex-M4.no_dsp>
    $<$<COMPILE_LANGUAGE:ASM>:--cpu_mode=thumb>
    $<$<COMPILE_LANGUAGE:ASM>:--fpu=VFPv4_sp>
    $<$<COMPILE_LANGUAGE:ASM>:--endian=little>
)

set(post_build_command ${POST_BUILD_EXE} postbuild "./wifi_access_point_soc_2.slpb" --parameter build_dir:"$<TARGET_FILE_DIR:wifi_access_point_soc_2>")

target_link_options(slc INTERFACE
    --config ${CMAKE_CURRENT_LIST_DIR}/../autogen/linkerfile_SoC.ld
    --map "$<TARGET_FILE_DIR:wifi_access_point_soc_2>/wifi_access_point_soc_2.map"
)

# BEGIN_SIMPLICITY_STUDIO_METADATA=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END_SIMPLICITY_STUDIO_METADATA
