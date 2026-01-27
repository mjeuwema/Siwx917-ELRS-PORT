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
    "${SDK_PATH}/platform/common/src/sl_syscalls.c"
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
    "-Wl,--start-group"
    "gcc"
    "nosys"
    "c"
    "m"
    "-Wl,--end-group"
)
target_compile_options(slc PUBLIC
    $<$<COMPILE_LANGUAGE:C>:-mcpu=cortex-m4>
    $<$<COMPILE_LANGUAGE:C>:-mthumb>
    $<$<COMPILE_LANGUAGE:C>:-mfpu=fpv4-sp-d16>
    $<$<COMPILE_LANGUAGE:C>:-mfloat-abi=softfp>
    $<$<COMPILE_LANGUAGE:C>:-Wall>
    $<$<COMPILE_LANGUAGE:C>:-Wextra>
    $<$<COMPILE_LANGUAGE:C>:-Os>
    $<$<COMPILE_LANGUAGE:C>:-fdata-sections>
    $<$<COMPILE_LANGUAGE:C>:-ffunction-sections>
    $<$<COMPILE_LANGUAGE:C>:-fomit-frame-pointer>
    $<$<COMPILE_LANGUAGE:C>:-g>
    "$<$<COMPILE_LANGUAGE:C>:SHELL:-Wall -Werror>"
    $<$<COMPILE_LANGUAGE:C>:-Wno-error=deprecated-declarations>
    "$<$<COMPILE_LANGUAGE:C>:SHELL:-Wall -Werror -Wno-error=deprecated-declarations>"
    $<$<COMPILE_LANGUAGE:C>:-mcpu=cortex-m4>
    $<$<COMPILE_LANGUAGE:C>:-fno-lto>
    $<$<COMPILE_LANGUAGE:C>:--specs=nano.specs>
    $<$<COMPILE_LANGUAGE:CXX>:-mcpu=cortex-m4>
    $<$<COMPILE_LANGUAGE:CXX>:-mthumb>
    $<$<COMPILE_LANGUAGE:CXX>:-mfpu=fpv4-sp-d16>
    $<$<COMPILE_LANGUAGE:CXX>:-mfloat-abi=softfp>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Os>
    $<$<COMPILE_LANGUAGE:CXX>:-fdata-sections>
    $<$<COMPILE_LANGUAGE:CXX>:-ffunction-sections>
    $<$<COMPILE_LANGUAGE:CXX>:-fomit-frame-pointer>
    $<$<COMPILE_LANGUAGE:CXX>:-g>
    "$<$<COMPILE_LANGUAGE:CXX>:SHELL:-Wall -Werror>"
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=deprecated-declarations>
    "$<$<COMPILE_LANGUAGE:CXX>:SHELL:-Wall -Werror -Wno-error=deprecated-declarations>"
    $<$<COMPILE_LANGUAGE:CXX>:-mcpu=cortex-m4>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-lto>
    $<$<COMPILE_LANGUAGE:CXX>:--specs=nano.specs>
    $<$<COMPILE_LANGUAGE:ASM>:-mcpu=cortex-m4>
    $<$<COMPILE_LANGUAGE:ASM>:-mthumb>
    $<$<COMPILE_LANGUAGE:ASM>:-mfpu=fpv4-sp-d16>
    $<$<COMPILE_LANGUAGE:ASM>:-mfloat-abi=softfp>
    "$<$<COMPILE_LANGUAGE:ASM>:SHELL:-x assembler-with-cpp>"
)

set(post_build_command ${POST_BUILD_EXE} postbuild "./wifi_access_point_soc_2.slpb" --parameter build_dir:"$<TARGET_FILE_DIR:wifi_access_point_soc_2>")
set_property(TARGET slc PROPERTY C_STANDARD 17)
set_property(TARGET slc PROPERTY CXX_STANDARD 17)
set_property(TARGET slc PROPERTY CXX_EXTENSIONS OFF)

target_link_options(slc INTERFACE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=softfp
    -T${CMAKE_CURRENT_LIST_DIR}/../autogen/linkerfile_SoC.ld
    --specs=nano.specs
    "SHELL:-Xlinker -Map=$<TARGET_FILE_DIR:wifi_access_point_soc_2>/wifi_access_point_soc_2.map"
    "SHELL:-u _printf_float"
    -Wl,--wrap=main
    -fno-lto
    -Wl,--gc-sections
)

# BEGIN_SIMPLICITY_STUDIO_METADATA=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=END_SIMPLICITY_STUDIO_METADATA