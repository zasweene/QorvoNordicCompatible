@echo off
cd /D D:\Downloads\dwm3000-master\examples\ex_02d_rx_sniff\Try3\zephyr\kconfig || (set FAIL_LINE=2& goto :ABORT)
C:\ncs\toolchains\b620d30767\opt\bin\cmake.exe -E env ZEPHYR_BASE=D:/Downloads/nordictest/sdk/zephyr PYTHON_EXECUTABLE=C:/ncs/toolchains/b620d30767/opt/bin/python.exe srctree=D:/Downloads/nordictest/sdk/zephyr KERNELVERSION= APPVERSION= APP_VERSION_EXTENDED_STRING= APP_VERSION_TWEAK_STRING= CONFIG_=SB_CONFIG_ KCONFIG_CONFIG=D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/Try3/zephyr/.config KCONFIG_BOARD_DIR=D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/Try3/Kconfig/boards BOARD=nrf52840dk BOARD_REVISION= BOARD_QUALIFIERS=/nrf52840 HWM_SCHEME=v2 KCONFIG_BINARY_DIR=D:/Downloads/dwm3000-master/examples/ex_02d_rx_sniff/Try3/Kconfig APPLICATION_SOURCE_DIR=D:/Downloads/nordictest/sdk/zephyr/share/sysbuild ZEPHYR_TOOLCHAIN_VARIANT= TOOLCHAIN_KCONFIG_DIR= TOOLCHAIN_HAS_NEWLIB=n TOOLCHAIN_HAS_PICOLIBC=n HIDE_CHILD_PARENT_CONFIG= EDT_PICKLE= NCS_MEMFAULT_FIRMWARE_SDK_KCONFIG=D:/Downloads/nordictest/sdk/nrf/modules/memfault-firmware-sdk/Kconfig BOARD=nrf52840dk ZEPHYR_NRF_MODULE_DIR=D:/Downloads/nordictest/sdk/nrf ZEPHYR_MCUBOOT_MODULE_DIR=D:/Downloads/nordictest/sdk/bootloader/mcuboot ZEPHYR_MCUBOOT_KCONFIG=D:/Downloads/nordictest/sdk/nrf/modules/mcuboot/Kconfig ZEPHYR_MBEDTLS_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/crypto/mbedtls ZEPHYR_MBEDTLS_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/mbedtls/Kconfig ZEPHYR_OBERON_PSA_CRYPTO_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/crypto/oberon-psa-crypto ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/tee/tf-m/trusted-firmware-m ZEPHYR_TRUSTED_FIRMWARE_M_KCONFIG=D:/Downloads/nordictest/sdk/nrf/modules/trusted-firmware-m/Kconfig ZEPHYR_PSA_ARCH_TESTS_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/tee/tf-m/psa-arch-tests ZEPHYR_SOC_HWMV1_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/soc-hwmv1 ZEPHYR_CJSON_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/cjson ZEPHYR_CJSON_KCONFIG=D:/Downloads/nordictest/sdk/nrf/modules/cjson/Kconfig ZEPHYR_AZURE_SDK_FOR_C_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/azure-sdk-for-c ZEPHYR_AZURE_SDK_FOR_C_KCONFIG=D:/Downloads/nordictest/sdk/nrf/modules/azure-sdk-for-c/Kconfig ZEPHYR_CIRRUS_LOGIC_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/hal/cirrus-logic ZEPHYR_OPENTHREAD_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/openthread ZEPHYR_OPENTHREAD_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/openthread/Kconfig ZEPHYR_SUIT_GENERATOR_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/suit-generator ZEPHYR_SUIT_PROCESSOR_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/suit-processor ZEPHYR_MEMFAULT_FIRMWARE_SDK_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/memfault-firmware-sdk ZEPHYR_COREMARK_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/benchmark/coremark ZEPHYR_COREMARK_KCONFIG=D:/Downloads/nordictest/sdk/nrf/modules/coremark/Kconfig ZEPHYR_CANOPENNODE_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/canopennode ZEPHYR_CANOPENNODE_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/canopennode/Kconfig ZEPHYR_CHRE_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/chre ZEPHYR_LZ4_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/lz4 ZEPHYR_LZ4_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/lz4/Kconfig ZEPHYR_NANOPB_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/nanopb ZEPHYR_NANOPB_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/nanopb/Kconfig ZEPHYR_TF_M_TESTS_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/tee/tf-m/tf-m-tests ZEPHYR_ZSCILIB_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/zscilib ZEPHYR_CMSIS_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/hal/cmsis ZEPHYR_CMSIS_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/cmsis/Kconfig ZEPHYR_CMSIS_DSP_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/cmsis-dsp ZEPHYR_CMSIS_DSP_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/cmsis-dsp/Kconfig ZEPHYR_CMSIS_NN_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/cmsis-nn ZEPHYR_CMSIS_NN_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/cmsis-nn/Kconfig ZEPHYR_FATFS_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/fs/fatfs ZEPHYR_FATFS_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/fatfs/Kconfig ZEPHYR_HAL_NORDIC_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/hal/nordic ZEPHYR_HAL_NORDIC_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/hal_nordic/Kconfig ZEPHYR_HAL_ST_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/hal/st ZEPHYR_HAL_ST_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/hal_st/Kconfig ZEPHYR_HAL_WURTHELEKTRONIK_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/hal/wurthelektronik ZEPHYR_HOSTAP_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/hostap ZEPHYR_HOSTAP_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/hostap/Kconfig ZEPHYR_LIBMETAL_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/hal/libmetal ZEPHYR_LIBLC3_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/liblc3 ZEPHYR_LIBLC3_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/liblc3/Kconfig ZEPHYR_LITTLEFS_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/fs/littlefs ZEPHYR_LITTLEFS_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/littlefs/Kconfig ZEPHYR_LORAMAC_NODE_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/loramac-node ZEPHYR_LORAMAC_NODE_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/loramac-node/Kconfig ZEPHYR_LVGL_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/gui/lvgl ZEPHYR_LVGL_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/lvgl/Kconfig ZEPHYR_MIPI_SYS_T_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/debug/mipi-sys-t ZEPHYR_NRF_WIFI_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/nrf_wifi ZEPHYR_NRF_WIFI_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/nrf_wifi/Kconfig ZEPHYR_OPEN_AMP_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/open-amp ZEPHYR_PICOLIBC_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/picolibc ZEPHYR_SEGGER_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/debug/segger ZEPHYR_SEGGER_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/segger/Kconfig ZEPHYR_TINYCRYPT_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/crypto/tinycrypt ZEPHYR_UOSCORE_UEDHOC_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/uoscore-uedhoc ZEPHYR_UOSCORE_UEDHOC_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/uoscore-uedhoc/Kconfig ZEPHYR_ZCBOR_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/zcbor ZEPHYR_ZCBOR_KCONFIG=D:/Downloads/nordictest/sdk/zephyr/modules/zcbor/Kconfig ZEPHYR_NRFXLIB_MODULE_DIR=D:/Downloads/nordictest/sdk/nrfxlib ZEPHYR_NRF_HW_MODELS_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/bsim_hw_models/nrf_hw_models ZEPHYR_CONNECTEDHOMEIP_MODULE_DIR=D:/Downloads/nordictest/sdk/modules/lib/matter ARCH=* ARCH_DIR=D:/Downloads/nordictest/sdk/zephyr/arch SHIELD_AS_LIST= DTS_POST_CPP= DTS_ROOT_BINDINGS= C:/ncs/toolchains/b620d30767/opt/bin/python.exe D:/Downloads/nordictest/sdk/zephyr/scripts/kconfig/guiconfig.py D:/Downloads/nordictest/sdk/zephyr/share/sysbuild/Kconfig || (set FAIL_LINE=3& goto :ABORT)
goto :EOF

:ABORT
set ERROR_CODE=%ERRORLEVEL%
echo Batch file failed at line %FAIL_LINE% with errorcode %ERRORLEVEL%
exit /b %ERROR_CODE%