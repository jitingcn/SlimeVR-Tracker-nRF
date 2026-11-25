sysbuild_get(mcuboot_accepts_boot_mode IMAGE mcuboot VAR CONFIG_BOOT_SERIAL_BOOT_MODE KCONFIG)
set_config_bool(${ZCMAKE_APPLICATION} CONFIG_MCUBOOT_SERIAL_BOOT_MODE_AVAILABLE
                mcuboot_accepts_boot_mode)
