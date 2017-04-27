CFLAGS += -c -std=gnu99
CFLAGS += -fno-common -fmessage-length=0 -fno-builtin -fno-exceptions -ffunction-sections -fdata-sections -fomit-frame-pointer -MMD -MP
CFLAGS += -DSTM32L1XX_MD -DCFG_DEBUG -DCFG_eu868 -DCFG_wimod_board -DCFG_sx1276_radio -DCFG_lmic_clib
CFLAGS += -Wno-maybe-uninitialized -Wno-maybe-uninitialized -Werror=unused-value -Wno-unused-value

