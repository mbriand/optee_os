PLATFORM_FLAVOR ?= mx6ulevk

# Get SoC associated with the PLATFORM_FLAVOR
ifeq ($(PLATFORM_FLAVOR),$(filter $(PLATFORM_FLAVOR),mx6ulevk))
$(call force,CFG_MX6UL,y)
endif

ifeq ($(PLATFORM_FLAVOR),$(filter $(PLATFORM_FLAVOR),mx6qsabrelite mx6qsabresd))
$(call force,CFG_MX6Q,y)
endif

ifeq ($(PLATFORM_FLAVOR),$(filter $(PLATFORM_FLAVOR),mx6dlsabresd))
$(call force,CFG_MX6DL,y)
endif


# Common i.MX6 config
arm32-platform-cflags		+= -mcpu=$(arm32-platform-cpuarch)
arm32-platform-aflags		+= -mcpu=$(arm32-platform-cpuarch)
core_arm32-platform-aflags	+= -mfpu=neon

$(call force,CFG_ARM32_core,y)
$(call force,CFG_GENERIC_BOOT,y)
$(call force,CFG_GIC,y)
$(call force,CFG_IMX_UART,y)
$(call force,CFG_PM_STUBS,y)
$(call force,CFG_WITH_SOFTWARE_PRNG,y)

CFG_CRYPTO_SIZE_OPTIMIZATION ?= n
CFG_WITH_STACK_CANARIES ?= y


# i.MX6UL specific config
ifeq ($(filter y, $(CFG_MX6UL)), y)
arm32-platform-cpuarch		:= cortex-a7

$(call force,CFG_SECURE_TIME_SOURCE_CNTPCT,y)
endif


# i.MX6 Solo/DualLite/Dual/Quad specific config
ifeq ($(filter y, $(CFG_MX6Q) $(CFG_MX6D) $(CFG_MX6DL) $(CFG_MX6S)), y)
arm32-platform-cpuarch		:= cortex-a9

$(call force,CFG_PL310,y)
$(call force,CFG_PL310_LOCKED,y)
$(call force,CFG_SECURE_TIME_SOURCE_REE,y)

CFG_BOOT_SYNC_CPU ?= y
CFG_BOOT_SECONDARY_REQUEST ?= y
endif


ta-targets = ta_arm32

