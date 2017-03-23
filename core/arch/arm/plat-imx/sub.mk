global-incdirs-y += .
srcs-y += main.c

srcs-$(CFG_PL310) += imx_pl310.c
srcs-$(CFG_PSCI_ARM32) += psci.c

srcs-$(CFG_MX6Q) += a9_plat_init.S
srcs-$(CFG_MX6Q) += imx6.c
srcs-$(CFG_MX6D) += a9_plat_init.S
srcs-$(CFG_MX6D) += imx6.c
srcs-$(CFG_MX6DL) += a9_plat_init.S
srcs-$(CFG_MX6DL) += imx6.c
srcs-$(CFG_MX6S) += a9_plat_init.S
srcs-$(CFG_MX6S) += imx6.c

srcs-$(CFG_MX6UL) += imx6ul.c
