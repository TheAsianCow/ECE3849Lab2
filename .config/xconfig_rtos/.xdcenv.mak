#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/tirtos_tivac_2_16_00_08/packages;C:/ti/tirtos_tivac_2_16_00_08/products/tidrivers_tivac_2_16_00_08/packages;C:/ti/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages;C:/ti/tirtos_tivac_2_16_00_08/products/ndk_2_25_00_09/packages;C:/ti/tirtos_tivac_2_16_00_08/products/uia_2_00_05_50/packages;C:/ti/tirtos_tivac_2_16_00_08/products/ns_1_11_00_10/packages;C:/ti/ccs910/ccs/ccs_base;C:/Users/JEFFRE~1/DOCUME~1/WPI/2019-2~1/BTERM~1/ECE3849/labs/ece3849_lab2_jyhuang_rskirschner/.config
override XDCROOT = C:/ti/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/tirtos_tivac_2_16_00_08/packages;C:/ti/tirtos_tivac_2_16_00_08/products/tidrivers_tivac_2_16_00_08/packages;C:/ti/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages;C:/ti/tirtos_tivac_2_16_00_08/products/ndk_2_25_00_09/packages;C:/ti/tirtos_tivac_2_16_00_08/products/uia_2_00_05_50/packages;C:/ti/tirtos_tivac_2_16_00_08/products/ns_1_11_00_10/packages;C:/ti/ccs910/ccs/ccs_base;C:/Users/JEFFRE~1/DOCUME~1/WPI/2019-2~1/BTERM~1/ECE3849/labs/ece3849_lab2_jyhuang_rskirschner/.config;C:/ti/xdctools_3_32_00_06_core/packages;..
HOSTOS = Windows
endif
