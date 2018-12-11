#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/CCStudio_v8/tirtos_c2000_2_16_01_14/packages;C:/CCStudio_v8/tirtos_c2000_2_16_01_14/products/tidrivers_c2000_2_16_01_13/packages;C:/CCStudio_v8/tirtos_c2000_2_16_01_14/products/bios_6_45_02_31/packages;C:/CCStudio_v8/tirtos_c2000_2_16_01_14/products/ndk_2_25_00_09/packages;C:/CCStudio_v8/tirtos_c2000_2_16_01_14/products/uia_2_00_05_50/packages;C:/CCStudio_v8/bios_6_70_01_03/packages
override XDCROOT = C:/CCStudio_v8/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/CCStudio_v8/tirtos_c2000_2_16_01_14/packages;C:/CCStudio_v8/tirtos_c2000_2_16_01_14/products/tidrivers_c2000_2_16_01_13/packages;C:/CCStudio_v8/tirtos_c2000_2_16_01_14/products/bios_6_45_02_31/packages;C:/CCStudio_v8/tirtos_c2000_2_16_01_14/products/ndk_2_25_00_09/packages;C:/CCStudio_v8/tirtos_c2000_2_16_01_14/products/uia_2_00_05_50/packages;C:/CCStudio_v8/bios_6_70_01_03/packages;C:/CCStudio_v8/xdctools_3_32_00_06_core/packages;..
HOSTOS = Windows
endif
