#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = E:/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack;E:/ti/simplelink_cc2640r2_sdk_2_30_00_28/source;E:/ti/simplelink_cc2640r2_sdk_2_30_00_28/kernel/tirtos/packages
override XDCROOT = E:/ccs8.2/xdctools_3_50_08_24_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = E:/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack;E:/ti/simplelink_cc2640r2_sdk_2_30_00_28/source;E:/ti/simplelink_cc2640r2_sdk_2_30_00_28/kernel/tirtos/packages;E:/ccs8.2/xdctools_3_50_08_24_core/packages;..
HOSTOS = Windows
endif
