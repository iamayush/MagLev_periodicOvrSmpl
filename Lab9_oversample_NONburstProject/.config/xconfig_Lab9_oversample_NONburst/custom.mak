## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,28FP linker.cmd package/cfg/Lab9_oversample_NONburst_p28FP.o28FP

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/Lab9_oversample_NONburst_p28FP.xdl
	$(SED) 's"^\"\(package/cfg/Lab9_oversample_NONburst_p28FPcfg.cmd\)\"$""\"C:/ayush2/repo/trunk/Lab9_oversample_NONburst/Lab9_oversample_NONburstProject/.config/xconfig_Lab9_oversample_NONburst/\1\""' package/cfg/Lab9_oversample_NONburst_p28FP.xdl > $@
	-$(SETDATE) -r:max package/cfg/Lab9_oversample_NONburst_p28FP.h compiler.opt compiler.opt.defs
