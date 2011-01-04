target remote localhost:8888
dir arch/arm/cpu/arm920t/s3c24x0
#add-symbol-file u-boot 0x33f80000
add-symbol-file u-boot 0x33f55000
#add-symbol-file u-boot 0x32000000
monitor halt
dir arch/arm/lib
b bootm.c:85
