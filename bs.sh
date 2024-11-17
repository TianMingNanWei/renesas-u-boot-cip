export ARCH=arm64 && export CROSS_COMPILE=aarch64-linux-gnu-

make distclean -j16
make smarc-rzg2ul_defconfig
make -j16

cd ../

./rz-bsp/build/tmp/sysroots-components/x86_64/fiptool-native/usr/bin/fiptool create --align 16 --soc-fw ./rz-bsp/build/tmp/deploy/images/smarc-rzg2ul/bl31-smarc-rzg2ul.bin --nt-fw ./renesas-u-boot-cip/u-boot.bin fip_pmic_$1.bin
# ./rz-bsp/build/tmp/sysroots-components/x86_64/fiptool-native/usr/bin/fiptool create --align 16 --soc-fw ./bl31-smarc-rzg2l.bin --nt-fw ./renesas-u-boot-cip/u-boot.bin fip_pmic_$1.bin

objcopy -I binary -O srec --adjust-vma=0x0000 --srec-forceS3 fip_pmic_$1.bin fip_pmic_$1.srec