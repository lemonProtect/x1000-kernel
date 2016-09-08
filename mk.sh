export PATH=/usr/lib/ccache:/home/yong/mips/prebuilts/toolchains/mips-gcc472-glibc216/bin:$PATH
time make uImage -j4 V=1 && time make modules -j4 V=1

rm -rf M/*

make INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=M modules_install V=1



