# LexxPluss Power Board Software

## Install Mbed CLI 2 (mbed-tools).
https://os.mbed.com/docs/mbed-os/v6.14/build-tools/install-or-upgrade.html

## Install ARM embedded toolchain
https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

## Build

```bash
$ git clone https://github.com/LexxPluss/mbed-os-powerboard.git
$ cd mbed-os-powerboard
$ mbed-tools deploy
$ mbed-tools compile -m LEXXPLUSS_PB01 -t GCC_ARM
```

## Install STLINK Tools

```bash
$ brew install stlink
```

## Program

```bash
$ st-flash --connect-under-reset cmake_build/LEXXPLUSS_PB01/develop/GCC_ARM/mbed-os-powerboard.bin 0x8000000
```
