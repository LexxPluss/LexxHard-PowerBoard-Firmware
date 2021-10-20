# LexxPluss Power Board Software

## Install Mbed CLI 2 (mbed-tools).
https://os.mbed.com/docs/mbed-os/v6.15/build-tools/install-or-upgrade.html

## Install ARM embedded toolchain
https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

## Build

```bash
$ git clone https://github.com/LexxPluss/mbed-os-powerboard.git
$ cd mbed-os-powerboard
$ mbed-tools deploy
$ mbed-tools compile -m LEXXPLUSS_PB01 -t GCC_ARM
```

> macOSでプラットフォーム以外のpython (pyenvとか) を使っている場合、cmakeがプラットフォームのpythonを使おうとして正常にビルドできない場合がある。
> その場合はmbed-osに下記パッチを適用すると正常にビルドできるようになる。

```diff
diff --git a/tools/cmake/app.cmake b/tools/cmake/app.cmake
index 9f62df8204..edc4d02fca 100644
--- a/tools/cmake/app.cmake
+++ b/tools/cmake/app.cmake
@@ -31,6 +31,7 @@ enable_language(C CXX ASM)
 set(CMAKE_EXECUTABLE_SUFFIX .elf)
 
 # Find Python
+set(CMAKE_FIND_FRAMEWORK NEVER)
 find_package(Python3 COMPONENTS Interpreter)
 include(CheckPythonPackage)
```

## Install STLINK Tools

```bash
$ brew install stlink
```

## Program

バッテリー電源で書き込みを行う場合は、書き込み中に電源が切れないよう、電源スイッチを押したままの状態で書き込みを行う必要がある。

### STLINK Tools (Open souce version)

```bash
$ st-flash --connect-under-reset cmake_build/LEXXPLUSS_PB01/develop/GCC_ARM/mbed-os-powerboard.bin 0x8000000
```

### STM32CubeProgrammer

右側ST-LINKを選択して、PortはSWD、Reset modeはHardwaree reset、Connectで接続。
左側はErasing & Programming画面を表示する。
File pathで書き込みファイルを選ぶ、Start addressは0x08000000、Start Programmingで書き込み開始。

![cubeprogrammer](docs/cubeprogrammer.png)

## Power Board CAN message

### 1000 (0x3e8)

Power Board Status

| byte offset | info | detail |
|---|---|---|
| 0 | switch status bitmap | ![switch_status](docs/0_switch_status.svg) |
| 1 | charging status bitmap | ![charging_status](docs/1_charging_status.svg) |
| 2 | dc/dc failure bitmap / bmu status bitmap | ![dcdc_bmu](docs/2_dcdc_bmu.svg) |
| 3 | wheel disabled bitmap | ![wheel_status](docs/3_wheel_disable.svg) |
| 4 | fan duty | 0-100 (%) |
| 5 | charge connector temperature (Positive) | 0-100 (degC) |
| 6 | charge connector temperature (Negative) | 0-100 (degC) |
| 7 | board temperature | -50-127 (degC) |
