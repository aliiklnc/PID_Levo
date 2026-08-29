#!/bin/bash
# PID_Levo - komut satiri derleme (CubeIDE'nin kendi ARM GCC'si ile)
set -e

PRJ="/c/STM32Projects/PID_Levo"
TC="/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin"
GCC="$TC/arm-none-eabi-gcc.exe"
OBJCOPY="$TC/arm-none-eabi-objcopy.exe"
SIZE="$TC/arm-none-eabi-size.exe"
BUILD="${1:-build}"

cd "$PRJ"
mkdir -p "$BUILD"

INC="-ICore/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-IUSB_HOST/App \
-IUSB_HOST/Target \
-IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc"

DEF="-DUSE_HAL_DRIVER -DSTM32F407xx"
CPU="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb"
CFLAGS="$CPU -std=gnu11 -g3 -O2 -Wall -ffunction-sections -fdata-sections -fstack-usage"

SRCS=$(ls Core/Src/*.c \
         Drivers/STM32F4xx_HAL_Driver/Src/*.c \
         USB_HOST/App/*.c \
         USB_HOST/Target/*.c \
         Middlewares/ST/STM32_USB_Host_Library/Core/Src/*.c \
         Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Src/*.c 2>/dev/null)

OBJS=""
for f in $SRCS; do
  o="$BUILD/$(echo "$f" | tr '/' '_' | sed 's/\.c$/.o/')"
  "$GCC" $CFLAGS $DEF $INC -c "$f" -o "$o"
  OBJS="$OBJS $o"
done

# Startup (assembler)
"$GCC" $CPU -c -x assembler-with-cpp Core/Startup/startup_stm32f407vgtx.s -o "$BUILD/startup.o"
OBJS="$OBJS $BUILD/startup.o"

"$GCC" $CPU -T STM32F407VGTX_FLASH.ld --specs=nosys.specs -Wl,--gc-sections \
       -Wl,-Map="$BUILD/PID_Levo.map" -static -Wl,--start-group -lc -lm -Wl,--end-group \
       $OBJS -o "$BUILD/PID_Levo.elf"

"$OBJCOPY" -O binary "$BUILD/PID_Levo.elf" "$BUILD/PID_Levo.bin"
"$SIZE" "$BUILD/PID_Levo.elf"
echo "ELF: $BUILD/PID_Levo.elf"
