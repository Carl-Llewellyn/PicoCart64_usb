To build and flash with a j-link:

```bash

# First of all, don't forget to update submodules
git submodule update --init

# If you have more than 2MB flash, you need to change the SDK file src/rp2_common/pico_standard_link/memmap_copy_to_ram.ld (for now!):
# Change to: `FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 16384k`

cd sw

# Load a rom into sw/picocart64/rom.h
# `--compress` will compress the rom with a simple compression scheme.
./scripts/load_rom.py --compress my_rom.z64

mkdir build
cd build

# Configure cmake. To use NTSC, set -DREGION=NTSC
# If you have more than 2MB flash, you need to change the flash size by adding -DFLASH_SIZE_MB={one of 2,4,8,16} here.

cmake -DREGION=PAL ..

# Build
cmake --build .

# You can now flash it by holding down the BOOT button and resetting the device, then mounting the usb mass storage device, then copy the file picocart64/picocart64.uf2 to the mounted path and run `sync`.

# Alternatively, you can use openocd with a J-Link:

# Requires https://github.com/raspberrypi/openocd
# AUR package if you're on archlinux: https://aur.archlinux.org/packages/openocd-raspberrypi-git
openocd -f interface/jlink.cfg -c "transport select swd; adapter_khz 1000" -f target/rp2040.cfg -c "program ./picocart64/picocart64.elf verify reset exit"

# Note! You *have* to power reset the pico after flashing it with a jlink, otherwise multicore doesn't work properly.
# Alternatively, attach a debugger, `mon reset halt`, `c`, does the trick as well. It seems that openocd's `reset run` doesn't work properly.

```

## Docker (Ubuntu 24.04, Oct 2024-era toolchain)

This uses a pinned Pico SDK inside the container and expects FreeRTOS under `lib/freertos-kernel`.

Build the image (override `PICO_SDK_REF` if desired):

```bash
cd sw
PICO_SDK_REF=2.1.1 ./docker/build-image.sh
```

Clean build with a ROM path (override `REGION`, `FLASH_SIZE_MB`, `JOBS` as needed):

```bash
cd sw
./docker/clean-build.sh ~/Downloads/testrom.z64
```

Normal build (reuses existing `build/`):

```bash
cd sw
./docker/build.sh
```

If `lib/freertos-kernel` is missing, the clean build script will initialize it. To force a specific tag:

```bash
cd sw
FREERTOS_KERNEL_REF=V11.1.0 FORCE_FREERTOS_REF=1 ./docker/clean-build.sh ~/Downloads/testrom.z64
```
