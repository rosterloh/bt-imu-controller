#!/bin/sh

../bootloader/scripts/imgtool.py sign --header-size 0x200 --align 8 --version 1.0 --slot-size 0x32000 --key ../bootloader/root-rsa-2048.pem build/zephyr/zephyr.bin build/zephyr-signed.bin
../bootloader/scripts/imgtool.py sign --header-size 0x200 --align 8 --version 1.0 --slot-size 0x32000 --key ../bootloader/root-rsa-2048.pem build/zephyr/zephyr.hex build/zephyr-signed.hex
