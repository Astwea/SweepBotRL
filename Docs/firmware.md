# Firmware Build Guide

This document outlines how to compile and flash the STM32 firmware contained in the `firmware/` directory.

## Building with STM32CubeIDE

1. Install **STM32CubeIDE** from the STMicroelectronics website.
2. Launch the IDE and open the project by selecting the file `firmware/pwm_test.ioc`.
3. STM32CubeIDE will generate the IDE project automatically.
4. Connect the controller board via ST‑Link or another programmer.
5. Click **Build** followed by **Run** to compile and flash the firmware.

## Building with MDK-ARM (Keil)

1. Open `firmware/MDK-ARM/pwm_test.uvprojx` in **Keil uVision**.
2. Choose the desired build target (e.g. `STM32F103`).
3. Use the **Build** option to compile the project.
4. Flash the board using your preferred debug interface.

Both environments generate the same firmware binary. Choose the toolchain that best fits your workflow.
