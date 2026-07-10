# CMS32FOC_GCC

GCC/CMake bring-up project for the CMS32M65xx FOC board.

The repository follows an ODrive-style layout: firmware code is grouped under
`Firmware/`, while development scripts and debugger files live under `Tools/`.

This project starts from the hardware pin configuration that was verified in
the Keil project at:

```text
D:\wjw\1_Motor\CMS32M65xx\CMS32FOCAC6
```

Build:

```sh
cmake --preset gcc-debug
cmake --build --preset gcc-debug
```

Generated artifacts are written to `build/gcc-debug/`.

Docker toolchain build:

```sh
docker build -t cms32foc-toolchain:ubuntu24.04 .
./scripts/docker-build.sh
```

Export the toolchain image for another machine:

```sh
./scripts/docker-export.sh
```

The exported Docker image can be loaded with:

```sh
docker load -i cms32foc-toolchain-ubuntu24.04.tar
```

The optional WSL rootfs tar can be imported from Windows PowerShell:

```powershell
wsl --import CMS32FOC C:\WSL\CMS32FOC .\cms32foc-toolchain-ubuntu24.04-wsl-rootfs.tar
```

Useful docs:

- `Docs/Architecture/ProjectStructure.md`
- `Docs/Architecture/ActiveControlChain.md`
- `Docs/Architecture/ReadingGuide.md`
- `Docs/Architecture/StartupAndLinker.md`
- `Docs/VSCode-Tasks.md`
- `Docs/Toolchain-Setup-zh.md`
- `Docs/Hardware/Pins.md`

Reference material copied from the original Keil workspace and hardware
packages lives under `Reference/`. It is for reading and staged migration only;
it is not part of the CMake build.
