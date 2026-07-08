# CMS32FOC 工具链使用说明

本文面向一台全新的 Windows 电脑：没有 WSL、没有 Ubuntu、也不知道如何使用本工程的
GCC/CMake 编译环境。

## 交付文件

建议把下面这些文件一起交给使用者：

```text
CMS32FOC_GCC/                                      # 本工程源码
cms32foc-toolchain-ubuntu24.04-wsl-rootfs.tar      # WSL Ubuntu 工具链 rootfs
cms32foc-toolchain-ubuntu24.04.tar                 # Docker 工具链镜像，可选
```

`wsl-rootfs.tar` 和 Docker 镜像都只包含干净 Ubuntu 24.04 工具链环境，不包含原电脑的
home、Codex、Claude、SSH、Git 个人配置。

## 推荐方式：导入 WSL Ubuntu

### 1. 安装 WSL

在 Windows PowerShell 里执行：

```powershell
wsl --status
```

如果提示没有安装 WSL，用管理员 PowerShell 执行：

```powershell
wsl --install --no-distribution
```

如果当前 Windows 不支持 `--no-distribution`，改用：

```powershell
wsl --install
```

安装完成后重启电脑。

### 2. 导入 CMS32FOC 工具链 Ubuntu

假设 `cms32foc-toolchain-ubuntu24.04-wsl-rootfs.tar` 放在
`D:\CMS32FOC\`：

```powershell
mkdir C:\WSL
wsl --import CMS32FOC C:\WSL\CMS32FOC D:\CMS32FOC\cms32foc-toolchain-ubuntu24.04-wsl-rootfs.tar
```

进入这个 Ubuntu：

```powershell
wsl -d CMS32FOC
```

查看 WSL 列表：

```powershell
wsl -l -v
```

应该能看到类似：

```text
  NAME       STATE    VERSION
* Ubuntu     Running  2
  CMS32FOC   Stopped  2
```

### 3. 放置工程源码

推荐把工程源码放在 WSL 文件系统里，不要直接放在 Windows 的 `C:\` 或 `D:\` 下编译。

进入 WSL 后执行：

```sh
mkdir -p ~/workspace
```

如果源码现在在 Windows 的 `D:\CMS32FOC\CMS32FOC_GCC`，可以在 WSL 里复制：

```sh
cp -a /mnt/d/CMS32FOC/CMS32FOC_GCC ~/workspace/
cd ~/workspace/CMS32FOC_GCC
```

如果源码已经通过 Git 拉取，也可以直接进入工程目录：

```sh
cd ~/workspace/CMS32FOC_GCC
```

## 确认工具链版本

在 WSL 的工程目录或任意目录执行：

```sh
cat /etc/os-release | sed -n '1,4p'
arm-none-eabi-gcc --version | sed -n '1p'
cmake --version | sed -n '1p'
ninja --version
openocd --version 2>&1 | sed -n '1p'
gdb-multiarch --version | sed -n '1p'
```

当前工具链镜像验证过的版本：

```text
Ubuntu 24.04
arm-none-eabi-gcc 13.2.1
cmake 3.28.3
ninja 1.11.1
openocd 0.12.0
gdb-multiarch 15.1
```

## 编译 CMS32 固件

进入工程目录：

```sh
cd ~/workspace/CMS32FOC_GCC
```

配置 CMake：

```sh
cmake --preset gcc-debug
```

编译主固件 `cms32foc`：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

生成文件在：

```text
build/gcc-debug/cms32foc
build/gcc-debug/cms32foc.hex
build/gcc-debug/cms32foc.bin
build/gcc-debug/cms32foc.map
```

清理：

```sh
cmake --build --preset gcc-debug --target clean
```

重新编译：

```sh
cmake --build --preset gcc-debug --target clean
cmake --build --preset gcc-debug --target cms32foc
```

诊断固件命令：

```sh
cmake --build --preset gcc-debug --target cms32_startup_smoke_test
cmake --build --preset gcc-debug --target cms32_board_watch_test
```

如果当前开发分支里诊断固件因为未完成代码而链接失败，先确认主固件：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

## 连接 VSCode

### 1. 安装 VSCode 插件

在 Windows VSCode 里安装：

```text
WSL
CMake Tools
clangd
Hex Editor
Material Icon Theme
```

本工程 `.vscode/extensions.json` 里也会推荐部分插件。

### 2. 打开 WSL 工程

方式一：从 Windows VSCode 打开。

1. 按 `Ctrl+Shift+P`
2. 输入并选择 `WSL: Connect to WSL using Distro...`
3. 选择 `CMS32FOC`
4. 在远程 VSCode 窗口里选择 `File -> Open Folder...`
5. 打开 `/root/workspace/CMS32FOC_GCC` 或实际工程目录

方式二：从 WSL 终端打开。

```sh
cd ~/workspace/CMS32FOC_GCC
code .
```

如果提示没有 `code` 命令，先用方式一连接一次 WSL。

### 3. VSCode 编译

打开工程后：

```text
Terminal -> Run Task... -> Build
```

或者直接按：

```text
Ctrl+Shift+B
```

当前 `.vscode/tasks.json` 里的默认 `Build` 任务会执行：

```sh
cmake --preset gcc-debug
cmake --build --preset gcc-debug --target cms32foc
```

## clangd 索引

第一次打开工程时，先编译一次，让 CMake 生成：

```text
build/gcc-debug/compile_commands.json
```

然后 clangd 才能准确识别头文件、宏定义和交叉编译参数。

如果 VSCode 里 C/C++ 报很多不存在的头文件错误，先执行：

```sh
cmake --preset gcc-debug
cmake --build --preset gcc-debug --target cms32foc
```

然后重启 clangd：

```text
Ctrl+Shift+P -> clangd: Restart language server
```

## 可选方式：使用 Docker 镜像

如果对方不想导入 WSL，也可以只用 Docker。

在装好 Docker Desktop 后，进入放置镜像 tar 的目录：

```powershell
docker load -i cms32foc-toolchain-ubuntu24.04.tar
docker images
```

运行工具链容器并挂载当前工程：

```powershell
cd D:\CMS32FOC\CMS32FOC_GCC
docker run --rm -it -v "${PWD}:/workspace" -w /workspace cms32foc-toolchain:ubuntu24.04
```

进入容器后编译：

```sh
cmake -S . -B build/docker-gcc-debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build/docker-gcc-debug --target cms32foc
```

如果是在 Linux/WSL shell 里运行 Docker，也可以直接用仓库脚本：

```sh
./scripts/docker-build.sh
```

## 常见问题

### `wsl` 命令不存在

Windows 版本过旧或 WSL 没安装。先升级 Windows，再用管理员 PowerShell 执行：

```powershell
wsl --install
```

### VSCode 打开的是 Windows 目录，不是 WSL 目录

左下角应该显示类似：

```text
WSL: CMS32FOC
```

如果没有，说明 VSCode 还在 Windows 本地窗口，需要重新执行：

```text
Ctrl+Shift+P -> WSL: Connect to WSL using Distro...
```

### 编译提示找不到 `arm-none-eabi-gcc`

说明没有进入导入好的 `CMS32FOC` WSL，或者 VSCode 没连接到 WSL。

检查：

```sh
which arm-none-eabi-gcc
arm-none-eabi-gcc --version | sed -n '1p'
```

正常应该看到：

```text
/usr/bin/arm-none-eabi-gcc
arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009
```

### Windows 路径和 WSL 路径怎么互相转换

Windows 路径：

```text
D:\CMS32FOC\CMS32FOC_GCC
```

在 WSL 里对应：

```text
/mnt/d/CMS32FOC/CMS32FOC_GCC
```

WSL 工程目录：

```text
/root/workspace/CMS32FOC_GCC
```

在 Windows 资源管理器里可以访问：

```text
\\wsl.localhost\CMS32FOC\root\workspace\CMS32FOC_GCC
```

### 重新导入 WSL

如果导入错了，可以先删除旧发行版：

```powershell
wsl --unregister CMS32FOC
```

然后重新导入：

```powershell
wsl --import CMS32FOC C:\WSL\CMS32FOC D:\CMS32FOC\cms32foc-toolchain-ubuntu24.04-wsl-rootfs.tar
```

`--unregister` 会删除这个 WSL 发行版里的所有文件，执行前确认工程源码已经备份。
