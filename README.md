## Toolchain

- Ubuntu 22.04.2 LTS
- Vitis v2024.1 (no SDT flow)
- custom drone hardware rev.5

## Build code

1. The repository contains git submodules (FreeRTOS-Kernel). To populate them use:

```
git submodule init
git submodule update
```

2. Generate bsp

```
cd src
make bsp
```
and manually set definition `FF_USE_FIND` to `1` in `xilffs_v5_2/src/include/ffconf.h`.

3. Build application
```
cd src
make clean all
```
