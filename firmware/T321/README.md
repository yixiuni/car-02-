# T321 当前试验固件

本车已烧录且完整回读一致，**待实车验证**。历史 T319/T320 失败记录仍保留，不能把此镜像视为已稳定达成四圈 30 秒。

- `stm32-car.bin` 烧录地址 `0x08000000`；HEX 自带地址。
- `manifest.json` 记录版本、SHA256、构建入口与标定适用范围。
- [修改说明与验证](../../docs/T321沿弧位置约束与验证.md)。
- 构建：`python3 tools/build_firmware.py --time-trial-arc-phase --toolchain /path/to/arm-toolchain/bin`。
- 测试：`python3 tests/run_tests.py --trial-profile time-trial-arc-phase --sanitize`。

NORMAL/CROSS、1/4 圈操作不变，启动由物理 USER 控制。运行后保持供电，再用 `tools/read_t321.tcl` 回收记录。
