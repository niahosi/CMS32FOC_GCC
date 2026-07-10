# Frozen Diagnostics

这里存放已经完成阶段性任务、默认不再参与 `cms32foc` 构建的辅助程序。

这些代码用于保留调试思路和寄存器操作序列，例如自动 Align、EncoderVoltage、MA600 在线 BCT/CORR/NVM 调参。它们不保证随主头文件持续可编译；需要恢复时，再同步接口并显式加入 CMake target。
