# LDC vNext 2.0.2（封版审计修订版）

本目录是 LDC 的重新设计版本，不复用旧版 liqueue、环形缓冲区或全局 registry 的实现思路。应用真正移植时只需复制 `ldc.h` 和 `ldc.c`；`tests`、ART-Pi 示例与测试脚本仅用于验证。

## 适用边界

LDC 用于把 UART 字节流整理成完整帧，适合 AT/URC、屏幕协议及小型私有协议。它不负责：

- Modbus RTU 的 T1.5/T3.5 判定；
- UART、DMA、IDLE 中断或硬件定时器初始化；
- RTOS 调度和业务消息队列；
- 动态内存分配。

时间边界由 UART 或定时器所有者判断，再显式调用 `ldc_rx_idle()`。LDC 不保存全局时间戳，也没有全局 `tick_all`。

## 并发、事务与所有权

- 一个 LDC 对象只允许一个生产者和一个消费者；
- ISR 与任务并发时，由应用为该对象提供成对的 `lock/unlock`；
- LDC 不绑定 PRIMASK、HAL、CMSIS 或 RTOS；裸机单上下文可不配置锁；
- 所有帧存储和槽元数据都由调用方静态提供，不使用堆；
- `ldc_rx_write()` 对整个输入块执行事务：全部接收或一个字节也不接收；
- 完整帧只在整个输入块复制完成后一次性发布；
- `ldc_frame_read()` 在锁内 claim，在锁外复制，最后短暂加锁 release；
- 零拷贝 `ldc_frame_claim()/ldc_frame_release()` 会固定槽位，固定期间不能被生产者覆盖；
- 每次发布使用独立 64 位 `claim_token` 校验 view 身份；token 种子在正常 `deinit/init` 间连续保留，32 位 `sequence` 仅用于有界队列排序，不再承担生命周期安全；
- `DROP_OLDEST`、超长帧、拒绝和溢出都通过返回结果及统计显式暴露。

LDC 对以下三块内存拥有独占权，三者不能互相重叠：

1. `ldc_t` 上下文；
2. `ldc_slot_t[]` 槽元数据；
3. `storage` 帧载荷区。

`ldc_rx_write()` 的输入以及 `ldc_frame_read()` 的 `destination`、`length` 输出也不能指向以上区域；两个输出对象之间也不能重叠。2.0.2 会在容量判断和占用帧前主动检查，非法别名返回 `LDC_STATUS_INVALID_ARGUMENT`，保持输出值和待读帧不变。首次初始化前将 `ldc_t` 置零；正常 `deinit()` 后不要再次手动清零上下文，可直接 `init()`，以保留跨生命周期 stale-view 防护。

## 最小示例

```c
static ldc_t uart_rx = LDC_CONTEXT_INITIALIZER;
static ldc_slot_t uart_slots[4];
static uint8_t uart_storage[4 * 128];
static const uint8_t crlf[] = {'\r', '\n'};

ldc_config_t config = {0};
config.storage = uart_storage;
config.storage_size = sizeof(uart_storage);
config.slots = uart_slots;
config.slot_count = 4;
config.frame_capacity = 128;
config.frame_mode = LDC_FRAME_MODE_DELIMITER;
config.full_policy = LDC_FULL_REJECT_NEW;
config.delimiter = crlf;
config.delimiter_length = sizeof(crlf);

(void)ldc_init(&uart_rx, &config);
```

UART 接收入口只投递数据：

```c
ldc_write_result_t result = ldc_rx_write(&uart_rx, data, length);
if (result.accepted_bytes != length) {
    /* 本输入块未被完整接收，记录 UART/应用层数据损失。 */
}
```

任务或主循环读取完整帧：

```c
size_t length;
uint8_t frame[128];

while (ldc_frame_read(&uart_rx, frame, sizeof(frame), &length) == LDC_STATUS_OK) {
    protocol_process(frame, length);
}
```

迁移期若需与旧 LDC 同时链接，可在整个目标中定义：

```text
LDC_SYMBOL_PREFIX=ldc_vnext_
```

单独使用时无需定义，API 仍为简洁的 `ldc_*`。

## 验证记录

- MinGW GCC C99，`-Wall -Wextra -Werror -pedantic`：默认符号及前缀符号两套回归通过；
- 回归覆盖分隔符跨块、定长帧、手动/空闲边界、事务拒绝、DROP_OLDEST、claim 固定、超长帧、KMP 重叠、生命周期、输出别名拒绝及 32 位同步回绕下的 stale-view 防护；
- Keil Arm Compiler 6 / ART-Pi STM32H750 ThreadX 隔离验收目标：0 error；
- ST-Link 下载至 `0x08000000`：Erase、Programming、Verify 均成功；
- UART4 / COM19 / 115200：跨块、连包、超长帧丢弃、恢复帧和统计均通过；
- v2.0.1 独立封版审计报告的 2 条 Medium 已在 2.0.2 修复：输出参数别名不再消费帧，stale view 不再依赖可同步回绕的 32 位身份组合。
