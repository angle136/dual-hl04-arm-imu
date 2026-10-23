# modules/LORA

塔石 L33 LoRa 透传模块。默认串口参数为 115200 8N1；空中速率、信道和透传模式需提前用模块上位机配置一致。

应用层帧格式：

```text
0xAA 0x55 LEN SEQ PAYLOAD CRC8
```

`PAYLOAD` 由注册变量按注册顺序拼接，数值按小端序编码。CRC8 多项式为 `0x07`，覆盖 `LEN + SEQ + PAYLOAD`。收发两端必须使用完全相同的注册顺序和类型；Payload 最大 59 字节，最多注册 16 项。

模块由框架初始化并创建内部处理线程。APP 在初始化阶段依次调用 `Lora_Register()` 注册持久化变量，最后调用一次 `Lora_Start()`；启动后不能继续注册。模块保存变量指针，每次发送时读取最新值。

启用前需在目标 `robot.cmake` 中将 `LORA` 加入模块列表，并配置：

```cmake
set(LORA_UART huart6)
set(LORA_M0_GPIO_PORT GPIOE)
set(LORA_M0_GPIO_PIN GPIO_PIN_13)
set(LORA_M1_GPIO_PORT GPIOE)
set(LORA_M1_GPIO_PIN GPIO_PIN_9)
set(LORA_AUX_ENABLE 0)
set(LORA_TX_ENABLE 1)
set(LORA_RX_ENABLE 0)
set(LORA_TX_INTERVAL_MS 50)
```

不要让 LoRa 与其它模块共用同一个 UART。DMA 发送缓冲区必须位于 DMA 可访问内存。
