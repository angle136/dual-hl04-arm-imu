# Generic FIFO Queue (kfifo)

通用元素级 FIFO 队列，基于外部缓冲区实现，适用于**单生产者单消费者 (SPSC)** ，容量自动向下取整为 2 的幂。

## 数据结构

### `struct kfifo` — FIFO 实例

| 字段 | 类型 | 说明 |
|------|------|------|
| `in` | `unsigned int` | 写索引（持续递增，永不回退） |
| `out` | `unsigned int` | 读索引（持续递增，永不回退） |
| `mask` | `unsigned int` | 容量 - 1，`in & mask` 即当前写入偏移 |
| `esize` | `unsigned int` | 每个元素的字节数（1 = 字节模式） |
| `data` | `char *` | 指向用户预分配缓冲区的指针 |

> **缓冲区由调用者提供**，kfifo 不负责内存分配。初始化后 `data` 指向的缓冲区生命周期必须覆盖整个 FIFO 使用期。

### 缓冲区布局

```
         mask = 3 (容量 = 4)
         ┌─────┬─────┬─────┬─────┐
data →   │ [0] │ [1] │ [2] │ [3] │
         └─────┴─────┴─────┴─────┘
         out=2           in=1
         └──已读──┘      └未写┘
                 └─有效数据─┘
         len = in - out = (1 - 2) mod 2³² = 3
```

```
in / out 持续递增溢出 32-bit 后自动回绕（unsigned 语义），
配合 mask 位与运算定位物理槽位，无需条件分支处理边界。
```

## API 说明

```c
int          kfifo_init(struct kfifo *fifo, void *buffer, unsigned int size, unsigned int esize);
unsigned int kfifo_size(const struct kfifo *fifo);
unsigned int kfifo_len(const struct kfifo *fifo);
unsigned int kfifo_avail(const struct kfifo *fifo);
int          kfifo_is_empty(const struct kfifo *fifo);
int          kfifo_is_full(const struct kfifo *fifo);
void         kfifo_reset(struct kfifo *fifo);
void         kfifo_reset_out(struct kfifo *fifo);
// 单元素操作
unsigned int kfifo_put(struct kfifo *fifo, const void *element);
unsigned int kfifo_get(struct kfifo *fifo, void *element);
unsigned int kfifo_peek(const struct kfifo *fifo, void *element);
// 批量操作
unsigned int kfifo_in(struct kfifo *fifo, const void *buf, unsigned int n);
unsigned int kfifo_out(struct kfifo *fifo, void *buf, unsigned int n);
unsigned int kfifo_out_peek(const struct kfifo *fifo, void *buf, unsigned int n);
```
> 返回值：单元素操作返回 `1`（成功）或 `0`（满/空）；批量操作返回**实际操作的元素数**，可能小于请求的 `n`。

## 使用示例

### 字节 FIFO（esize = 1）

```c
#include "kfifo.h"

#define BUF_SIZE 64  // 2 的幂，否则自动向下取整

void byte_fifo_demo(void)
{
    uint8_t      buf[BUF_SIZE];
    struct kfifo fifo;

    kfifo_init(&fifo, buf, BUF_SIZE, 1);  // esize=1 字节模式

    // 批量写入
    const char *msg = "Hello kfifo!";
    unsigned int n  = kfifo_in(&fifo, msg, 12);
    // n = 12, len = 12

    // 批量读出
    char read_buf[64];
    n = kfifo_out(&fifo, read_buf, 12);
    // read_buf = "Hello kfifo!", len = 0
}
```

### 结构体 FIFO（esize > 1）

```c
typedef struct
{
    int   id;
    float value;
} sensor_data_t;

void struct_fifo_demo(void)
{
    sensor_data_t buf[8];
    struct kfifo  fifo;

    kfifo_init(&fifo, buf, 8, sizeof(sensor_data_t));

    // 单元素写入
    sensor_data_t s1 = {.id = 1, .value = 3.14f};
    sensor_data_t s2 = {.id = 2, .value = 2.71f};
    kfifo_put(&fifo, &s1);  // len = 1
    kfifo_put(&fifo, &s2);  // len = 2

    // peek 不移除
    sensor_data_t peek;
    kfifo_peek(&fifo, &peek);
    // peek = {1, 3.14}, len = 2 (unchanged)

    // 批量读取
    sensor_data_t batch[8];
    unsigned int n = kfifo_out(&fifo, batch, 8);
    // n = 2, batch[0] = s1, batch[1] = s2
}
```
> **注意**：`kfifo_in` / `kfifo_out` 不是原子操作，SPSC 模式下安全的前提是**生产者和消费者各只有一方**，且不使用 `kfifo_reset` / `kfifo_reset_out` 等可能破坏索引一致性的操作。

## 内部实现

### 回绕拷贝

批量 `in` / `out` 时，数据可能跨越缓冲区末尾。内部通过两次 `memcpy` 自动处理：

```
写入 n=3 个元素到 off=3 (mask=3, size=4):
                    off=3
                    ↓
    ┌─────┬─────┬─────┬─────┐
    │ [0] │ [1] │ [2] │ [3] │
    └─────┴─────┴─────┴─────┘
    └ 后半段 (size-off=1) ┘└ 回绕到开头 (n-l=2) ┘
              ↑  memcpy #1   ↑  memcpy #2
```

```
fifo_copy_in / fifo_copy_out:
┌──────────────────────────────────┐
│ 1. off = off & mask              │
│ 2. l = min(len, size - off)      │
│ 3. memcpy(data + off, src, l)    │  ← 尾部段
│ 4. memcpy(data, src + l, len-l)  │  ← 回绕段
└──────────────────────────────────┘
```

### 容量取整

`kfifo_init` 自动将 `size` 向下取整为 2 的幂：

```c
// 非 2 的幂检测 & 取整
if ((size & (size - 1)) != 0)
    size = rounddown_pow_of_two(size);  // 如 100 → 64
```

`mask = size - 1`，之后所有索引计算只需 `idx & mask` 一条位与指令，无需取模运算。

### in/out 溢出语义

```c
// in/out 为 unsigned int，持续递增
// 32-bit 溢出后自动归零（C 标准保证）
kfifo_len  = in - out;        // 始终正确（unsigned 回绕）
kfifo_avail = mask + 1 - len; // 始终正确
```

## 注意事项

1. **缓冲区生命周期** — kfifo 不分配内存，`buffer` 必须在整个 FIFO 使用期间保持有效。

2. **容量为 2 的幂** — 非 2 的幂会自动取整，可能导致实际容量小于预期。建议直接使用 2 的幂值避免"缩水"。
   ```c
   // 推荐写法：
   kfifo_init(&fifo, buf, 64, sizeof(item_t));   // 64 是 2 的幂
   ```

3. **非线程安全** — 所有操作均不包含锁或原子指令。多任务访问必须外部同步。

4. **SPSC 可用规则** — 在唯一生产者 + 唯一消费者模式下可无锁使用，但必须遵守：
   - 生产者只调用 `kfifo_put` / `kfifo_in`
   - 消费者只调用 `kfifo_get` / `kfifo_out` / `kfifo_peek` / `kfifo_out_peek`
   - 双方均不调用 `kfifo_reset` / `kfifo_reset_out`（这些操作同时修改 `in` 和 `out`）
   - 状态查询函数 (`len`, `avail`, `is_empty`, `is_full`) 在并发下仅提供近似值

5. **`kfifo_peek` 不修改索引** — 可以安全地在消费逻辑中预读判断，不影响后续 `get`。

6. **`kfifo_reset` vs `kfifo_reset_out`** — `reset` 清空全部数据，`reset_out` 仅丢弃已消费部分（将 `out` 同步到 `in`，等价于标记已读数据可被覆盖）。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-04 | 初始版本 |
