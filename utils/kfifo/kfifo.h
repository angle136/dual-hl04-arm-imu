#ifndef _KFIFO_H
#define _KFIFO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct kfifo
{
    volatile unsigned int in;    // 写索引（持续递增）
    volatile unsigned int out;   // 读索引（持续递增）
    unsigned int mask;  // 容量 - 1，用于快速 & 运算
    unsigned int esize; // 元素大小（字节）
    char        *data;  // 用户提供的缓冲区指针
};

/**
 * @brief 使用外部缓冲区初始化 kfifo。
 * @param fifo      指向 kfifo 实例的指针
 * @param buffer    用户预分配的缓冲区（需自行保证生命期）
 * @param size      期望的元素容量（会向下取整为 2 的幂）
 * @param esize     每个元素的字节数
 * @return 成功返回 0，若 size < 2 返回 -1
 */
int kfifo_init(struct kfifo *fifo, void *buffer, unsigned int size, unsigned int esize);

/**
 * @brief 获取已占用元素数。
 * @param fifo 指向 kfifo 实例的指针
 * @return 已占用元素数
 */
unsigned int kfifo_len(const struct kfifo *fifo);
/**
 * @brief 剩余可用元素数。
 * @param fifo 指向 kfifo 实例的指针
 * @return 剩余可用元素数
 */
unsigned int kfifo_avail(const struct kfifo *fifo);
/**
 * @brief 是否为空。
 * @param fifo 指向 kfifo 实例的指针
 * @return 是否为空
 */
int kfifo_is_empty(const struct kfifo *fifo);
/**
 * @brief 是否已满。
 * @param fifo 指向 kfifo 实例的指针
 * @return 是否已满
 */
int kfifo_is_full(const struct kfifo *fifo);
/**
 * @brief 总容量（元素数）。
 * @param fifo 指向 kfifo 实例的指针
 * @return 总容量（元素数）
 */
unsigned int kfifo_size(const struct kfifo *fifo);
/**
 * @brief 清空所有数据。
 * @param fifo 指向 kfifo 实例的指针
 */
void kfifo_reset(struct kfifo *fifo);
/**
 * @brief 仅丢弃已读数据。
 * @param fifo 指向 kfifo 实例的指针
 */
void kfifo_reset_out(struct kfifo *fifo);

/**
 * @brief 单元素操作。
 * @param fifo 指向 kfifo 实例的指针
 * @param element 指向要操作的元素的指针
 * @return 操作成功返回 1，失败返回 0
 */
unsigned int kfifo_put(struct kfifo *fifo, const void *element);
/**
 * @brief 读取一个元素。
 * @param fifo 指向 kfifo 实例的指针
 * @param element 指向要读取的元素的指针
 * @return 操作成功返回 1，失败返回 0
 */
unsigned int kfifo_get(struct kfifo *fifo, void *element);
/**
 * @brief 预读但不移除。
 * @param fifo 指向 kfifo 实例的指针
 * @param element 指向要预读的元素的指针
 * @return 操作成功返回 1，失败返回 0
 */
unsigned int kfifo_peek(const struct kfifo *fifo, void *element);

/**
 * @brief 批量写入。
 * @param fifo 指向 kfifo 实例的指针
 * @param buf 指向要操作的缓冲区的指针
 * @param n 要操作的元素数
 * @return 操作成功返回 1，失败返回 0
 */
unsigned int kfifo_in(struct kfifo *fifo, const void *buf, unsigned int n);
/**
 * @brief 批量读取。
 * @param fifo 指向 kfifo 实例的指针
 * @param buf 指向要操作的缓冲区的指针
 * @param n 要操作的元素数
 * @return 操作成功返回 1，失败返回 0
 */
unsigned int kfifo_out(struct kfifo *fifo, void *buf, unsigned int n);
/**
 * @brief 批量读取（预读但不移除）。
 * @param fifo 指向 kfifo 实例的指针
 * @param buf 指向要操作的缓冲区的指针
 * @param n 要操作的元素数
 * @return 操作成功返回 1，失败返回 0
 */
unsigned int kfifo_out_peek(const struct kfifo *fifo, void *buf, unsigned int n);

#ifdef __cplusplus
}
#endif

#endif /* _KFIFO_H */
