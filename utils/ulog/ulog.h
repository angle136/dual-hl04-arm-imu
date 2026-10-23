#ifndef _ULOG_H_
#define _ULOG_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int      ulog_init(void);
void     ulog_deinit(void);
void     ulog_output(uint32_t level, const char *tag, bool newline, const char *format, ...);
void     ulog_voutput(uint32_t level, const char *tag, bool newline, const char *format, va_list args);
void     ulog_raw(const char *format, ...);
void     ulog_hexdump(const char *tag, uint32_t width, const uint8_t *buf, uint32_t size);
void     ulog_flush(void);
void     ulog_set_level(uint32_t level);
uint32_t ulog_get_level(void);

#ifdef __cplusplus
}
#endif

#endif /* _ULOG_H_ */
