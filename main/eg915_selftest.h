#ifndef EG915_SELFTEST_H
#define EG915_SELFTEST_H

#include <stdbool.h>

// 启动EG915 AT握手自测任务（FreeRTOS任务）
void start_eg915_at_selftest(void);

// 单次AT握手（发送"AT"并等待"OK"），返回是否成功
bool eg915_at_handshake_once(unsigned timeout_ms, int retries);

#endif // EG915_SELFTEST_H
