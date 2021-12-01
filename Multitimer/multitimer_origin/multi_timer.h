/*
 * Copyright (c) 2016 Zibin Zheng <znbin@qq.com>
 * All rights reserved
 */
 
#ifndef _MULTI_TIMER_H_
#define _MULTI_TIMER_H_
 
#include "stdint.h"
#include "stddef.h"
 
typedef struct Timer {
    uint32_t timeout;						// 超时时间（用来与定时器心跳比较）
    uint32_t repeat;						// 循环定时触发时间（周期定时设置），为0时代表单次定时
    void (*timeout_cb)(void);		// 定时器回调处理函数
    struct Timer* next;					// 指向下一个定时器节点
}Timer;
 
#ifdef __cplusplus  
extern "C" {  
#endif  
 
void timer_initial(struct Timer* handle, void(*timeout_cb)(), uint32_t timeout, uint32_t repeat);
int  timer_start(struct Timer* handle);
void timer_stop(struct Timer* handle);
void timer_ticks(void);
void timer_loop(void);
 
// void timer_again(struct Timer* handle);
// void timer_set_repeat(struct Timer* handle, uint32_t repeat);
 
#ifdef __cplusplus
} 
#endif

#endif