/*
 * Copyright (c) 2016 Zibin Zheng <znbin@qq.com>
 * All rights reserved
 */

#include "multi_timer.h"
#include <stdio.h>
//timer handle list head.
static struct Timer* head_handle = NULL;

//Timer ticks
static uint32_t _timer_ticks = 4294967290;    //32位最大值而已，再加一就溢出归0即可

/**
  * @brief  Initializes the timer struct handle.
  * @param  handle: the timer handle strcut.
  * @param  timeout_cb: timeout callback.
  * @param  repeat: repeat interval time.
  * @retval None
  */
void timer_initial(struct Timer* handle, void (*timeout_cb)(), uint32_t timeout, uint32_t repeat)
{
    // memset(handle, sizeof(struct Timer), 0);
    handle->timeout_cb = timeout_cb;
    handle->timeout    = _timer_ticks + timeout;
    handle->repeat     = repeat;
}

/**
  * @brief  Start the timer work, add the handle into work list.
  * @param  btn: target handle strcut.
  * @retval 0: succeed. -1: already exist.
  */
int timer_start(struct Timer* handle)
{
    struct Timer* target = head_handle;			// 设置一个临时变量就不会改变

		// 遍历查找判断该节点是否已存在
    while(target)
    {
        if(target == handle)
            return -1;  //already exist.
        target = target->next;							// 不断遍历下一个节点
    }
		// 采用链表前插的方式，最新的定时器放在前面并作为头结点
    handle->next = head_handle;
    head_handle  = handle;
    return 0;
}

/**
  * @brief  Stop the timer work, remove the handle off work list.
  * @param  handle: target handle strcut.
  * @retval None
  */
void timer_stop(struct Timer* handle)
{
    struct Timer** curr;
    for(curr = &head_handle; *curr;)
    {
        struct Timer* entry = *curr;
        if(entry == handle)
        {
            *curr = entry->next;						// 将当前节点脱离队列
            //			free(entry);
        }
        else
            curr = &entry->next;						// 二级指针curr不断后移
    }
}

/**
  * @brief  main loop.
  * @param  None.
  * @retval None
  */
void timer_loop()														// 在While循环中使用，定时器才会起作用
{
    struct Timer* target;
    for(target = head_handle; target; target = target->next)
    {
#if 0
        if(_timer_ticks >= target->timeout)
#else
      if((int)((uint32_t)(target->timeout -_timer_ticks)) <= 0)
			//_timer_ticks 这个变量会出现溢出的情况，因为它是32位的，所以不能大于0xFFFFFFFF = 4,294,967,295,
			//一天有多少s, 24*60*60*1000 = 86,400,000s,也就是改变了一直累加49.7天就会溢出，4,294,967,295 / 86,400,000 = 49.71026961805556天
			
#endif
        {
            if(target->repeat == 0)
            {
                timer_stop(target);
            }
            else
            {
                target->timeout = _timer_ticks + target->repeat;
            }
            target->timeout_cb();
        }
    }
}

/**
  * @brief  background ticks, timer repeat invoking interval 1ms.
  * @param  None.
  * @retval None.
  */
void timer_ticks()			// 在定时器中断中调用该函数
{
    _timer_ticks++;    //但是溢出风险
}
