#include "uv.h"
#include "queue.h"
#include <AudioCApi.h>
#include <string.h>

#ifdef STM32F723xx
#include <stm32f7xx_hal.h>
#elif defined(STM32N657xx)
#include <stm32n6xx_hal.h>
#endif

enum {
	UV__HANDLE_INTERNAL = 0x8000,
	UV__HANDLE_ACTIVE = 0x4000,
	UV__HANDLE_REF = 0x2000,
	UV__HANDLE_CLOSING = 0 /* no-op on unix */
};

static uv_loop_t default_loop;
static int default_loop_initialized = 0;

uv_loop_t* uv_default_loop(void) {
	if(!default_loop_initialized) {
		default_loop_initialized = 1;
		uv_loop_init(&default_loop);
	}

	return &default_loop;
}

int uv_loop_init(uv_loop_t* loop) {
	memset(loop, 0, sizeof(*loop));
	QUEUE_INIT(&loop->timer_queue);
	QUEUE_INIT(&loop->idle_queue);
	QUEUE_INIT(&loop->async_queue);

	return 0;
}

int uv_loop_close(uv_loop_t* loop) {
	(void) loop;

	return 0;
}

#define timer_before_or_eq(a, b) ((int32_t) ((uint32_t) (a) - (uint32_t) (b)) <= 0)

static int32_t uv__run_timers(uv_loop_t* loop) {
	QUEUE* q;
	uint32_t current_time = HAL_GetTick();
	int32_t next_sleep_duration = INT32_MAX;

	for(q = QUEUE_NEXT(&loop->timer_queue); q != &loop->timer_queue;) {
		// Go to next before processing the current timer as we might remove it from the list
		uv_timer_t* current_timer = QUEUE_DATA(q, uv_timer_t, timer_queue);
		q = QUEUE_NEXT(q);

		if(timer_before_or_eq(current_timer->next_wakeup, current_time)) {
			// Stop before as the callback might rearm the timer
			if(current_timer->repeat) {
				uv_timer_again(current_timer);
			} else {
				uv_timer_stop(current_timer);
			}
			DAMC_beginMeasure(TMI_MainLoop);
			current_timer->callback(current_timer);
			DAMC_endMeasure(TMI_MainLoop);
		}

		if(current_timer->flags & UV__HANDLE_ACTIVE) {
			int32_t timer_remaining = current_timer->next_wakeup - current_time;
			if(timer_remaining < next_sleep_duration) {
				next_sleep_duration = timer_remaining;
			}
		}
	}

	// return next earliest wakeup time
	return next_sleep_duration + current_time;
}

static void uv__run_idle(uv_loop_t* loop) {
	QUEUE* q;
	QUEUE_FOREACH(q, &loop->idle_queue) {
		uv_idle_t* handle = QUEUE_DATA(q, uv_idle_t, idle_queue);
		DAMC_beginMeasure(TMI_MainLoop);
		handle->callback(handle);
		DAMC_endMeasure(TMI_MainLoop);
	}
}

static void uv__run_async(uv_loop_t* loop) {
	if(!loop->async_pending) {
		return;
	}

	loop->async_pending = 0;
	__DMB();

	QUEUE* q;
	QUEUE_FOREACH(q, &loop->async_queue) {
		uv_async_t* handle = QUEUE_DATA(q, uv_async_t, async_queue);
		if(handle->pending) {
			handle->pending = 0;
			__DMB();
			DAMC_beginMeasure(TMI_MainLoop);
			handle->callback(handle);
			DAMC_endMeasure(TMI_MainLoop);
		}
	}
}

volatile uint32_t current_time;
volatile uint32_t next_wakeup_time;
int uv_run(uv_loop_t* loop, uv_run_mode mode) {
	while(1) {
		next_wakeup_time = uv__run_timers(loop);
		uv__run_async(loop);

		// Sleep until the next timer needs to be triggered
		// while(timer_before_or_eq((current_time = HAL_GetTick()), next_wakeup_time)) {
		uv__run_idle(loop);
		// SysTick interrupt will wakeup us every time the tick is incremented
		//__WFI();
		//}
	}

	return 0;
}

int uv_timer_init(uv_loop_t* loop, uv_timer_t* handle) {
	memset(handle, 0, sizeof(*handle));
	handle->loop = loop;
	QUEUE_INIT(&handle->timer_queue);

	return 0;
}

static void __uv_timer_enable(uv_timer_t* handle) {
	if(handle->flags & UV__HANDLE_ACTIVE) {
		return;
	}
	handle->flags |= UV__HANDLE_ACTIVE;
	QUEUE_INSERT_TAIL(&handle->loop->timer_queue, &handle->timer_queue);
}

static void __uv_timer_disable(uv_timer_t* handle) {
	if(!(handle->flags & UV__HANDLE_ACTIVE)) {
		return;
	}
	handle->flags &= ~UV__HANDLE_ACTIVE;
	QUEUE_REMOVE(&handle->timer_queue);
}

int uv_timer_start(uv_timer_t* handle, uv_timer_cb cb, uint64_t timeout, uint64_t repeat) {
	handle->callback = cb;
	handle->timeout = timeout;
	handle->repeat = repeat;

	handle->next_wakeup = HAL_GetTick() + timeout;
	__uv_timer_enable(handle);

	return 0;
}

int uv_timer_stop(uv_timer_t* handle) {
	__uv_timer_disable(handle);

	return 0;
}

int uv_timer_again(uv_timer_t* handle) {
	handle->next_wakeup = handle->next_wakeup + handle->repeat;
	__uv_timer_enable(handle);

	return 0;
}

void uv_timer_set_repeat(uv_timer_t* handle, uint64_t repeat) {
	handle->repeat = repeat;
}

uint64_t uv_timer_get_repeat(const uv_timer_t* handle) {
	return handle->repeat;
}

int uv_idle_init(uv_loop_t* loop, uv_idle_t* handle) {
	memset(handle, 0, sizeof(*handle));
	handle->loop = loop;
	QUEUE_INIT(&handle->idle_queue);

	return 0;
}

int uv_idle_start(uv_idle_t* handle, uv_idle_cb cb) {
	handle->callback = cb;

	if(handle->flags & UV__HANDLE_ACTIVE) {
		return 0;
	}
	handle->flags |= UV__HANDLE_ACTIVE;
	QUEUE_INSERT_TAIL(&handle->loop->idle_queue, &handle->idle_queue);

	return 0;
}
int uv_idle_stop(uv_idle_t* handle) {
	if(!(handle->flags & UV__HANDLE_ACTIVE)) {
		return 0;
	}
	handle->flags &= ~UV__HANDLE_ACTIVE;
	QUEUE_REMOVE(&handle->idle_queue);

	return 0;
}

UV_EXTERN int uv_async_init(uv_loop_t* loop, uv_async_t* handle, uv_async_cb async_cb) {
	memset(handle, 0, sizeof(*handle));
	handle->loop = loop;
	handle->callback = async_cb;
	QUEUE_INIT(&handle->async_queue);

	handle->flags |= UV__HANDLE_ACTIVE;
	QUEUE_INSERT_TAIL(&handle->loop->async_queue, &handle->async_queue);

	return 0;
}

UV_EXTERN int uv_async_send(uv_async_t* handle) {
	handle->pending = 1;
	__DMB();
	handle->loop->async_pending = 1;
	__DMB();

	return 0;
}