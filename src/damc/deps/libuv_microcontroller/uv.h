#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

# define UV_EXTERN /* nothing */

typedef struct uv_loop_s uv_loop_t;
typedef struct uv_timer_s uv_timer_t;
typedef struct uv_idle_s uv_idle_t;
typedef struct uv_async_s uv_async_t;

typedef void (*uv_timer_cb)(uv_timer_t* handle);
typedef void (*uv_idle_cb)(uv_idle_t* handle);
typedef void (*uv_async_cb)(uv_async_t* handle);

#define UV_HANDLE_FIELDS                                              \
	void* data;            \
	uv_loop_t* loop;              \
  unsigned int flags;                                                         \

struct uv_loop_s {
	void* timer_queue[2];
	void* idle_queue[2];
	int async_pending;
	void* async_queue[2];
};

struct uv_timer_s {
	UV_HANDLE_FIELDS

	void* timer_queue[2];
	
	uint32_t timeout;
	uint32_t repeat;
	uv_timer_cb callback;
	uint32_t next_wakeup;
};

struct uv_idle_s {
	UV_HANDLE_FIELDS
	
	void* idle_queue[2];
	uv_idle_cb callback;
};

struct uv_async_s {
	UV_HANDLE_FIELDS

	void* async_queue[2];
	uv_async_cb callback;
	int pending;
};

typedef enum {
  UV_RUN_DEFAULT = 0,
  UV_RUN_ONCE,
  UV_RUN_NOWAIT
} uv_run_mode;

UV_EXTERN uv_loop_t* uv_default_loop(void);
UV_EXTERN int uv_loop_init(uv_loop_t* loop);
UV_EXTERN int uv_loop_close(uv_loop_t* loop);
UV_EXTERN int uv_run(uv_loop_t*, uv_run_mode mode);

UV_EXTERN int uv_timer_init(uv_loop_t*, uv_timer_t* handle);
UV_EXTERN int uv_timer_start(uv_timer_t* handle,
                             uv_timer_cb cb,
                             uint64_t timeout,
                             uint64_t repeat);
UV_EXTERN int uv_timer_stop(uv_timer_t* handle);
UV_EXTERN int uv_timer_again(uv_timer_t* handle);
UV_EXTERN void uv_timer_set_repeat(uv_timer_t* handle, uint64_t repeat);
UV_EXTERN uint64_t uv_timer_get_repeat(const uv_timer_t* handle);

UV_EXTERN int uv_idle_init(uv_loop_t*, uv_idle_t* idle);
UV_EXTERN int uv_idle_start(uv_idle_t* idle, uv_idle_cb cb);
UV_EXTERN int uv_idle_stop(uv_idle_t* idle);

UV_EXTERN int uv_async_init(uv_loop_t*,
                            uv_async_t* async,
                            uv_async_cb async_cb);
UV_EXTERN int uv_async_send(uv_async_t* async);

#ifdef __cplusplus
}
#endif
