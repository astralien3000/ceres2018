#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdlib.h>

#include <thread.h>

typedef void (*scheduler_cb_t)(void *arg);

typedef struct {
  uint32_t freq;
  scheduler_cb_t cb;
  void * arg;
} scheduler_task_t;

typedef struct {
  scheduler_task_t * tasks;
  size_t max;
  char stack[THREAD_STACKSIZE_DEFAULT];
} scheduler_t;

int scheduler_init(scheduler_t * sched, scheduler_task_t * tasks, size_t max);

int scheduler_add_task(scheduler_t * sched, uint32_t freq, scheduler_cb_t cb, void * arg);
void scheduler_rm_task(scheduler_t * sched, int taskid);

#endif//SCHEDULER_H
