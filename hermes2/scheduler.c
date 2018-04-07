#include "scheduler.h"

#include <xtimer.h>
#include <thread.h>

#define ENABLE_DEBUG 1
#include "debug.h"

static uint32_t _gcd(uint32_t a, uint32_t b) {
  while (b != 0) {
    a %= b;
    a ^= b;
    b ^= a;
    a ^= b;
  }
  return a;
}

static void * _thread(void * arg) {
  scheduler_t * sched = (scheduler_t *)arg;
  while(1) {
    xtimer_periodic_wakeup(&sched->last_wakeup, sched->period);
    //DEBUG("sched !");
    for(size_t i = 0 ; i < sched->max ; i++) {
      scheduler_task_t * task = &sched->tasks[i];

      if(task->cb) {
        task->counter++;
        const uint32_t wait_duration = task->counter * sched->period;

        if(wait_duration >= task->period) {
          task->cb(task->arg);
          task->counter = 0;
        }
      }
    }
  }
}

int scheduler_init(scheduler_t * sched, scheduler_task_t * tasks, size_t max) {
  sched->tasks = tasks;
  sched->max = max;
  sched->last_wakeup.ticks32 = 0;
  sched->period = 1000000;

  thread_create(sched->stack, THREAD_STACKSIZE_DEFAULT, THREAD_PRIORITY_MAIN-1, 0, _thread, sched, "scheduler");
  return 0;
}

int scheduler_add_task(scheduler_t * sched, uint32_t freq, scheduler_cb_t cb, void * arg) {
  for(size_t i = 0 ; i < sched->max ; i++) {
    scheduler_task_t * task = &sched->tasks[i];
    if(!task->cb) {
      task->cb = cb;
      task->arg = arg;
      task->period = 1000000 / freq;
      task->counter = 0;
      sched->period = _gcd(sched->period, task->period);
      return i;
    }
  }

  return -1;
}

void scheduler_rm_task(scheduler_t * sched, int taskid);
