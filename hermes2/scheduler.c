#include "scheduler.h"

#include <xtimer.h>
#include <thread.h>

#include "usb_serial.h"
/*
static void * _thread(void * arg) {
  while(1) {
    usb_serial_write("Hello!\n", sizeof("Hello!\n"));
    xtimer_sleep(1);
  }
}

int scheduler_init(scheduler_t * sched, scheduler_task_t * tasks, size_t max) {
  sched->tasks = tasks;
  sched->max = max;

  thread_create(sched->stack, THREAD_STACKSIZE_DEFAULT, THREAD_PRIORITY_MAIN-1, 0,
                _thread, sched, "scheduler");
}

int scheduler_add_task(scheduler_t * sched, uint32_t freq, scheduler_cb_t cb, void * arg) {
  for(size_t i = 0 ; i < sched->max ; i++) {
    scheduler_task_t * task = &sched->tasks[i];
    if(!task->cb) {
      task->cb = cb;
      task->arg = arg;
      task->freq = freq;
      return i;
    }
  }

  return -1;
}

void scheduler_rm_task(scheduler_t * sched, int taskid);
*/
