#ifndef DEV_H
#define DEV_H

#include "scheduler.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "secure_motor.hpp"
#include "odometer.hpp"
#include "differential.hpp"
#include "locator.hpp"
#include "control_system.h"
#include "pid.hpp"
#include "trajectory_manager.h"

extern scheduler_t sched;

extern motor_t lmot, rmot;
extern encoder_t lenc, renc;

extern secure_motor_t lsmot, rsmot;

extern odometer_t odo;
extern differential_t diff;
extern locator_t loc;

extern pid_filter_t lpid, rpid;
extern control_system_t lcs, rcs;

extern pid_filter_t apid;
extern control_system_t acs;

extern trajectory_manager_t traj;

#endif//DEV_H
