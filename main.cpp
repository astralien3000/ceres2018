#include "control_layer_3.hpp"
#include "pull.h"
#include "side.h"

#include "positioning_action.h"

int main(void) {
  Serial.begin(115200);
  ControlLayer0::instance().init();
  ControlLayer1::instance().init();
  ControlLayer2::instance().init();
  ControlLayer3::instance().init();

  Pull::init();
  Side::init();

  PositioningAction pos_act;
  pos_act.config.pos.x = 0;
  pos_act.config.pos.y = 0;
  pos_act.config.dir.x = -10;
  pos_act.config.dir.y = 10;
  pos_act.init();

  while(1) {
    if(!Pull::isPresent()) {
      pos_act.update();
    }

    Serial.print(ControlLayer3::instance().loc.getX());
    Serial.print(" ");
    Serial.print(ControlLayer3::instance().loc.getY());
    Serial.println("");
    delay(100);
  }
}

#if 0
#include <rclc/rclc.h>

#include <ceres2018_msgs/msg/encoders.h>

#include <xtimer.h>

#include <stdio.h>
#include <string.h>

#include "motor.hpp"
#include "encoder.hpp"
#include "scheduler.hpp"

#include "secure_motor.h"

#include "odometer.h"
#include "differential.h"
#include "locator.h"

#include "pid.h"
#include "control_system.h"

#include "trajectory_manager.h"

#include "positioning_action.h"

#include "pull.h"
#include "side.h"

#define __BSD_VISIBLE 1
#include <math.h>

enum {
  LEFT, RIGHT, ANGLE
};

#define ENC_FREQ (200)
#define MOT_CS_FREQ (800)
#define ROBOT_CS_FREQ (50)

#define MOT_MAX (100)

motor_pwm_dev_cfg_t mots_pwm = {
  .dev = PWM_DEV(0),
  .freq = 40000u,
  .res = 256
};

motor_cfg_t mots_cfg[] = {
  [LEFT] = {
    .pwm = {
      .chan = 0,
      .max = MOT_MAX,
    },
    .dir = {
      .pos_pin = GPIO_PIN(PORT_A, 5),
      .neg_pin = GPIO_PIN(PORT_A, 4),
    }
  },
  [RIGHT] = {
    .pwm = {
      .chan = 1,
      .max = MOT_MAX,
    },
    .dir = {
      .pos_pin = GPIO_PIN(PORT_E, 0),
      .neg_pin = GPIO_PIN(PORT_E, 1),
    }
  },
};

encoder_cfg_t encs_cfg[] = {
  [LEFT] = {
    .dev = 1,
    .mode = QDEC_X4,
    .ppr = 4096,
    .radius = 3, // cm
    .invert = false,
    .freq = ENC_FREQ,
  },
  [RIGHT] = {
    .dev = 2,
    .mode = QDEC_X4,
    .ppr = 4096,
    .radius = 3, // cm
    .invert = true,
    .freq = ENC_FREQ,
  },
};

secure_motor_cfg_t smot_cfg = {
    .duration = 0.5,
    .freq = 10,
    .min_cmd = 10,
    .max_speed = 1,
};

odometer_cfg_t odo_cfg = {
  .wheels_distance = 11.16, //11
};

locator_cfg_t loc_cfg = {
  .freq = 50,
};

pid_cfg_t pid_cfgs[] = {
  [LEFT] = {
    .kp = 4,
    .ki = 1,
    .kd = 1,
    .freq = MOT_CS_FREQ,
  },
  [RIGHT] = {
    .kp = 4,
    .ki = 1,
    .kd = 1,
    .freq = MOT_CS_FREQ,
  },
  [ANGLE] = {
    .kp = 32,
    .ki = 50,
    .kd = 0,
    .freq = ROBOT_CS_FREQ,
  },
};

#define MAX_TASKS 16

scheduler_t sched;
scheduler_task_t tasks[MAX_TASKS];

motor_t lmot, rmot;
encoder_t lenc, renc;

secure_motor_t lsmot, rsmot;

odometer_t odo;
differential_t diff;
locator_t loc;

pid_filter_t lpid, rpid;
control_system_t lcs, rcs;

pid_filter_t apid;
control_system_t acs;

trajectory_manager_t traj;

int main(int argc, char* argv[])
{
  scheduler_init(&sched, tasks, MAX_TASKS);

  motor_init(&lmot, &mots_pwm, &mots_cfg[LEFT]);
  motor_init(&rmot, &mots_pwm, &mots_cfg[RIGHT]);

  encoder_init(&lenc, &sched, &encs_cfg[LEFT]);
  encoder_init(&renc, &sched, &encs_cfg[RIGHT]);

  secure_motor_init(&lsmot, &sched, &lmot, &lenc, &smot_cfg);
  secure_motor_init(&rsmot, &sched, &rmot, &renc, &smot_cfg);

  odometer_init(&odo, &lenc, &renc, &odo_cfg);

  locator_init(&loc, &sched, &odo, &loc_cfg);

  pid_init(&lpid, &pid_cfgs[LEFT]);
  pid_init(&rpid, &pid_cfgs[RIGHT]);

  pid_init(&apid, &pid_cfgs[ANGLE]);
  apid.sum_coeff = 9.0/10.0;

  {
    differential_cfg_t cfg = {
      .left_motor = &lcs,
      .left_motor_set = control_system_set,
      .right_motor = &rcs,
      .right_motor_set = control_system_set,
    };
    differential_init(&diff, &cfg);
  }

  {
    control_system_cfg_t cfg = {
      .freq = MOT_CS_FREQ,

      .feedback_filter_eval = 0,
      .command_filter_eval = 0,

      .error_filter = &lpid,
      .error_filter_eval = pid_eval,

      .sensor = &lenc,
      .sensor_read = encoder_read_speed,

      .actuator = &lsmot,
      .actuator_set = secure_motor_set,
    };

    control_system_init(&lcs, &sched, &cfg);
  }

  {
    control_system_cfg_t cfg = {
      .freq = MOT_CS_FREQ,

      .feedback_filter_eval = 0,
      .command_filter_eval = 0,

      .error_filter = &rpid,
      .error_filter_eval = pid_eval,

      .sensor = &renc,
      .sensor_read = encoder_read_speed,

      .actuator = &rsmot,
      .actuator_set = secure_motor_set,
    };

    control_system_init(&rcs, &sched, &cfg);
  }

  {
    control_system_cfg_t cfg = {
      .freq = ROBOT_CS_FREQ,

      .feedback_filter_eval = 0,
      .command_filter_eval = 0,

      .error_filter = &apid,
      .error_filter_eval = pid_eval,

      .sensor = &odo,
      .sensor_read = odometer_read_angle,

      .actuator = &diff,
      .actuator_set = differential_set_angular,
    };

    control_system_init(&acs, &sched, &cfg);
  }

  {
    trajectory_manager_cfg_t cfg = {
      .loc = &loc,

      .linear_speed = &diff,
      .linear_speed_set = differential_set_linear,

      .angle = &acs,
      .angle_set = control_system_set,

      .freq = 20,
      .speed = 20,
    };

    trajectory_manager_init(&traj, &sched, &cfg);
  }

  locator_reset(&loc, 20, 20, 0);
  control_system_set(&acs, 0);

  positioning_action_cfg_t pos_act_cfg = {
    .pos = { .x =  20, .y =  20 },
    .dir = { .x = -10, .y = 10 },
  };

  positioning_action_t pos_act;
  positioning_action_init(&pos_act, &pos_act_cfg);

  pull_init();
  side_init();

  while (1) {
    //float p1 = locator_read_x(&loc);
    //float p2 = locator_read_y(&loc);

    float p1 = encoder_read_speed(&lenc);
    float p2 = encoder_read_speed(&renc);

    //differential_set_linear(&diff, 20);

    //trajectory_manager_goto(&traj, 0, 0);
    positioning_action_update(&pos_act);

    //printf("%f;%f\n", p1, p2);
    if(side_read() == GREEN) {
      printf("GREEN\n");
    }
    else {
      printf("ORANGE\n");
    }

    xtimer_usleep(1000000 / 100);
  }

  return 0;
}
#endif
