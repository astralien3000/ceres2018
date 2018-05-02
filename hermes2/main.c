#include <rclc/rclc.h>

#include <ceres2018_msgs/msg/encoders.h>

#include <xtimer.h>

#include <stdio.h>
#include <string.h>

#include "motor.h"
#include "encoder.h"
#include "scheduler.h"

#include "secure_motor.h"

#include "odometer.h"
#include "differential.h"
#include "locator.h"

#include "pid.h"
#include "control_system.h"

#include "trajectory_manager.h"

#define __BSD_VISIBLE 1
#include <math.h>

enum {
  LEFT, RIGHT, ANGLE
};

#define ENC_FREQ (200)
#define MOT_CS_FREQ (200)
#define ROBOT_CS_FREQ (50)

#define MOT_MAX (80)

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
    .min_cmd = 5,
    .max_speed = 1,
};

odometer_cfg_t odo_cfg = {
  //.wheels_distance = 11,
  .wheels_distance = 11.16,
};

locator_cfg_t loc_cfg = {
  .freq = 50,
};

pid_cfg_t pid_cfgs[] = {
  [LEFT] = {
    .kp = 6,
    .ki = 1,
    .kd = 0,
    .freq = MOT_CS_FREQ,
  },
  [RIGHT] = {
    .kp = 6,
    .ki = 1,
    .kd = 0,
    .freq = MOT_CS_FREQ,
  },
  [ANGLE] = {
    .kp = 32,
    .ki = 50,
    .kd = 0,
    .freq = ROBOT_CS_FREQ,
  },
};

void _callback(const void* v_msg)
{
  const ceres2018_msgs__msg__Encoders* msg = v_msg;
  printf("motors: [%i;%i]\n", (int)msg->left, (int)msg->right);
}

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

      .freq = 10,
      .speed = 20,
    };

    trajectory_manager_init(&traj, &sched, &cfg);
  }

  /*
  rclc_init(0, NULL);
  rclc_node_t* node = rclc_create_node("encoders", "");
  rclc_publisher_t* pub = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(ceres2018_msgs, msg, Encoders), "encoders", 1);
  rclc_subscription_t* sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(ceres2018_msgs, msg, Encoders), "motors", _callback, 1, false);

  ceres2018_msgs__msg__Encoders msg;
  */

  //while (rclc_ok()) {
  while (1) {
    float p1 = locator_read_x(&loc);
    float p2 = locator_read_y(&loc);

    //motor_set(&lmot, p1);
    //motor_set(&rmot, p2);

    //differential_set_linear(&diff, 0);
    //differential_set_angular(&diff, (M_PI * 2.0 / 10.0) * 11.0);
    //char * buff[128];
    //int size = 0;

    //control_system_set(&lcs, -10);
    //control_system_set(&rcs, 10);

    //control_system_set(&acs, 0);

    trajectory_manager_goto(&traj, 10, 0);

    //printf("%f;%f\n", p1, p2);
    //printf("position: [%f;%f]\n", locator_read_x(&loc), locator_read_y(&loc));

    xtimer_usleep(1000000 / 100);
    /*
    msg.left = p1;
    msg.right = p2;
    printf("encoders: [%i;%i]\n", (int)msg.left, (int)msg.right);
    
    rclc_publish(pub, (const void*)&msg);

    rclc_spin_node_once(node, 1000/20);
    */
  }

  /*
  rclc_destroy_publisher(pub);
  rclc_destroy_node(node);
  */
  return 0;
}
