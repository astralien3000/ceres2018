#include <rclc/rclc.h>

#include <ceres2018_msgs/msg/encoders.h>

#include <xtimer.h>

#include <stdio.h>
#include <string.h>

#include "motor.h"
#include "encoder.h"
#include "scheduler.h"

#include "odometer.h"
#include "differential.h"

#define __BSD_VISIBLE 1
#include <math.h>

motor_pwm_dev_cfg_t mots_pwm = {
  .dev = PWM_DEV(0),
  .freq = 30000u,
  .res = 256
};

motor_cfg_t mots_cfg[] = {
  {
    .pwm = {
      .chan = 0,
      .max = 150,
    },
    .dir = {
      .pos_pin = GPIO_PIN(PORT_A, 5),
      .neg_pin = GPIO_PIN(PORT_A, 4),
    }
  },
  {
    .pwm = {
      .chan = 1,
      .max = 150,
    },
    .dir = {
      .pos_pin = GPIO_PIN(PORT_E, 0),
      .neg_pin = GPIO_PIN(PORT_E, 1),
    }
  },
};

encoder_cfg_t encs_cfg[] = {
  {
    .dev = 1,
    .mode = QDEC_X4,
    .ppr = 4096,
    .radius = 3, // cm
    .invert = false,
    .freq = 100,
  },
  {
    .dev = 2,
    .mode = QDEC_X4,
    .ppr = 4096,
    .radius = 3, // cm
    .invert = true,
    .freq = 100,
  },
};

odometer_cfg_t odo_cfg = {
  .wheels_distance = 11,
};

void _callback(const void* v_msg)
{
  const ceres2018_msgs__msg__Encoders* msg = v_msg;
  printf("motors: [%i;%i]\n", (int)msg->left, (int)msg->right);
}

scheduler_t sched;
scheduler_task_t tasks[8];

motor_t lmot, rmot;
encoder_t lenc, renc;

odometer_t odo;
differential_t diff;

int main(int argc, char* argv[])
{
  scheduler_init(&sched, tasks, 8);

  motor_init(&lmot, &mots_pwm, &mots_cfg[0]);
  motor_init(&rmot, &mots_pwm, &mots_cfg[1]);

  encoder_init(&lenc, &sched, &encs_cfg[0]);
  encoder_init(&renc, &sched, &encs_cfg[1]);

  odometer_init(&odo, &lenc, &renc, &odo_cfg);

  differential_init(&diff, &lmot, &rmot);

  /*
  rclc_init(0, NULL);
  rclc_node_t* node = rclc_create_node("encoders", "");
  rclc_publisher_t* pub = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(ceres2018_msgs, msg, Encoders), "encoders", 1);
  rclc_subscription_t* sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(ceres2018_msgs, msg, Encoders), "motors", _callback, 1, false);

  ceres2018_msgs__msg__Encoders msg;
  */

  //while (rclc_ok()) {
  while (1) {
    /*
    float p1 = encoder_read_distance(&lenc);
    float p2 = encoder_read_distance(&renc);
    /*/
    float p1 = odometer_read_distance(&odo);
    float p2 = odometer_read_angle(&odo) * 180 / M_PI;
    //*/

    /*
    motor_set(&lmot, 80);
    motor_set(&rmot, 80);
    */
    differential_set_speed(&diff, -p1 * 10);
    differential_set_angular(&diff, -p2);

    char * buff[128];
    int size = 0;

    printf("encoders: [%f;%f]\n", p1, p2);

    xtimer_usleep(10000);
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
