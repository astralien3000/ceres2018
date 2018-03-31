#include <rclc/rclc.h>

#include <ceres2018_msgs/msg/encoders.h>

//#include "usb_serial.h"
#include "periph/qdec.h"
#include "periph/pwm.h"
#include "periph/gpio.h"

#include <stdio.h>
#include <string.h>

#include "motor.h"

#define MOT1_CHAN 0
#define MOT1_DIR1 GPIO_PIN(PORT_A, 4)
#define MOT1_DIR2 GPIO_PIN(PORT_A, 5)

#define MOT2_CHAN 1
#define MOT2_DIR1 GPIO_PIN(PORT_E, 0)
#define MOT2_DIR2 GPIO_PIN(PORT_E, 1)

#define MAX_CMD 150

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
      .pos_pin = GPIO_PIN(PORT_A, 4),
      .neg_pin = GPIO_PIN(PORT_A, 5),
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

void _callback(const void* v_msg)
{
  const ceres2018_msgs__msg__Encoders* msg = v_msg;
  printf("motors: [%i;%i]\n", (int)msg->left, (int)msg->right);
}

int main(int argc, char* argv[])
{
  motor_t lmot, rmot;
  motor_init(&lmot, &mots_pwm, &mots_cfg[0]);
  motor_init(&rmot, &mots_pwm, &mots_cfg[1]);

  qdec_init(1, QDEC_X4, NULL, NULL);
  qdec_init(2, QDEC_X4, NULL, NULL);

  rclc_init(0, NULL);
  rclc_node_t* node = rclc_create_node("encoders", "");
  rclc_publisher_t* pub = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(ceres2018_msgs, msg, Encoders), "encoders", 1);
  rclc_subscription_t* sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(ceres2018_msgs, msg, Encoders), "motors", _callback, 1, false);

  ceres2018_msgs__msg__Encoders msg;

  while (rclc_ok()) {
    int p1 = qdec_read(1);
    int p2 = qdec_read(2);

    motor_set(lmot, p1/16);
    motor_set(rmot, p2/16);

    msg.left = p1;
    msg.right = p2;
    printf("encoders: [%i;%i]\n", (int)msg.left, (int)msg.right);
    
    rclc_publish(pub, (const void*)&msg);

    rclc_spin_node_once(node, 1000/20);
  }
    
  rclc_destroy_publisher(pub);
  rclc_destroy_node(node);
  return 0;
}
