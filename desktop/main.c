#include <rclc/rclc.h>

#include <ceres2018_msgs/msg/encoders.h>

#include <stdio.h>
#include <string.h>

ceres2018_msgs__msg__Encoders enc;

void _callback(const void* v_msg)
{
  const ceres2018_msgs__msg__Encoders* msg = v_msg;
  printf("I heard: [%i;%i]\n", (int)msg->left, (int)msg->right);
  enc.left = msg->left;
  enc.right = msg->right;
}

int main(int argc, char* argv[])
{
  rclc_init(0, NULL);
  rclc_node_t* node = rclc_create_node("encoders", "");
  rclc_publisher_t* pub = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(ceres2018_msgs, msg, Encoders), "motors", 1);
  rclc_subscription_t* sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(ceres2018_msgs, msg, Encoders), "encoders", _callback, 1, false);

  ceres2018_msgs__msg__Encoders msg;

  int i = 0;
  while (rclc_ok()) {
    msg.left = enc.left;
    msg.right = enc.right+42;
    i++;
    printf("Publish: [%i;%i]\n", (int)msg.left, (int)msg.right);

    rclc_publish(pub, (const void*)&msg);

    rclc_spin_node_once(node, 1000/20);
  }

  rclc_destroy_subscription(sub);
  rclc_destroy_node(node);
  return 0;
}
