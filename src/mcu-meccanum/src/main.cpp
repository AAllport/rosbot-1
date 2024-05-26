#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "motor.h"

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__String string_msg;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

void error_loop(bool limit = 1)
{
    // for (int i = 0; i < 21; i += limit)
    // {
    //     // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    //     delay(100);
    // }
}

// twist message cb
void subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;
    handleMovement(twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    auto speeds = getSpeeds();

    char buffer[255];
    auto len = sprintf(
        buffer,
        "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
        speeds[0],
        speeds[1],
        speeds[2],
        speeds[3]);

    string_msg.data = micro_ros_string_utilities_set(string_msg.data, buffer);
    RCSOFTCHECK(rcl_publish(&publisher, &string_msg, NULL));
}

void setup()
{
    setupMotors();

    Serial.begin(115200);

    set_microros_serial_transports(Serial);
    delay(2000);
    rmw_uros_sync_session(500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    delay(2000);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "mcu_meccanum_node", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "cmd_vel_out"));

    micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        &string_msg,
        (micro_ros_utilities_memory_conf_t){});

    // create timer
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(200),
        timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    delay(100);
}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(200)));
}