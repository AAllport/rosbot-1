#include <Arduino.h>
#include <stdio.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <nav_msgs/msg/odometry.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "maths.h"
#include "motor.h"

static micro_ros_utilities_memory_conf_t conf = {0};

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
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

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    Serial2.printf("Timer Callback: %d\n",last_call_time);
    if (timer == NULL)
    {
        return;
    }

    // struct timespec tv = {0};
    // clock_gettime(0, &tv);

    // odom_msg.header.stamp.sec = tv.tv_sec;
    // odom_msg.header.stamp.nanosec = tv.tv_nsec;

    Serial2.printf("Stamp: %d\n", tv.tv_sec);

    handleIntergration(&odom_msg, &twist_msg);

    Serial2.printf("Pose: %f, %f, %f\t", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
    Serial2.printf("Yaw: %f\t", maths::quat_to_yaw(odom_msg.pose.pose.orientation) * 180 / 3.14);
    Serial2.printf("Orientation: %f, %f, %f, %f\n", odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);

    RCSOFTCHECK(rcl_publish(&publisher, &odom_msg, NULL));
}

void setup()
{
    setupMotors();

    Serial.begin(115200);

    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    Serial2.println("Serial2 started");
    Serial2.printf("Built at: %s %s\n", __DATE__, __TIME__);

    set_microros_serial_transports(Serial);
    delay(1000);
    rmw_uros_sync_session(500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    Serial2.print("Setting up odometry...\t");
    // ODOM Setup
    RCCHECK(micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), &odom_msg, conf));
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
    odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");

    odom_msg.pose.pose.position.x = 0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation = maths::pack_quat(maths::euler_to_quat(0, 0, 0));
    Serial2.println("Done");

    // create node
    RCCHECK(rclc_node_init_default(&node, "mcu_meccanum_node", "", &support));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));

    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(200), timer_callback));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

    delay(100);
}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(200)));
}