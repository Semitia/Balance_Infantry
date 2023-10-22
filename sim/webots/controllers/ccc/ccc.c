#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  WbDeviceTag imu;
  imu = wb_robot_get_device("imu");
  wb_inertial_unit_enable(imu, TIME_STEP);
  
  char wheels_names[2][8] = {"motor1", "motor2"};
  WbDeviceTag wheels[2];
  
  for(int i=0;i<2;i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }

  double left_speed =  1.0;
  double right_speed = 1.0;
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    //printf("IMU roll: %f pitch: %f yaw: %f\n", rpy[0], rpy[1], rpy[2]);

    int key = wb_keyboard_get_key();
    switch(key) {
      case WB_KEYBOARD_UP:
        printf("Forward\n");
        left_speed = 1.0;
        right_speed = 1.0;
        break;
      case WB_KEYBOARD_DOWN:
        printf("Backward\n");
        left_speed = -1.0;
        right_speed = -1.0;
        break;
      case WB_KEYBOARD_LEFT:
        printf("Left\n");
        left_speed = -1.0;
        right_speed = 1.0;
        break;
      case WB_KEYBOARD_RIGHT:
        printf("Right\n");
        left_speed = 1.0;
        right_speed = -1.0;
        break;
      default:
        break;
      }
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
  };

  wb_robot_cleanup();

  return 0;
}
