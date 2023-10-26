#include "./include/Ctrl.h"

int main(int argc, char **argv) {
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);


  double left_speed =  1.0;
  double right_speed = 1.0;
  while (wb_robot_step(TIME_STEP) != -1) {
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
  };

  wb_robot_cleanup();

  return 0;
}
