#include "./include/Ctrl.h"

Dormily_t Dormily;

int main(int argc, char **argv) {
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  Dormily_init(&Dormily);

  double left_speed =  1.0;
  double right_speed = 1.0;
  while (wb_robot_step(TIME_STEP) != -1) {
    int key = wb_keyboard_get_key();
    switch(key) {
      case WB_KEYBOARD_UP:
        //printf("Forward\n");
        left_speed = 1.0;
        right_speed = 1.0;

        break;
      case WB_KEYBOARD_DOWN:
        //printf("Backward\n");
        left_speed = -1.0;
        right_speed = -1.0;
        break;
      case WB_KEYBOARD_LEFT:
        //printf("Left\n");
        left_speed = -1.0;
        right_speed = 1.0;
        break;
      case WB_KEYBOARD_RIGHT:
        //printf("Right\n");
        left_speed = 1.0;
        right_speed = -1.0;
        break;
      default:
        //printf("Stop\n");
        left_speed = 0.0;
        right_speed = 0.0;
        break;
    }
    //使用channel0、1显示速度
    Dormily.display.data[0] = left_speed;
    Dormily.display.data[1] = right_speed;
    addDisData(&Dormily.display);
    updateDis(&Dormily.display);
  };


  // WbDeviceTag display = wb_robot_get_device("display");
  // while(wb_robot_step(TIME_STEP) != -1) {
  //   wb_display_set_color(display, 0xff0000);
  //   wb_display_draw_pixel(display, 50, 10);   
  // }

  wb_robot_cleanup();
  return 0;
}
