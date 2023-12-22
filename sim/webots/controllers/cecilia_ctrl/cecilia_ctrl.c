#include "./cecilia_ctrl.h"

Cecilia_t cecilia;
bool on_off = 1;
double torque[2] = {0, 0};

int main(int argc, char **argv) {
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);

  ceciliaInit(&cecilia);

  while (wb_robot_step(TIME_STEP) != -1) {
    //double start_ts = wb_robot_get_time();

    keyInput();
    updateState(&cecilia);                    //更新状态
    updateLqrK(&cecilia);                     //更新控制器
    //calcOutput(&cecilia);                     //计算输出
    leg_test(&cecilia, 2, 0.3);
    if(on_off) setAllTorque(&cecilia);        //输出力矩
    drawData(&cecilia);                       //绘制波形

    //double end_ts = wb_robot_get_time();
    //printf("used time: %f\n", end_ts - start_ts);
  };

  // while (wb_robot_step(TIME_STEP) != -1) {
  //   //keyInput();
  //   peripherals_check(&cecilia);
  // }

  wb_robot_cleanup();
  return 0;
}

void keyInput(void) {
    int key = wb_keyboard_get_key();
    switch(key) {
      case WB_KEYBOARD_UP:
        //printf("Forward\n");

        break;
      case WB_KEYBOARD_DOWN:
        //printf("Backward\n");

        break;
      case WB_KEYBOARD_LEFT:
        //printf("Left\n");

        break;
      case WB_KEYBOARD_RIGHT:
        //printf("Right\n");

        break;
      case 48:
        printf("0\n");
        if(cecilia.Motor[0].torque_tgt>0) 
          cecilia.Motor[0].torque_tgt = -0.4;
        else 
          cecilia.Motor[0].torque_tgt = 0.4;
        break;
      case 49:
        printf("1\n");
        if(cecilia.Motor[1].torque_tgt>0) 
          cecilia.Motor[1].torque_tgt = -0.4;
        else 
          cecilia.Motor[1].torque_tgt = 0.4;
        break;
      case 50:
        printf("2\n");
        if(cecilia.Motor[2].torque_tgt>0) 
          cecilia.Motor[2].torque_tgt = -0.4;
        else 
          cecilia.Motor[2].torque_tgt = 0.4;
        break;
      case 51:
        printf("3\n");
        if(cecilia.Motor[3].torque_tgt>0) 
          cecilia.Motor[3].torque_tgt = -0.4;
        else 
          cecilia.Motor[3].torque_tgt = 0.4;
        break;
      case 52:
        printf("4\n");
        if(cecilia.Motor[4].torque_tgt>0) 
          cecilia.Motor[4].torque_tgt = -0.4;
        else 
          cecilia.Motor[4].torque_tgt = 0.4;
        break;
      case 53:
        printf("5\n");
        if(cecilia.Motor[5].torque_tgt>0) 
          cecilia.Motor[5].torque_tgt = -0.4;
        else 
          cecilia.Motor[5].torque_tgt = 0.4;
        break;

      case 87: // w
        
        break;

      case 83: // s
        
        break;

      case 65: // a
        
        break;

      case 68: // d
        
        break;
      case 72: // h
        
        break;

      case 76: // l
        
        break;

      case 74: // J
        on_off = 1;
        break;

      case 70: // f
        on_off = 0;
        break;

      default:
        //printf("Stop\n");
        torque[0] = 0;
        torque[1] = 0;

        break;
    }
}