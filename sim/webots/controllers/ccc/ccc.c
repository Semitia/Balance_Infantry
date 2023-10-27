#include "./include/Ctrl.h"

Dormily_t Dormily;

int main(int argc, char **argv) {
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  Dormily_init(&Dormily);

  while (wb_robot_step(TIME_STEP) != -1) {
    keyInput();
    updateState(&Dormily);
    drawData(&Dormily);

  };

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
        
        break;

      case 70: // f

        break;

      default:
        //printf("Stop\n");

        break;
    }
}