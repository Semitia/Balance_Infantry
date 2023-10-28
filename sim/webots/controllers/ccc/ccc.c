#include "./include/Ctrl.h"

Dormily_t Dormily;
Display_t display;

double torque[2] = {0, 0};
int main(int argc, char **argv) {
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);

  Dormily_init(&Dormily);
  while (wb_robot_step(TIME_STEP) != -1) {
    //double start_ts = wb_robot_get_time();

    keyInput();
    updateState(&Dormily);
    // Dormily.motor_torque_out[0] = torque[0];
    // Dormily.motor_torque_out[1] = torque[1];
    // setMotorTorque(&Dormily, Dormily.motor_torque_out[0], Dormily.motor_torque_out[1]);
    dormilyCtrl(&Dormily);
    drawData(&Dormily);

    //double end_ts = wb_robot_get_time();
    //printf("used time: %f\n", end_ts - start_ts);
  };

  // hardwareCheck();

  wb_robot_cleanup();
  return 0;
}

void keyInput(void) {
    int key = wb_keyboard_get_key();
    switch(key) {
      case WB_KEYBOARD_UP:
        //printf("Forward\n");
        torque[0] = 1;
        torque[1] = 1;
        break;
      case WB_KEYBOARD_DOWN:
        //printf("Backward\n");
        torque[0] = -1;
        torque[1] = -1;
        
        break;
      case WB_KEYBOARD_LEFT:
        //printf("Left\n");
        torque[0] = 1;
        torque[1] = -1;
        break;
      case WB_KEYBOARD_RIGHT:
        //printf("Right\n");
        torque[0] = 1;
        torque[1] = -1;
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
        torque[0] = 0;
        torque[1] = 0;
        break;
    }
}


void hardwareCheck(void) {
  WbDeviceTag motor1 = wb_robot_get_device("motor1");
  WbDeviceTag motor2 = wb_robot_get_device("motor2");
  // wb_motor_set_position(motor1, INFINITY);
  // wb_motor_set_position(motor2, INFINITY);
  WbDeviceTag pos_ss1 = wb_robot_get_device("position_sensor1");
  WbDeviceTag pos_ss2 = wb_robot_get_device("position_sensor2");
  wb_position_sensor_enable(pos_ss1, TIME_STEP);
  wb_position_sensor_enable(pos_ss2, TIME_STEP);
  displayInit(&display);
  while(wb_robot_step(TIME_STEP) != -1) {
    keyInput();
    double pos1 = wb_position_sensor_get_value(pos_ss1);
    double pos2 = wb_position_sensor_get_value(pos_ss2);
    static double last_pos1, last_pos2;
    // wb_motor_set_velocity(motor1, torque[0]);
    // wb_motor_set_velocity(motor2, torque[1]);
    wb_motor_set_torque(motor1, torque[0]);
    wb_motor_set_torque(motor2, torque[1]);
    double vel1 = (pos1 - last_pos1) / TIME_STEP * 1000;
    double vel2 = (pos2 - last_pos2) / TIME_STEP * 1000;
    last_pos1 = pos1;
    last_pos2 = pos2;
    printf("pos1:%f, vel1:%f, pos2:%f, vel2:%f\n", pos1, vel1, pos2, vel2);
    printf("torque1:%f, torque2:%f\n", torque[0], torque[1]);
    //打印当前时间戳
    //printf("time:%f\n", wb_robot_get_time()*1000);
  }
  return;
}