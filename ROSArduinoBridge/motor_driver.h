/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 8
  #define LEFT_MOTOR_BACKWARD  13
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   12
  #define RIGHT_MOTOR_ENABLE 10
  #define LEFT_MOTOR_ENABLE 11


  #define BACK_LEFT_MOTOR_BACKWARD 7
  #define BACK_RIGHT_MOTOR_BACKWARD A0
  #define BACK_LEFT_MOTOR_FORWARD  4
  #define BACK_RIGHT_MOTOR_FORWARD A1
  #define BACK_LEFT_MOTOR_ENABLE 5
  #define BACK_RIGHT_MOTOR_ENABLE 6
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
