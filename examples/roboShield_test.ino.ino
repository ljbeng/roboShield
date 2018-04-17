/*
 * roboShield Example
 * Control 8 servos
 * Control 2 motors including direction
 * only using i2c pins
 * This example creates 2 instances of the PCA9685 library
 * servos can be controlled from either instance
 * motor 1 is controlled by instance servos_Motor1
 * motor 2 is controlled by instance servos_Motor2
 * The library is a mash of PCA9685 and TB6612FNG libraries.
 * The PCA9685 has 16 PWM capable outputs.  8 are utilized as servo control and 7 are connected to the TB6612FNG
 *  for control of 2 motors
 * 
 */

#include <Wire.h>
#include <roboShield.h>

roboShield servos_Motor1(Wire, PCA9685_PhaseBalancer_Weaved,1); // Library using Wire and weaved phase balancing scheme
roboShield servos_Motor2(Wire, PCA9685_PhaseBalancer_Weaved,2); // Library using Wire and weaved phase balancing scheme
roboShield_ServoEvaluator pwmServo1;

void setup() {
    Serial.begin(115200);

    Wire.begin();                      // Wire must be started first
    Wire.setClock(400000);             // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    servos_Motor1.resetDevices();       // Software resets all PCA9685 devices on Wire line
    servos_Motor1.init(0x40);           // Address pins A5-A0 set to 0x40
    servos_Motor1.setPWMFrequency(50);  // 50Hz provides 20ms standard servo phase length

    servos_Motor2.resetDevices();       // Software resets all PCA9685 devices on Wire line
    servos_Motor2.init(0x40);           // Address pins A5-A0 set to 0x40
    servos_Motor2.setPWMFrequency(50);  // 50Hz provides 20ms standard servo phase length

    Serial.println("reset done");
    delay(3000);
    
}
int sp = -0x1000;
int srv = -90, dir = 1;
void loop() {

  //pwmController.setChannelPWM(13, 0);

  
  Serial.println("direct");
  Serial.println(pwmServo1.pwmForAngle(srv));
  servos_Motor1.setChannelPWM(0, pwmServo1.pwmForAngle(srv));
  servos_Motor1.setChannelPWM(1, 128 << 4);
  servos_Motor1.setChannelPWM(2, 128 << 4);
  servos_Motor1.setChannelPWM(3, 128 << 4);
  servos_Motor1.setChannelPWM(4, 128 << 4);
  servos_Motor1.setChannelPWM(5, 128 << 4);
  servos_Motor1.setChannelPWM(6, 128 << 4);
  servos_Motor1.setChannelPWM(7, 128 << 4);

  if (dir == 1) srv += 10;
  if (dir == 0) srv -= 10;
  if (srv > 90) dir = 0;
  if (srv < -90) dir = 1;
  servos_Motor1.drive(sp);
  delay(1000);

  servos_Motor2.drive(0x500);
  delay(1000);
  //pwmController.setChannelPWM(13, 0x1000);
  servos_Motor2.brake();
  delay(1000);
  servos_Motor2.drive(-0x500);
  delay(1000);
  servos_Motor1.brake();
  servos_Motor2.brake();
  delay(1000);

  sp += 0x100;
  if (sp > 0x1000) sp = -0x1000;
  Serial.println("for/back");
  forward(servos_Motor1,servos_Motor2);
  delay(1000);
  servos_Motor1.brake();
  servos_Motor2.brake();
  delay(1000);
  back(servos_Motor1,servos_Motor2);
  delay(1000);
  servos_Motor1.brake();
  servos_Motor2.brake();  
  Serial.println("drive duration");
  servos_Motor2.drive(0x500, 3000);
  delay(3000);
}
