#include <Servo.h>

Servo sail;
Servo rudder;

//adjust this values with exact
int sail_max = 90;
int sail_straight = 0;
int sail_mid = 45;
int pos;

void init_servo()
{
  sail.attach(OUT5);
  rudder.attach(OUT6);
}

void adjust_sail(float a)
{
  a = abs(a);
  if (a < 45) {
    pos = 110;
  }
  else if (a >= 45 && a < 90) {
    pos = map(a, 45, 90, 110, 90);
  }
  else if (a >= 90 && a <= 180) {
    pos = map(a, 90, 180, 90, 45);
  }
  sail.write(pos);
}

void stop_sail(float a) {
  if (a <= 0 && a >= -90) {
    pos = map(a, -90, 0, 45, 110);
  }
  else if (a <= -90 && a >= -180) {
    pos = map(a, -180, -90, 110, 45);
  }
  else if (a <= 180 && a >= 90) {
    pos = map(a, 90, 180, 45, 110);
  }
  else if (a <= 90 && a >= 0) {
    pos = map(a, 0, 90, 110, 45);
  }
  sail.write(pos);
  rudder.write(90);
}

void adjust_rudder(float a) {
  float range = 10.00;
  if (a < -range && a >= -180) {
    pos = 45;
  }
  else if (a > range && a <= 180) {
    pos = 135;
  }
  else if (a <= range && a >= -range) {

    pos = 90;
  }
  rudder.write(pos);
}

void turnRudder(int a) {
  rudder.write(a);
  delay(100);
}

void turnSail(int a) {
  sail.write(a);
  delay(100);
}

void sailControl(unsigned long a) {
  sail.writeMicroseconds(a);
}

void rudderControl(unsigned long a) {
  rudder.writeMicroseconds(a);
}
