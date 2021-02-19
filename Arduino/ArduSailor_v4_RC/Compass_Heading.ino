//INERTIAL MEASUREMENT UNIT (IMU) 6DOF as followed in http://www.starlino.com/imu_guide.html
//Using Simplified Kalman Filter (which uses predetermined weight of factors instead of Kalman Complex Noise achuchu)
//Jerome Kim G. Belo

#include <Wire.h>
#include <math.h>

#define addrs 0x1E
#define MPU6050 0x68

#define ToD(x) (x/131)
#define ToG(x) (x/16384) //currently in units of G=9.80665 m/s/s (x*9.80665/16384) for acceleration

//Calibrated Values for Sensor Offset
float AccelOffX;// = 50;
float AccelOffY;// = -576;
float AccelOffZ;// = 15106 - 16384;

float GyrosOffX;// = -70;
float GyrosOffY;// = -30;
float GyrosOffZ;// = -142;

float MagOffX = -13;
float MagOffY = -132;
float MagOffZ = -6;
float declination = -(2 + (19 / 60));
float mag_softiron_matrix[3][3] = { {  1.007, -0.007,  0.031 },
                                    { -0.007,  0.924, -0.035 },
                                    {  0.031, -0.035,  1.077 } };

//Raw OUTPUT of Sensors
float AccelX, AccelY, AccelZ;
float AccelXG, AccelYG, AccelZG;

float GyrosX, GyrosY, GyrosZ;
float GyrosXD, GyrosYD, GyrosZD;

int MagX, MagY, MagZ;

//Math Variables
double Racc, RxAcc, RyAcc, RzAcc;
double Rest, RxEst[2], RyEst[2], RzEst[2];
double Rgyro, RxGyro, RyGyro, RzGyro;
double Axz[2], Ayz[2];
double RateAxz, RateAyz;
int Sign;
int wGyro = 20; //5-20 Measure of how much we trust Gyroscope Data than Accelerometer
int count = 0;
float pastMillis = 0;
float currentMillis = 0;
float pitch, roll, Heading, GyrosHeading;

//smoothen output
const int numReadings = 90;
int readings[numReadings];
int readIndex = 0;
int total = 0;
float lastHeading;

void initMagnet() //initialize Magnetometer
{
  Wire.beginTransmission(addrs);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(addrs);
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.endTransmission();
}

void initMPU6050() //initialize MPU6050 (Accelerometer & Gyroscope)
{
  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void getInitSensVal()
{
  updateAccelerometer();
  RxEst[0] = RxAcc;
  RyEst[0] = RyAcc;
  RzEst[0] = RzAcc;

  Axz[0] = (atan2(RxEst[0], RzEst[0])) * (180 / PI);
  Ayz[0] = (atan2(RyEst[0], RzEst[0])) * (180 / PI);
}

void updateAccelerometer()
{
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050, 6);

  AccelX = (Wire.read() << 8 | Wire.read());
  AccelY = (Wire.read() << 8 | Wire.read());
  AccelZ = (Wire.read() << 8 | Wire.read());

  AccelX -= AccelOffX;
  AccelY -= AccelOffY;
  AccelZ -= AccelOffZ;

  AccelXG = ToG(AccelX);
  AccelYG = ToG(AccelY);
  AccelZG = ToG(AccelZ);

  RxAcc = AccelXG;
  RyAcc = AccelYG;
  RzAcc = AccelZG;

  Racc = sqrt((RxAcc * RxAcc) + (RyAcc * RyAcc) + (RzAcc * RzAcc));

  RxAcc = RxAcc / Racc;
  RyAcc = RyAcc / Racc;
  RzAcc = RzAcc / Racc;
}

void updateGyroscope()
{
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050, 6);
  GyrosX = (Wire.read() << 8 | Wire.read());
  GyrosY = (Wire.read() << 8 | Wire.read());
  GyrosZ = (Wire.read() << 8 | Wire.read());

  GyrosX -= GyrosOffX;
  GyrosY -= GyrosOffY;
  GyrosZ -= GyrosOffZ;

  GyrosXD = -ToD(GyrosX);
  GyrosYD = ToD(GyrosY);
  GyrosZD = -ToD(GyrosZ);

  RateAxz = GyrosYD;
  RateAyz = GyrosXD;
}

void updateMagnetometer()
{
  Wire.beginTransmission(addrs);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.requestFrom(addrs, 6);

  if (6 <= Wire.available())
  {
    MagX = Wire.read() << 8;
    MagX |= Wire.read();
    MagZ = Wire.read() << 8;
    MagZ |= Wire.read();
    MagY = Wire.read() << 8;
    MagY |= Wire.read();
  }

  MagX -= MagOffX;
  MagY -= MagOffY;
  MagZ -= MagOffZ;

  float mx = MagX * mag_softiron_matrix[0][0] + MagY * mag_softiron_matrix[0][1] + MagZ * mag_softiron_matrix[0][2];
  float my = MagX * mag_softiron_matrix[1][0] + MagY * mag_softiron_matrix[1][1] + MagZ * mag_softiron_matrix[1][2];
  float mz = MagX * mag_softiron_matrix[2][0] + MagY * mag_softiron_matrix[2][1] + MagZ * mag_softiron_matrix[2][2];

  MagX = mx;
  MagY = my;
  MagZ = mz;

  //  Heading = (atan2(MagY, MagX) * (180 / PI));
  //  Heading += declination;
  //  if (Heading < 0) {
  //    Heading = 360 + Heading;
  //  }
}

void calibrate()
{
  Serial.println("Calibrating Sensors...");

  float AccelXMIN = 16384;
  float AccelXMAX = -16384;
  float AccelYMIN = 16384;
  float AccelYMAX = -16384;
  float AccelZMIN = 16384;
  float AccelZMAX = -16384;

  float GyrosXMIN = 3000;
  float GyrosXMAX = -3000;
  float GyrosYMIN = 3000;
  float GyrosYMAX = -3000;
  float GyrosZMIN = 3000;
  float GyrosZMAX = -3000;

  float AccelXOff = 0;
  float AccelYOff = 0;
  float AccelZOff = 0;
  float GyrosXOff = 0;
  float GyrosYOff = 0;
  float GyrosZOff = 0;

  while (count < 200)
  {
    count = count + 1;
    
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050, 6);

    AccelX = (Wire.read() << 8 | Wire.read());
    AccelY = (Wire.read() << 8 | Wire.read());
    AccelZ = (Wire.read() << 8 | Wire.read());

    if (AccelX > AccelXMAX) AccelXMAX = AccelX;
    if (AccelX < AccelXMIN) AccelXMIN = AccelX;
    if (AccelY > AccelYMAX) AccelYMAX = AccelY;
    if (AccelY < AccelYMIN) AccelYMIN = AccelY;
    if (AccelZ > AccelZMAX) AccelZMAX = AccelZ;
    if (AccelZ < AccelZMIN) AccelZMIN = AccelZ;

    AccelOffX = (AccelXMAX + AccelXMIN) / 2;
    AccelOffY = (AccelYMAX + AccelYMIN) / 2;
    AccelOffZ = (AccelZMAX + AccelZMIN) / 2;

    AccelOffX += AccelXOff;
    AccelOffY += AccelYOff;
    AccelOffZ += AccelZOff;
  }
  AccelOffX /= count;
  AccelOffY /= count;
  AccelOffZ /= count;
  count = 0;

  while (count < 200)
  {
    count = count + 1;

    Wire.beginTransmission(MPU6050);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050, 6);
    GyrosX = (Wire.read() << 8 | Wire.read());
    GyrosY = (Wire.read() << 8 | Wire.read());
    GyrosZ = (Wire.read() << 8 | Wire.read());

    if (GyrosX > GyrosXMAX) GyrosXMAX = GyrosX;
    if (GyrosX < GyrosXMIN) GyrosXMIN = GyrosX;
    if (GyrosY > GyrosYMAX) GyrosYMAX = GyrosY;
    if (GyrosY < GyrosYMIN) GyrosYMIN = GyrosY;
    if (GyrosZ > GyrosZMAX) GyrosZMAX = GyrosZ;
    if (GyrosZ < GyrosZMIN) GyrosZMIN = GyrosZ;

    GyrosOffX = (GyrosXMAX + GyrosXMIN) / 2;
    GyrosOffY = (GyrosYMAX + GyrosYMIN) / 2;
    GyrosZOff = (GyrosZMAX + GyrosZMIN) / 2;

    GyrosOffX = GyrosOffX + GyrosXOff;
    GyrosOffY = GyrosOffY + GyrosYOff;
    GyrosOffZ = GyrosOffZ + GyrosZOff;
  }
  GyrosOffX /= count;
  GyrosOffY /= count;
  GyrosOffZ /= count;
  count = 0;

  Serial.print("Calibrated Values: ");
  Serial.print("Accel X:Y:Z = ");
  Serial.print(AccelOffX);
  Serial.print(":");
  Serial.print(AccelOffY);
  Serial.print(":");
  Serial.print(AccelOffZ);
  Serial.print("   ");
  Serial.print("Gyros X:Y:Z = ");
  Serial.print(GyrosOffX);
  Serial.print(":");
  Serial.print(GyrosOffY);
  Serial.print(":");
  Serial.print(GyrosOffZ);
  Serial.println();
}

void getHeading()
{
  updateAccelerometer();
  updateGyroscope();
  updateMagnetometer();

  Axz[0] = (atan2(RxEst[0], RzEst[0])) * (180 / PI);
  Ayz[0] = (atan2(RyEst[0], RzEst[0])) * (180 / PI);

  pastMillis = currentMillis;
  currentMillis = millis();
  float dt = currentMillis - pastMillis;
  Axz[1] = Axz[0] + (RateAxz * (dt / 1000));
  Ayz[1] = Ayz[0] + (RateAyz * (dt / 1000));

  if (RzEst[0] <= 0.1)
  {
    RxGyro = RxEst[0];
    RyGyro = RyEst[0];
    RzGyro = RzEst[0];
  }
  else
  {

    //Get rotation angles from current position angles for Gyro compensation
    RxGyro = (sin(Axz[1] * (PI / 180))) / (sqrt(1.0000 + (cos(Axz[1] * (PI / 180)) * cos(Axz[1] * (PI / 180)) * tan(Ayz[1] * (PI / 180)) * tan(Ayz[1] * (PI / 180)))));
    RyGyro = (sin(Ayz[1] * (PI / 180))) / (sqrt(1.0000 + (cos(Ayz[1] * (PI / 180)) * cos(Ayz[1] * (PI / 180)) * tan(Axz[1] * (PI / 180)) * tan(Axz[1] * (PI / 180)))));

    if (RzEst[0] >= 0) {
      Sign = 1;
    }
    else Sign = -1;

    RzGyro = Sign * (sqrt(1 - (RxGyro * RxGyro) - (RyGyro * RyGyro)));

    Rgyro = sqrt((RxGyro * RxGyro) + (RyGyro * RyGyro) + (RzGyro * RzGyro));

    RxGyro = RxGyro / Rgyro;
    RyGyro = RyGyro / Rgyro;
    RzGyro = RzGyro / Rgyro;
  }

  RxEst[1] = (RxAcc + (RxGyro * wGyro)) / (1 + wGyro);
  RyEst[1] = (RyAcc + (RyGyro * wGyro)) / (1 + wGyro);
  RzEst[1] = (RzAcc + (RzGyro * wGyro)) / (1 + wGyro);

  Rest = sqrt((RxEst[1] * RxEst[1]) + (RyEst[1] * RyEst[1]) + (RzEst[1] * RzEst[1]));

  RxEst[1] = RxEst[1] / Rest;
  RyEst[1] = RyEst[1] / Rest;
  RzEst[1] = RzEst[1] / Rest;

  pitch = atan2(-RxEst[1], sqrt((RyEst[1] * RyEst[1]) + (RzEst[1] * RzEst[1])));
  roll = atan2(RyEst[1], RzEst[1]);

  float xM = (MagX * cos(pitch)) + (MagZ * sin(pitch));
  float yM = (MagX * sin(roll) * sin(pitch)) + (MagY * cos(roll)) - (MagZ * sin(roll) * cos(pitch));

  compHeading = (atan2(yM, xM) * (180 / PI));
  compHeading += declination;
  if (compHeading < 0) compHeading += 360;
  if (compHeading > 360) compHeading -= 360;

//  compHeading = 0.01*compHeading + 0.99*lastHeading;
//  lastHeading = compHeading;

  /*total = total - readings[readIndex];
  readings[readIndex] = compHeading;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  average = total / numReadings;
  if (average > 360) {
    average = average - 360;
  }
  if (average < 0) average += 360;
  if (readIndex >= numReadings) {
    readIndex = 0;
  } */

  RxEst[0] = RxEst[1];
  RyEst[0] = RyEst[1];
  RzEst[0] = RzEst[1];
}
