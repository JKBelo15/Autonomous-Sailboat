/* ArduSailor_v4: Program for Autonomous Sailing (by: Jerome Kim G. Belo)

                 LIST OF COORDINATES
  -----------------------------------------------------------
   Wild Life 1              (14.6505    ,  121.0434444)
   Wild Life 2              (14.6506384 ,  121.0429095)
   JEB                      (14.593877  ,  121.087899 )
   Jollibee along JEB       (14.591024  ,  121.086155 )
   Bahay                    (14.546715  ,  121.105812 )
   Tapsihan                 (14.546476  ,  121.105662 )
   Jade's                   (14.532789  ,  121.155158 )
   PERC                     (14.604659  ,  120.988271 )
  -----------------------------------------------------------

                        LIST OF SERIAL COMMANDS
  ----------------------------------------------------------------------------
      a                      Automatic/Autonomous Mode
      b                      Rudder & Sail Test
        f                    Turn Rudder Left     (Under Rudder & Sail Test)
        g                    Turn Rudder Straight (Under Rudder & Sail Test)
        h                    Turn Rudder Right    (Under Rudder & Sail Test)
      c                      Input Waypoint Mode  (Enter Latitude, Longitude)
      d                      Heading Output Test
      x                      Stop                 (Any Command)
      y                      Data Serial Out
      z                      Simple Serial Out
  ----------------------------------------------------------------------------

*/
#include <Wire.h>

#define NO_RC //uncomment this line if Remote Control is not present

//---------------Boolean Logic Variables-------------------//

#define MODE_AUTO        true
#define MODE_MANUAL      false
boolean mode = MODE_MANUAL; // set default mode on start up

#define Simple           false
#define Complex          true

//---------------------------------------------------------//

//----------------------------------Adjust Waypoints Here--------------------------------------------------//

const int NoOfWaypoints = 4; //change this base on how many coordinates you want to pursue x2
float Waypoint[NoOfWaypoints] = {14.6505, 121.0434444, 14.6506384, 121.0429095}; //place coordinates here

//--------------------------------------------------------------------------------------------------------//

//--GPS and Coordinates Variables--//

float TrueWaypoint[2];
float StartPoint[2];
float InputPoint[2];
float x_component = 0;
float y_component = 0;
float distance;
unsigned int nw = 0;

//--------------------------------//

//----Heading Variables---//

float Desired_Heading;
float compHeading;
float Turning_Angle;
float Real_Turning_Angle;
unsigned int average = 0;

//-----------------------//

float WindDir; //Wind Direction

//----Serial Command Variables----//

int talk_time = 0;
int previous_talk_time = 0;
char command = 'x';
char md;
bool auto_pilot = 0;
bool ip = true;
bool SerialOut = Complex;
String fl;

//-------------------------------//

//-------Servo Control-------//

#define OUT5 8  //Sail Pin
#define OUT6 7  //Rudder Pin

//---------------------------//

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(OUT5, OUTPUT);
  pinMode(OUT6, OUTPUT);

  Serial.println("Initializing...");

  initGPS(); //initialize GPS

  initWindVane(); //initialize Wind Vane
  init_servo();

  initMagnet(); //initialize Magnetometer
  initMPU6050(); //initialize MPU6050 (Accelerometer & Gyroscope)

  calibrate();
  getInitSensVal(); //get initial sensor values
}

void loop() {
#ifndef NO_RC
  checkMode();
  if (mode == MODE_MANUAL) {
    Manual();
    talk_time = millis();
    if (talk_time - previous_talk_time >= 500) {
      Serial.println("Manual Mode...");
      previous_talk_time = talk_time;
    }
  }
  else {
#endif
    if (Serial.available()) {
      command = Serial.read();
    }
    if (command == 'z') {
      SerialOut = Simple;
      Serial.println("Simple Serial Out...");
      delay(2000);
    }
    else if (command == 'y') {
      SerialOut = Complex;
      Serial.println("Data Serial Out...");
      delay(2000);
    }
    else if (command == 'b') {
      auto_pilot = false;
      Serial.println("Command Mode: ");
      while (command == 'b') {
        adjust_sail(WindDirection());
        if (Serial.available()) {
          md = Serial.read();
        }
        if (md == 'a') {
          command = 'a';
          md = 0;
        }
        else if (md == 'x') {
          Serial.println("Exiting Command Mode...");
          command = 'x';
          md = 0;
        }
        else if (md == 'f') {
          Serial.println("Turn Rudder Left...");
          turnRudder(135);
        }
        else if (md == 'h') {
          Serial.println("Turn Rudder Right...");
          turnRudder(45);
        }
        else if (md == 'g') {
          Serial.println("Turn Rudder Straight...");
          turnRudder(90);
        }
      }
    }
    else if (command == 'd') {
      Serial.println("Heading Test...");
      bool cm = true;
      while (cm) {
        if (Serial.available()) {
          md = Serial.read();
        }
        if (md == 'x') {
          cm = false;
        }
        getHeading();
        talk_time = millis();
        if (talk_time - previous_talk_time >= 1000) {
          Serial.print(compHeading);
          //      Serial.print("  ");
          //      Serial.println(average);
          Serial.println();
          previous_talk_time = talk_time;
        }
      }
    }
    else if (command == 'a') {
      auto_pilot = true;
      Serial.println("Auto Mode...");
    }
    else if (command == 'c') {
      if (ip) {
        InputPoint[0] = 0;
        InputPoint[1] = 0;
      }
      auto_pilot = false;
      Serial.println("Input Mode...");
      delay(1000);
      nw = (NoOfWaypoints / 2) + 1;
      while (InputPoint[0] == 0 || InputPoint[1] == 0) {
        stop_sail(WindDirection());
        if (Serial.available()) {
          fl = Serial.readString();
        }
        float f = fl.toFloat();
        if (f <= 15 && f > 0) {
          InputPoint[0] = f;
        }
        else if (f >= 120 && f <= 150) {
          InputPoint[1] = f;
        }
        if (InputPoint[0] > 0 && InputPoint[1] > 0) {
          Serial.println("Got It!...");
          delay(1000);
          ip = false;
          Serial.print("Going to   ");
          delay(1000);
        }
        talk_time = millis();
        if (talk_time - previous_talk_time >= 1000) {
          Serial.print(InputPoint[0], 6);
          Serial.print(", ");
          Serial.println(InputPoint[1], 6);
          previous_talk_time = talk_time;
        }
        f = 0;
      }
      command == 'a';
    }
    else if (command == 'x') {
      auto_pilot = false;
      Serial.println("Awaiting Command...");
      delay(1000);
      command = 'x';
    }
    if (auto_pilot) {
      ip = true;
      if (x_component == 0 || y_component == 0) {
        stop_sail(WindDirection());
        Serial.println("Waiting GPS Data...");
        getGPSData();
        StartPoint[0] = x_component;
        StartPoint[1] = y_component;
        delay(1000);
        checkWaypoint();
      }
      else {
        checkWaypoint();
        get_distance(x_component, y_component, TrueWaypoint[0], TrueWaypoint[1]);
        if (distance <= 20) {
          stop_sail(WindDirection());
          Serial.print("Waypoint Reached -- ");
          Serial.print("GPS Position: ");
          Serial.print(x_component, 6);
          Serial.print(",");
          Serial.print(y_component, 6);
          Serial.print("  ");
          Serial.print("Waypoint: ");
          Serial.print(TrueWaypoint[0], 6);
          Serial.print(", ");
          Serial.print(TrueWaypoint[1], 6);
          Serial.print("  ");
          Serial.print("Distance(m): ");
          Serial.println(distance, 2);
          delay(3000);
          Serial.println("Adjusting Waypoint...");
          delay(3000);
          nw = nw + 1;
          if (nw >= NoOfWaypoints) {
            nw = 0;
            InputPoint[0] = 0;
            InputPoint[1] = 0;
          }
          checkWaypoint();
        }
        getGPSData();
        get_DesiredHeading();
        getHeading();
        Turning_Angle = compHeading - Desired_Heading;
        if (Turning_Angle > 180) Turning_Angle = Turning_Angle - 360;
        if (Turning_Angle < -180) Turning_Angle = Turning_Angle + 360;
        check_no_go();
        adjust_rudder(Real_Turning_Angle);
        adjust_sail(WindDirection());

        if (SerialOut == Complex) {
          talk_time = millis();
          if (talk_time - previous_talk_time >= 1000) {
            Serial.print("GPS Location: ");
            Serial.print(x_component, 6);
            Serial.print(",");
            Serial.print(y_component, 6);
            Serial.print("  ");
            Serial.print("Heading: ");
            Serial.print(compHeading, 2);
            Serial.print("  ");
            Serial.print("DesHeading: ");
            Serial.print(Desired_Heading, 2);
            Serial.print("  ");
            Serial.print("Turning Angle: ");
            Serial.print(Turning_Angle, 2);
            Serial.print("  ");
            Serial.print("Wind Dir: ");
            Serial.print(WindDir);
            Serial.print("  ");
            Serial.print("Distance(m): ");
            Serial.print(distance, 2);
            Serial.print("  ");
            Serial.print("Waypoint #");
            Serial.print(nw + 1);
            Serial.println();
            previous_talk_time = talk_time;
          }
        }
        else if (SerialOut == Simple) {
          talk_time = millis();
          if (talk_time - previous_talk_time >= 1000) {
            if (Real_Turning_Angle < 0) {
              Serial.print("Turning Right...");
            }
            else if (Real_Turning_Angle > 0) {
              Serial.print("Turning Left...");
            }
            else {
              Serial.print("Going Straight...");
            }
            Serial.print("  ");
            Serial.print("Distance(m): ");
            Serial.print(distance, 2);
            Serial.print("  ");
            Serial.print("Waypoint #");
            Serial.print(nw + 1);
            Serial.println();
            previous_talk_time = talk_time;
          }
        }
      }
    }
  }
#ifndef NO_RC
}
#endif

void checkWaypoint() {
  if (nw < ((NoOfWaypoints / 2) + 1)) {
    TrueWaypoint[0] = Waypoint[2 * nw];
    TrueWaypoint[1] = Waypoint[(2 * nw) + 1];
  }
  else if (nw >= ((NoOfWaypoints / 2) + 1)) {
    TrueWaypoint[0] = InputPoint[0];
    TrueWaypoint[1] = InputPoint[1];
  }
}

void get_distance(float x1, float y1, float x2, float y2) {
  float a = ((sin((y2 - y1) * (PI / 180) / 2)) * (sin((y2 - y1) * (PI / 180) / 2))) + cos(x1 * (PI / 180)) * cos(x2 * (PI / 180)) * (sin((x2 - x1) * (PI / 180) / 2)) * (sin((x2 - x1) * (PI / 180) / 2));
  distance = 6378.137 * (2 * atan2(sqrt(a), sqrt(1 - a)));
  distance = distance * 1000;
}

void get_DesiredHeading () {
  Desired_Heading = atan2((TrueWaypoint[1] - y_component), (TrueWaypoint[0] - x_component));
  if (Desired_Heading < 0) Desired_Heading += 2 * PI;
  if (Desired_Heading > 2 * PI) Desired_Heading -= 2 * PI;
  Desired_Heading *= (180 / PI);
}

void check_no_go() {
  WindDir = WindDirection();
  if (WindDir <= 50 && WindDir >= 45) {
    if (Turning_Angle >= -100 && Turning_Angle <= 0) {
      Real_Turning_Angle = 0;
      Serial.print("Near No Go Zone...");
      Serial.print("  ");
    }
  }
  else if (WindDir >= -50 && WindDir <= -45) {
    if (Turning_Angle <= 100 && Turning_Angle >= 0) {
      Real_Turning_Angle = 0;
      Serial.print("Near No Go Zone...");
      Serial.print("  ");
    }
  }
  else if (WindDir > -45 && WindDir < 45) {
    Serial.println("Inside No Go Zone...");
    if (Turning_Angle <= 0) {
      turnRudder(45);
      turnSail(90);
      delay(3000);
    }
    if (Turning_Angle > 0) {
      turnRudder(135);
      turnSail(90);
      delay(3000);
    }
  }
  else {
    Real_Turning_Angle = Turning_Angle;
  }
}
