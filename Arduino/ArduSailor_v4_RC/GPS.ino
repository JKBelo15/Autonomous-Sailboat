#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;

#define serialgps Serial1
int c;
float latitude = 0.0;
float longitude = 0.0;
unsigned long chars;
unsigned short sentences, failed_checksum;


void initGPS() {
  serialgps.begin(9600);
}

void getGPSData() {
  while (serialgps.available()) {
    c = serialgps.read();
    if (gps.encode(c)) {
      gps.f_get_position(&latitude, &longitude);
      y_component = longitude;
      x_component = latitude;
      gps.stats(&chars, &sentences, &failed_checksum);
      delay(1);
    }
    else {
      delay(1);
    }
  }
}
