#include <math.h>

#define WindVaneX A6
#define WindVaneY A7
//#define PRINT_OUTPUT

//Calibrated Values of the Wind Vane
int WxMax = 604 ; //North of Magnet at HallEffect A0
int WxMin = 396 ; //South of Magnet at HallEffect A0
int WxMid = 504 ;
int WyMax = 604 ; //North of Magnet at HallEffect A1
int WyMin = 397 ; //South of Magnet at HallEffect A1
int WyMid = 506 ;

void initWindVane() {
  pinMode(WindVaneX, INPUT);
  pinMode(WindVaneY, INPUT);
}

float WindDirection() {
  float Wx = analogRead(WindVaneX);
  float Wy = analogRead(WindVaneY);

  if (Wx <= WxMid) {
    Wx = map(Wx, WxMin, WxMid, -180, 0);
  }
  else if (Wx > WxMid) {
    Wx = map(Wx, WxMid, WxMax, 0, 180);
  }
  if (Wy <= WyMid) {
    Wy = map(Wy, WyMin, WyMid, -180, 0);
  }
  else if (Wy >= WyMid) {
    Wy = map(Wy, WyMid, WyMax, 0, 180);
  }
  
  float val = -atan2(Wx,Wy);
  val = val *(180/PI);
  
  return val;
}
