/*
 * Test Code for RC control for Sailboat
 * written by: gHiL
 * 06/26/2019 
 * 
 * Using Taranis QX7s
 */

/***************************INPUTS**********************/

#define PIN_SIG_RUDDER   (33) /***< Signal from RC to control rudder*/
#define PIN_SIG_WINCH    (34) /***< Signal from RC to control winch*/
#define PIN_SIG_CH5      (35) /***< Signal from RC channel 5 to control mode*/
// pos 1-982
// pos 2-1493
// pos 3-1998

// signal length of command
#define SIG_LEN_CM1     (900)
#define SIG_LEN_CM2     (1400)
#define SIG_LEN_CM3     (2050)

// signal from remote
unsigned long sig_rc_winch = 0;
unsigned long sig_rc_rudder = 0;
unsigned long sig_rc_ch5 = 0;

void init_RC() {
  pinMode(PIN_SIG_WINCH, INPUT);
  pinMode(PIN_SIG_RUDDER, INPUT);
  pinMode(PIN_SIG_CH5, INPUT);
}

void checkMode() {
  sig_rc_winch = pulseIn(PIN_SIG_WINCH, HIGH);
  sig_rc_rudder = pulseIn(PIN_SIG_RUDDER, HIGH);
  sig_rc_ch5 = pulseIn(PIN_SIG_CH5, HIGH);  

  if(sig_rc_ch5 > SIG_LEN_CM1 && sig_rc_ch5 < SIG_LEN_CM2){    
    mode = MODE_MANUAL;
  }
  else if(sig_rc_ch5 > SIG_LEN_CM2){
    mode = MODE_AUTO;
  }else{
    mode = MODE_MANUAL;
  }
}

void Manual() {
  sailControl(sig_rc_winch);
  rudderControl(sig_rc_rudder);
}
