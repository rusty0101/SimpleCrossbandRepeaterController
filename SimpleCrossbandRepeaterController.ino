/* 
Crossband repeater controller project

Copyright 2017 Jay Russell Curry - kc0vcu
This software licensed under the GPL - If used in a product you distribute
you are obligated to follow the requirements of the GPL, which includes,
but is not limited to, linking to the GPL, a link to this source code, a
link to any source code modifications you have made to this code to get
it to work with your hardware.

About:
Using an arduino to build a legal crossband repeater to be used in the
field on rare occasions when a handheld radio may have trouble getting
to a standard repeater, and alternatively when the handheld may not
be able to get good recpetion from the repeater but the crossband
repeater can be set up to provide good coverage into and out of the
area the handheld is being used in and the repeater that is expected to
be being used.

No explanation of how to set up radios will be made in this document.
This is highly dependent on the radios in use, and outside of the
scope here.

The arduino has somewhere in the neighborhood of 13 usable digital IO
pins, along with another 5-7 analog input digital output pins that may
be used. There are between 5 and 7 digital IO pins that also provide
PWM outputs that can be used to either generate an apparent analog level
or as may be used in this project to provide a signal that can be used
to generate an audio tone.

A legal Crossband repeater does not need to provide an ID from the local
users to the repeater, as the local user is legaly obligated to ID. It
does need to provide an ID if it is retransmitting the output of the
repeater into the local handheld user's area however. This ID needs to
happen ever 590 seconds (legally every 10 minutes, but we work somewhat
conservatively here) that there is audio being transmitted into the
local area, with the only delay being due to the local radio being 
retransmitted to the repeater.

{ Research required, We may need to ID to the repeater if the controller
is being used by multiple users in the local area, as it is the owner
of the crossband repeater controller that is responsible for this
device, and legally oblicated to provide an ID. The above notes are
specific to a single user crossband full duplex crossband repeater.
}

Each radio is expected to have a separate pair of control leads, as well
as a pair of audio leads. There are situations where the control leads
either do not exist, or are not isolated from the audio leads, however 
this document does not identify how to address this situation.

The two control leads per radio will be the COR or Carier Or Receive, 
PTT or Push To Talk leads. Initially we will not be building the audio
interface, but we will build in hooks for that. Essentially that
interface will need to support three audio inputs, and two audio outputs
with control leads that switches the received audio from one radio to
the tx audio of the other radio, and vice versa, with one lead also 
controlling the connection from the IDer to the.

As a result we need to allocate link ports pretty much right away. Each
radio needs to have three leads allocated, and the controller needs a
few leads as well. As we want to be able to make some configuration
changes in the field, I'm going to enable the RX/TX serial digital IO
pins 0 and 1 so that someone with a laptop can re-configure that way.
Pin ~3 is a PWM pin, so let's use that to send CW, either through a tone
generator (later project) or by using the PWM capability passed through 
a capacitor to genrate tones directly. Pin's 2 and 4 will be used to 
indicate what direction any retransmtion is happening, and may also be 
used to control the link to the IDer path.

A later project using I2C or another interface, may be used to interface
a dtmf sensor to allow for handheld remote control of the controller. 
Other later updates include console re-configuration and the ability to
control the controller via wifi or console, as well as the ability to 
get the current time from the console, an ntp server, a gps receiver or
a realtimeclock i2c device. A GPS receiver may be used to add location
information to the ID string. This may also be done with the time, though
initially I'm more interested in adding that for logging.

Pins ~5, ~6 and 7 will be used for radio 0, which by default will be the
repeater. Pins 8, ~9 & ~10 will be used for radio 1 which by deault will
be the local radio. (This may also be the local connection for an
echolink link station, however the full configuration of that is beyond
the scope of this document.)

I have built this code to run on an Arduino Uno or Arduino Leonardo. It
should run w/o significant changes on most of the arduino line, however 
different boards offer different features. An arduino leonardo pro micro
does not use pin 13 for an onboard led example. There are workarounds.
An Arduino Due has analog out pins that can be used to provide a better
tone source for an IDer, however is not an option for the Arduino IDE
that I'm working with.

We will be using pins 11 and 12 to control the state of the controller,
with pin 11 enabling crossband capability, and 12 switching from half
duplex or tx local to repeater only, to full duplex of retransmit both
directions and enable the id'r. Pin 13 will be used for now to switch
the id'r path.

These buttons can be overridden through software.

Initial setup is to use pins 5 an 8 as COR for radios 0 and 1, and pins
6 and 9 as PTT. Pins 7 and 10 will control switching the audio paths, 
(during development this will be indicated by turning on an LED when the
path is in use.) (COR is the indication that the receiver is receiving a
signal that needs to be retransmitted. It may show up on various radios as
squelch or some other rx-state) This is an active low lead. If you need to
drive this from a source that is active high, you should be able to invert
the signal with a switching transistor.

If both radio0cor and radio1cor are live at the same time, we do not tx
from radio1 to radio0. 

The code for sending Morse Code was derived from the code by Mark 
Vanderwettering K6HX, and may be found online at: 
http://brainwagon.org/2009/11/14/another-try-at-an-arduino-based-morse-beacon/

My modifications are to use analogWrite on a pwm pin to drive a piezo 
electric buzzer, and which pin to use for the output.
 */

// Pin relabels.
#define Ser_tx 0
#define Ser_rx 1
#define ind0 2
#define idr 3 // (awrite for pwm, write for external oscilator)
#define ind1 4
#define radio0cor 5
#define radio0ptt 6
#define radio0switch 7
#define radio1cor 8
#define radio1ptt 9
#define radio1switch 10
#define CrossbandEnable 11
#define DuplexEnable 12
#define idr_switch 13

// Variables
long idertimer = 1000;
int iderneed = 1;
int path01 = 0;
int path10 = 0;
int pathid = 0;
String idString = "KC0VCU"; // over ride from sram if exists, later
                            // cwfont will be loaded from a font file
                            // cw speed and interval variables will by
                            // default be 20 15, but override from sram
                            // if set there, will be done if needed.
int cwvalue = 490; // awrite value to write to generate cw tone, aim for
                   // 700 hz, but range to work with is 1-1023
struct t_mtab { char c, pat; } ;

struct t_mtab morsetab[] = {
  	{'.', 106},
	{',', 115},
	{'?', 76},
	{'/', 41},
	{'A', 6},
	{'B', 17},
	{'C', 21},
	{'D', 9},
	{'E', 2},
	{'F', 20},
	{'G', 11},
	{'H', 16},
	{'I', 4},
	{'J', 30},
	{'K', 13},
	{'L', 18},
	{'M', 7},
	{'N', 5},
	{'O', 15},
	{'P', 22},
	{'Q', 27},
	{'R', 10},
	{'S', 8},
	{'T', 3},
	{'U', 12},
	{'V', 24},
	{'W', 14},
	{'X', 25},
	{'Y', 29},
	{'Z', 19},
	{'1', 62},
	{'2', 60},
	{'3', 56},
	{'4', 48},
	{'5', 32},
	{'6', 33},
	{'7', 35},
	{'8', 39},
	{'9', 47},
	{'0', 63}
} ;

#define N_MORSE  (sizeof(morsetab)/sizeof(morsetab[0]))

#define SPEED  (15)
#define DOTLEN  (1200/SPEED)
#define DASHLEN  (3*(1200/SPEED))
                   
                   
long delaytil = 0; // we'll be doing non 
boolean sendingcw = false;
long eventms = 0;
boolean ledon = true;
long nowms;

// Functions

void dash() {
  analogWrite(idr, cwvalue) ;
  delay(DASHLEN);
  analogWrite(idr, 0) ;
  delay(DOTLEN) ;
}

void dit() {
  analogWrite(idr, cwvalue) ;
  delay(DOTLEN);
  analogWrite(idr, 0) ;
  delay(DOTLEN);
}

void send(char c) {
  int i ;
  if (c == ' ') {
//    Serial.print(c) ;
    delay(7*DOTLEN) ;
    return ;
  }
  for (i=0; i<N_MORSE; i++) {
    if (morsetab[i].c == c) {
      unsigned char p = morsetab[i].pat ;
//      Serial.print(morsetab[i].c) ;

      while (p != 1) {
          if (p & 1)
            dash() ;
          else
            dit() ;
          p = p / 2 ;
      }
      delay(2*DOTLEN) ;
      return ;
    }
  }
  /* if we drop off the end, then we send a space */
//  Serial.print("?") ;
}

void enablePath(int path2enable) {
  if (path2enable == 0) {
    //path to enable is 0 to 1, repeater to local
    if (digitalRead(CrossbandEnable) == LOW and digitalRead(DuplexEnable) == LOW) {
      path01 = 1;
      digitalWrite(radio1ptt, HIGH);
      digitalWrite(radio1switch, HIGH);
      digitalWrite(ind0, HIGH);
      digitalWrite(ind1, LOW);
//      Serial.println("path 01 up");
      if (iderneed == 0) { 
        iderneed = 1;
      }
    }
  } else {
    //path to enable is 1 to 0, local to repeater
    if (digitalRead(CrossbandEnable) == LOW) {
      path10 = 1;
      digitalWrite(radio0ptt, HIGH);
      digitalWrite(radio0switch, HIGH);
      digitalWrite(ind0, LOW);
      digitalWrite(ind1, HIGH);
//      Serial.println("path 10 up");      
    }
  }
}

void disablePath(int path2disable) {
  if (path2disable == 0) {
    // path to disable is 0 to 1, repeater to local
    path01 = 0;
    if (iderneed == 1 and nowms > idertimer) {sendID();} else {
      digitalWrite(radio1ptt, LOW);
      digitalWrite(radio1switch, LOW);
      digitalWrite(ind0, LOW);
      digitalWrite(ind1, LOW);
//      Serial.println("path 01 down");
    } 
  } else {
    // path to disalbe is 1 to 0, local to repeater
    path10 = 0;
    digitalWrite(radio0ptt, LOW);
    digitalWrite(radio0switch, LOW);
    digitalWrite(ind0, LOW);
    digitalWrite(ind1, LOW);
//    Serial.println("path 10 down");
  }
}
void sendmsg(char *str) {
  while (*str)
    send(*str++) ;
//  Serial.println("");
}

void sendID(){
//  Serial.println("kc0vcu");
  // stub to add sending cw as needed.
  Serial.println("Sending ID");
  sendmsg("KC0VCU");
  iderneed = 0;
  disablePath(0);
}


void setup(){
  Serial.begin(9600);
  pinMode(ind0, OUTPUT);
  pinMode(ind1, OUTPUT);
  pinMode(idr, OUTPUT);
  pinMode(radio0cor, INPUT);
  digitalWrite(radio0cor, HIGH);  // set pull-up high
  pinMode(radio0ptt, OUTPUT);
  digitalWrite(radio0ptt, LOW);
  pinMode(radio0switch, OUTPUT);
  digitalWrite(radio0switch, LOW);
  pinMode(radio1cor, INPUT);
  digitalWrite(radio1cor, HIGH);   // set pull-up high
  pinMode(radio1ptt, OUTPUT);
  digitalWrite(radio1ptt, LOW);
  pinMode(radio1switch, OUTPUT);
  digitalWrite(radio1switch, LOW);
  pinMode(CrossbandEnable, INPUT);
  digitalWrite(CrossbandEnable, HIGH);   // set pull-up high
  pinMode(DuplexEnable, INPUT);
  digitalWrite(DuplexEnable, HIGH);     // set pull-up high
  pinMode(idr_switch, OUTPUT);
  digitalWrite(idr_switch, LOW);
}

void loop() {
  nowms = millis();
  if (iderneed == 0 ) {idertimer = nowms + 580000;}
  // Two basic possibilities, we're not transmitting, and we need to, or
  // we are transmitting and we don't need to.
  if (path01 == 0 and path10 == 0) {
    // not transmitting, do we need to?
    if (digitalRead(radio0cor) == LOW) { // need to tx from repeater to local
      enablePath(0);
      Serial.println("repeater>local");
    } else if (digitalRead(radio1cor) == LOW) { // need to tx from local to repeater
    // tx from local to repeater
      enablePath(1);
      Serial.println("local>repeater");
    }
  }
  if (path01 == 1 and digitalRead(radio0cor) == HIGH) {
    //don't need to tx from repeater to local any more
    disablePath(0);
    Serial.println("Listening");
  }
  if (path10 == 1 and digitalRead(radio1cor) == HIGH) {
    //don't need to tx from local to repeater any more
    disablePath(1);
      Serial.println("Listening");
    if (nowms > idertimer and digitalRead(CrossbandEnable) == LOW and digitalRead(DuplexEnable) == LOW) {
      enablePath(0); // enable path from repeater to local to perform id function
      sendID();
      disablePath(0);
    }
  }
  if (nowms > idertimer and digitalRead(CrossbandEnable) == LOW and digitalRead(DuplexEnable) == LOW) {
    enablePath(0);
    disablePath(0);
  }
  if (nowms > eventms) {
    eventms = nowms + 1000;
    ledon = !ledon;
    digitalWrite(idr_switch, ledon);
    Serial.println("Sending ID");
/*    // Debuging   code, push state of sensors to serial port.
 *       
      Serial.print("r0cor: ");
      Serial.print(digitalRead(radio0cor));
      Serial.print(" r1cor: ");
      Serial.print(digitalRead(radio1cor));
      Serial.print(" xBand: ");
      Serial.print(digitalRead(CrossbandEnable));
      Serial.print(" Duplex: ");
      Serial.println(digitalRead(DuplexEnable));
      */
  }
}  
