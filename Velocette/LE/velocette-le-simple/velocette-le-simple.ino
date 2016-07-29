/*
  This is free and unencumbered software released into the public domain.

  Anyone is free to copy, modify, publish, use, compile, sell, or
  distribute this software, either in source code form or as a compiled
  binary, for any purpose, commercial or non-commercial, and by any
  means.

  In jurisdictions that recognize copyright laws, the author or authors
  of this software dedicate any and all copyright interest in the
  software to the public domain. We make this dedication for the benefit
  of the public at large and to the detriment of our heirs and
  successors. We intend this dedication to be an overt act of
  relinquishment in perpetuity of all present and future rights to this
  software under copyright law.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
  OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.

  For more information, please refer to <http://unlicense.org>
*/
#define DEBUG                 1
#define REPORT_INTERVAL       500000      // microseconds

#define MAX_RPM               3000.0

// The parameters for the advance curves.  They will advance from MIN_ADVANCE at MIN_RPM (and below) to MAX_ADVANCE at MAX_RPM (and above)
#define MAX_ADVANCE 30.0
#define MIN_ADVANCE 0.0
#define MIN_RPM     900
#define MAX_RPM     2000

#define PI          3.14159265359


#define YELLOW_LED 10  //  YELLOW_LED is used to give feedback on the shutter reading
#define SPARK_OUT  5   //  Pin that is connected to the 'Gate' on the transistor that drives the coil
#define INPUT_PIN  2   //  Input from the shutter sensor
#define GREEN_LED  11  //  GREEN_LED is not used at this time
#define SWITCH     12  //  The switch is not used at this time
#define POT_IN     A5  //  The input POT is used to adjust how much timing retard is used during starting
#define RPM_REF    14  //  Used for debugging
#define RPM_OUT    15  //  Used for debugging

// On every event of the shutter opening or closing we calculate the elapsed time since the previous event
// mics and lastmicros are used to store the elapsed time since the arduino booted until the current event, and the previous event 
volatile unsigned long mics = 1;
volatile unsigned long lastmicros = 0;

// In order to get the most stable output for the spark timing we set 'sparkArmed' to 1 between 50-BTDC and firing the spark
// This allows us to deffer non-critical functions at critical times by skipping them if sparkArmed is 1
volatile int sparkArmed = 0;

// nextspark is used to hold the time in the future at which the the spark should fire
volatile unsigned long nextspark = 0;

unsigned long reportTime = 0;

// degrees of advance
volatile double advance = 0;

// current rpm
volatile double rpm = 0;

void setup() {
  
  // Configure the serial port (USB) for the debug reporting
  Serial.begin(57600);
  Serial.print("Started\n");  
  
  pinMode(RPM_REF, OUTPUT);
  digitalWrite(RPM_REF, 0);

  pinMode(RPM_OUT, OUTPUT);
  digitalWrite(RPM_OUT, 0);
  
  // Configure the PINs we use
  pinMode(SPARK_OUT,OUTPUT);
  pinMode(INPUT_PIN,INPUT_PULLUP);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(GREEN_LED,OUTPUT);
  pinMode(POT_IN, INPUT);
  pinMode(SWITCH, INPUT);
  
  // Set up the shutter input to interrupt and call serviceRoutine() any time it changes  
  attachInterrupt(0, serviceRoutine, CHANGE);
  
}

void serviceRoutine(){
  lastmicros = mics;
  mics = micros();
  
  int inputstate = digitalRead(INPUT_PIN);
  
  if ( inputstate == HIGH ){   // TDC-50
      nextspark = mics + (((double)(mics - lastmicros) / (double)180.0) * (double)(50.0 - advance ) );
      sparkArmed = 1;
  }
  else { // 130
      if (rpm < MAX_RPM){
        digitalWrite(SPARK_OUT, HIGH);  // start charging the coil    
      }
  }
  digitalWrite(YELLOW_LED, inputstate);
  digitalWrite(RPM_OUT, inputstate);
}

void loop() {  
  if ( sparkArmed == 1 ) {  // This part of the loop should be kept as small as possible
    if ( nextspark <= micros() ) {
      
      digitalWrite(SPARK_OUT, LOW);  // fire the spark
      
      sparkArmed = 0;
      
      rpm = 0.5 / (double) ( (double)(mics - lastmicros) / 60000000.0);
      advance = calculateLinearAdvance(rpm);
      
    }
  }
  else { // not spark armed
  
    if ( reportTime < micros() ){
       report();
       reportTime = micros() + REPORT_INTERVAL;
    }

  }
  
}  

void report(){
  Serial.print("RPM=");
  Serial.print(rpm);
  Serial.print("\n");
  Serial.print("ADV=");
  Serial.print(advance);
  Serial.print("\n");
  Serial.print("\n");
}


// Calculate advance in degress based on RPM
// This version calculates a linear advance
double calculateLinearAdvance(double rpm){
  double a = (((MAX_ADVANCE - MIN_ADVANCE) / (MAX_RPM - MIN_RPM)) * ( rpm - MIN_RPM)) + MIN_ADVANCE;
  advance = min(max(MIN_ADVANCE, a), MAX_ADVANCE);
  return(advance);
}

// This version maps the advance curve to a 1/4 sine wave
double calculateSinAdvance(double rpm){
 advance = (sin(((min(max(rpm, MIN_RPM), MAX_RPM)-MIN_RPM)*(PI/(MAX_RPM-MIN_RPM)))-(PI/2.0)) * ((MAX_ADVANCE-MIN_ADVANCE)/2.0)) + ((MAX_ADVANCE-MIN_ADVANCE)/2.0);
 advance = advance + MIN_ADVANCE;
 return(advance);
}


