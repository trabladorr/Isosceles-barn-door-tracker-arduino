#include "Arduino.h"
#include <avr/sleep.h>

class GuttedCheapStepper
//https://github.com/tyhenry/CheapStepper
//with all timing functionality removed
{

  public:
    GuttedCheapStepper (int in1, int in2, int in3, int in4) : pins( {
      in1, in2, in3, in4
    }) {
      for (int pin = 0; pin < 4; pin++) {
        pinMode(pins[pin], OUTPUT);
      }
    }

    void setTotalSteps (int numSteps) {
      totalSteps = numSteps;
    }

    void step(bool clockwise) {

      if (clockwise) seqCW();
      else seqCCW();
    }

    void off() {
      for (int p = 0; p < 4; p++)
        digitalWrite(pins[p], 0);
    }

  private:

    int pins[4]; // defaults to pins {8,9,10,11} (in1,in2,in3,in4 on the driver board)

    int stepN = 0; // keeps track of step position
    // 0-4095 (4096 mini-steps / revolution) or maybe 4076...
    int totalSteps = 4096;
    int seqN = -1; // keeps track of sequence number

    void seqCW () {
      seqN++;
      if (seqN > 7) seqN = 0; // roll over to A seq
      seq(seqN);

      stepN++; // track miniSteps
      if (stepN >= totalSteps) {
        stepN -= totalSteps; // keep stepN within 0-(totalSteps-1)
      }
    }

    void seqCCW () {
      seqN--;
      if (seqN < 0) seqN = 7; // roll over to DA seq
      seq(seqN);

      stepN--; // track miniSteps
      if (stepN < 0) {
        stepN += totalSteps; // keep stepN within 0-(totalSteps-1)
      }
    }

    void seq (int seqNum) {

      int pattern[4];
      // A,B,C,D HIGH/LOW pattern to write to driver board

      switch (seqNum) {
        case 0:
          {
            pattern[0] = 1;
            pattern[1] = 0;
            pattern[2] = 0;
            pattern[3] = 0;
            break;
          }
        case 1:
          {
            pattern[0] = 1;
            pattern[1] = 1;
            pattern[2] = 0;
            pattern[3] = 0;
            break;
          }
        case 2:
          {
            pattern[0] = 0;
            pattern[1] = 1;
            pattern[2] = 0;
            pattern[3] = 0;
            break;
          }
        case 3:
          {
            pattern[0] = 0;
            pattern[1] = 1;
            pattern[2] = 1;
            pattern[3] = 0;
            break;
          }
        case 4:
          {
            pattern[0] = 0;
            pattern[1] = 0;
            pattern[2] = 1;
            pattern[3] = 0;
            break;
          }
        case 5:
          {
            pattern[0] = 0;
            pattern[1] = 0;
            pattern[2] = 1;
            pattern[3] = 1;
            break;
          }
        case 6:
          {
            pattern[0] = 0;
            pattern[1] = 0;
            pattern[2] = 0;
            pattern[3] = 1;
            break;
          }
        case 7:
          {
            pattern[0] = 1;
            pattern[1] = 0;
            pattern[2] = 0;
            pattern[3] = 1;
            break;
          }
        default:
          {
            pattern[0] = 0;
            pattern[1] = 0;
            pattern[2] = 0;
            pattern[3] = 0;
            break;
          }
      }

      // write pattern to pins
      for (int p = 0; p < 4; p++) {
        digitalWrite(pins[p], pattern[p]);
      }
    }
};






//polynomial regression result's polynomial factors
double f0 = 2228.5;
double f1 = -3.2547E-6;
double f2 =  8.64583E-12;

double calc_nominal_delay(double steps) {
  return f0 + steps * f1 + steps * steps * f2;
}

double mm_to_steps(float mm) {
  return mm * 4096 * 15.3333 / 2;
}

// 2, 3, 4, 5 are the arduino pins the stepper control board is connected to
GuttedCheapStepper stepper (2, 3, 4, 5);

//depends on your gearing
boolean moveClockwise = false;

//stepper steps taken so far
unsigned long steps = 0;
//for timing stats
unsigned long start_time = 0;

//the time to wait between steps
double interstep = 0;

unsigned long  init_steps = mm_to_steps(17.5); //Starting at 17.5mm, 5.1 degrees 
//unsigned long max_steps = 10000; //10000 steps for timing_stats(), to find calculation overhead
unsigned long max_steps = mm_to_steps(100); //100mm is a good limit for 150mm leadscrew, to avoid running out of screw and damaging the camera

//prints timing stats over the run, to help find timing errors
void timing_stats(){
  unsigned long end_time = micros();
    
  Serial.begin(9600);
  Serial.print((max_steps - init_steps));
  Serial.print(" step run done, started at ");
  Serial.print(init_steps);
  Serial.print(" steps, ");
  Serial.print(calc_nominal_delay(init_steps));
  Serial.println(" delay.");

  Serial.print("Avg exec usec/step:\t");
  double avg_exec_time_per_step = (end_time - start_time) / (max_steps - init_steps);
  Serial.println(avg_exec_time_per_step);
  Serial.print("Avg overhead usec/step:\t");
  Serial.println(avg_exec_time_per_step - (calc_nominal_delay(init_steps)+calc_nominal_delay(steps))/2);
}

//enter full sleep at end, to reduce idle power usage
void turn_off(){
  stepper.off();
  delay(2000);
  noInterrupts();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
}

void setup() {
  steps = init_steps;
  max_steps += init_steps;
  start_time = micros();
}

//main loop
void loop() {
  if (steps < max_steps) {
    stepper.step(moveClockwise);
    
    steps ++;
    interstep = calc_nominal_delay(steps);
    //calculation overhead trimming, timing_stats() helps find this
    //there is also a small delay fluctuation depending on the steps variable, the second part helps remove that
    //modify this trim also to fix any drift due to crystal frequency inaccuracy, by timing the rotation of the lead screw gear
    interstep -= 123.1 - steps*2E-6; 
    
    //delayMicroseconds should be rather accurate (https://github.com/arduino/ArduinoCore-avr/issues/118), but even if it isn't, the brute force trim above certainly helps!
    delayMicroseconds(interstep);
  }
  else {
    
    timing_stats();

    turn_off();
  }
}
